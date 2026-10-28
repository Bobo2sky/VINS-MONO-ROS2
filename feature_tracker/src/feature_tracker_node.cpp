#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

// 图像输入格式:0 = sensor_msgs/msg/Image,1 = sensor_msgs/msg/CompressedImage
enum ImageFormat
{
    FORMAT_RAW = 0,
    FORMAT_COMPRESSED = 1
};
std::atomic<int> image_format{-1};

// 自动判断接收的图像数据格式:首条到达的消息锁定格式,
// 之后只处理该格式(raw 与 compressed 同时发布时防止重复处理)
bool acceptImageFormat(int format)
{
    int expected = -1;
    if (image_format.compare_exchange_strong(expected, format))
        return true;
    if (image_format.load() != format)
    {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true))
            RCUTILS_LOG_WARN("both raw and compressed image streams detected, only the %s stream is processed",
                             image_format.load() == FORMAT_RAW ? "raw" : "compressed");
        return false;
    }
    return true;
}

rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_img;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;

// ---- 两级流水线:callback 只入队(µs 级),解码线程与跟踪线程经队列衔接 ----
// 解码后的帧:decode/resize 计时随帧传递(替代原全局 g_decode_ms),
// img 已是跟踪分辨率(降采样 resize 在解码线程完成)
struct DecodedFrame
{
    cv::Mat img;                // 已 resize 到 (COL, ROW) 的灰度跟踪图
    std_msgs::msg::Header header;
    double decode_ms = 0.0;     // compressed: imdecode / raw: cv_bridge 转换耗时
    double resize_ms = 0.0;     // 降采样 resize 耗时(尺寸已一致时为 0)
};

// 丢旧保新策略(实时语义,延迟有界):
// - 解码前队列(raw_buf/comp_buf)深度 3:吸收回放/驱动启动时的帧突发与抖动,
//   解码(~20ms)远快于帧周期(100ms),积压可迅速消化
// - 解码后队列(decoded_buf)深度 1:跟踪是串行瓶颈(LK 帧间状态机),真超载时丢旧保新
// 丢弃计数 g_dropped 并入 30 帧汇总日志
queue<sensor_msgs::msg::Image::SharedPtr> raw_buf;            // 解码前(raw 路径)
queue<sensor_msgs::msg::CompressedImage::SharedPtr> comp_buf; // 解码前(compressed 路径)
queue<DecodedFrame> decoded_buf;                              // 解码后,待跟踪
std::mutex m_buf;
std::condition_variable con;
std::atomic<bool> running{true};
std::atomic<int> g_dropped{0};

// 入队(超深度丢旧保新)
template <typename MsgPtr>
void push_latest(queue<MsgPtr> &q, MsgPtr msg)
{
    {
        std::lock_guard<std::mutex> lg(m_buf);
        while (q.size() >= 3)
        {
            q.pop();
            g_dropped++;
        }
        q.push(std::move(msg));
    }
    con.notify_all();
}

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

// 跟踪阶段:消费解码后的帧(readImage / 发布 / 计时汇总),在跟踪线程执行。
// LK 光流是帧间串行状态机(下一帧依赖上一帧的 forw_pts/ids/track_cnt),
// 帧间不可并行;流水线只重叠解码与跟踪两个阶段。
// 时序状态(first_image_time 等)只在本线程访问,无需加锁
void process_image(const DecodedFrame &frame)
{
    const std_msgs::msg::Header &header = frame.header;
    const cv::Mat &track_img = frame.img;
    double t = header.stamp.sec + header.stamp.nanosec * (1e-9);
    if (first_image_flag)
    {
        first_image_flag = false;
        first_image_time = t;
        last_image_time = t;
        return;
    }

    // detect unstable camera stream
    if (t - last_image_time > 1.0 || t < last_image_time)
    {
        RCUTILS_LOG_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart->publish(restart_flag);
        return;
    }

    last_image_time = t;
    // frequency control
    if (round(1.0 * pub_count / (t - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (t - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = t;
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    cv::Mat show_img = track_img;
    // cv::imshow("img", show_img);
    // cv::waitKey(0);
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        RCUTILS_LOG_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(track_img.rowRange(ROW * i, ROW * (i + 1)), t);
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(track_img.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = track_img.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

    double pub_ms = 0.0;
    if (PUB_THIS_FRAME)
    {
        TicToc t_pub;
        pub_count++;
        sensor_msgs::msg::PointCloud::SharedPtr feature_points(new sensor_msgs::msg::PointCloud);
        sensor_msgs::msg::ChannelFloat32 id_of_point;
        sensor_msgs::msg::ChannelFloat32 u_of_point;
        sensor_msgs::msg::ChannelFloat32 v_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

        feature_points->header = header;
        feature_points->header.frame_id = "odom";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::msg::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }

        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);

        RCUTILS_LOG_DEBUG("publish %f, at %f", feature_points->header.stamp.sec + feature_points->header.stamp.nanosec * (1e-9), rclcpp::Clock().now().nanoseconds() * (1e-9));
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img->publish(*feature_points);

        if (SHOW_TRACK)
        {
            // 单次灰度→BGR 转换,画跟踪点后缩到 1/2 分辨率发布
            // (原实现做两次全图 GRAY2RGB 且发布全尺寸,8MP 下每帧 ~25MB 序列化)
            cv::Mat stereo_img;
            cv::cvtColor(show_img, stereo_img, CV_GRAY2BGR);

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                }
            }
            cv::Mat small_img;
            cv::resize(stereo_img, small_img, cv::Size(COL / 2, ROW / 2), 0, 0, cv::INTER_AREA);
            cv_bridge::CvImage out_img(header, sensor_msgs::image_encodings::BGR8, small_img);
            pub_match->publish(*(out_img.toImageMsg()));
        }
        pub_ms = t_pub.toc();
    }
    double total_ms = t_r.toc();
    RCUTILS_LOG_DEBUG("whole feature tracker processing costs: %fms", total_ms);

    // 30 帧窗口分段耗时汇总(INFO 级可见;decode/resize 由解码线程随帧传递)
    {
        static double sum_decode = 0, sum_resize = 0, sum_lk = 0, sum_reject = 0,
                      sum_mask = 0, sum_gftt = 0, sum_undist = 0, sum_pub = 0, sum_total = 0;
        static int frame_cnt = 0;
        sum_decode += frame.decode_ms;
        sum_resize += frame.resize_ms;
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            sum_lk += trackerData[i].t_lk_ms;
            sum_reject += trackerData[i].t_reject_ms;
            sum_mask += trackerData[i].t_mask_ms;
            sum_gftt += trackerData[i].t_gftt_ms;
            sum_undist += trackerData[i].t_undist_ms;
        }
        sum_pub += pub_ms;
        sum_total += total_ms;
        frame_cnt++;
        if (frame_cnt >= 30)
        {
            int dropped = g_dropped.exchange(0);
            RCUTILS_LOG_INFO("ft timing(ms, 30f avg): decode %.1f resize %.1f lk %.1f rejectF %.1f mask %.1f gftt %.1f undist %.1f pub %.1f | total %.1f | dropped %d",
                             sum_decode / frame_cnt, sum_resize / frame_cnt, sum_lk / frame_cnt,
                             sum_reject / frame_cnt, sum_mask / frame_cnt, sum_gftt / frame_cnt,
                             sum_undist / frame_cnt, sum_pub / frame_cnt, sum_total / frame_cnt, dropped);
            if (sum_total / frame_cnt > 100.0)
                RCUTILS_LOG_WARN("feature tracker avg total %.1fms exceeds 100ms frame budget",
                                 sum_total / frame_cnt);
            sum_decode = sum_resize = sum_lk = sum_reject = sum_mask = 0;
            sum_gftt = sum_undist = sum_pub = sum_total = 0;
            frame_cnt = 0;
        }
    }
    // 单帧超预算立即告警(10Hz 下预算 100ms)
    if (total_ms + frame.decode_ms + frame.resize_ms > 100.0)
        RCUTILS_LOG_WARN("feature tracker frame costs %.1fms (decode %.1fms + track %.1fms), exceeds 100ms budget",
                         total_ms + frame.decode_ms + frame.resize_ms, frame.decode_ms + frame.resize_ms, total_ms);
}

// 解码线程:raw 路径 cv_bridge 转 mono8 / compressed 路径 imdecode(含缩放解码),
// 并完成降采样 resize,产出跟踪分辨率的 DecodedFrame
void decode_loop()
{
    while (running.load())
    {
        sensor_msgs::msg::Image::SharedPtr raw_msg;
        sensor_msgs::msg::CompressedImage::SharedPtr comp_msg;
        {
            std::unique_lock<std::mutex> lk(m_buf);
            con.wait(lk, []
                     { return !raw_buf.empty() || !comp_buf.empty() || !running.load(); });
            if (!raw_buf.empty())
            {
                raw_msg = raw_buf.front();
                raw_buf.pop();
            }
            else if (!comp_buf.empty())
            {
                comp_msg = comp_buf.front();
                comp_buf.pop();
            }
            else
                break; // running = false 且无待处理消息
        }

        DecodedFrame frame;
        TicToc t_dec;
        if (raw_msg)
        {
            // raw 路径:cv_bridge 转 mono8(含 8UC1 特判)
            cv_bridge::CvImageConstPtr ptr;
            if (raw_msg->encoding == "8UC1")
            {
                sensor_msgs::msg::Image img;
                img.header = raw_msg->header;
                img.height = raw_msg->height;
                img.width = raw_msg->width;
                img.is_bigendian = raw_msg->is_bigendian;
                img.step = raw_msg->step;
                img.data = raw_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(raw_msg, sensor_msgs::image_encodings::MONO8);
            frame.img = ptr->image; // cv::Mat 引用计数共享,ptr 析构不影响
            frame.header = raw_msg->header;
        }
        else
        {
            // 压缩路径:imdecode 直接解码为灰度图(jpeg/png 均可)。
            // 解码阶段直接 1/DOWNSAMPLE 降尺度(libjpeg DCT scaling),免去全尺寸
            // 解码 + resize;仅压缩路径且倍数为 2/4/8 时生效,失败回退全尺寸
            cv::Mat buf(1, comp_msg->data.size(), CV_8UC1, const_cast<uint8_t *>(comp_msg->data.data()));
            int decode_flag = cv::IMREAD_GRAYSCALE;
            if (DECODE_REDUCED && DOWNSAMPLE == 2)
                decode_flag = cv::IMREAD_REDUCED_GRAYSCALE_2;
            else if (DECODE_REDUCED && DOWNSAMPLE == 4)
                decode_flag = cv::IMREAD_REDUCED_GRAYSCALE_4;
            else if (DECODE_REDUCED && DOWNSAMPLE == 8)
                decode_flag = cv::IMREAD_REDUCED_GRAYSCALE_8;
            cv::Mat gray = cv::imdecode(buf, decode_flag);
            if (gray.empty() && decode_flag != cv::IMREAD_GRAYSCALE)
            {
                static std::atomic<bool> fallback_warned{false};
                if (!fallback_warned.exchange(true))
                    RCUTILS_LOG_WARN("reduced decode failed (format: %s), fall back to full decode + resize",
                                     comp_msg->format.c_str());
                gray = cv::imdecode(buf, cv::IMREAD_GRAYSCALE);
            }
            if (gray.empty())
            {
                RCUTILS_LOG_ERROR("compressed image decode failed (format: %s)", comp_msg->format.c_str());
                continue;
            }
            frame.img = gray;
            frame.header = comp_msg->header;
        }
        frame.decode_ms = t_dec.toc();

        // 降采样跟踪:resize 到 (COL, ROW) 再进入光流/特征检测
        // (解码端已按 DOWNSAMPLE 降尺度输出时尺寸一致,跳过 resize)
        if (DOWNSAMPLE > 1 && (frame.img.cols != COL || frame.img.rows != ROW))
        {
            TicToc t_resize;
            cv::Mat scaled;
            cv::resize(frame.img, scaled, cv::Size(COL, ROW), 0, 0, cv::INTER_AREA);
            frame.img = scaled;
            frame.resize_ms = t_resize.toc();
        }

        {
            std::lock_guard<std::mutex> lg(m_buf);
            if (decoded_buf.size() >= 1)
            {
                decoded_buf.pop();
                g_dropped++;
            }
            decoded_buf.push(std::move(frame));
        }
        con.notify_all();
    }
}

// 跟踪线程:消费解码后的帧,process_image 全部逻辑在本线程串行执行
void track_loop()
{
    while (running.load())
    {
        DecodedFrame frame;
        {
            std::unique_lock<std::mutex> lk(m_buf);
            con.wait(lk, []
                     { return !decoded_buf.empty() || !running.load(); });
            if (decoded_buf.empty())
                break; // running = false 且无待处理帧
            frame = std::move(decoded_buf.front());
            decoded_buf.pop();
        }
        process_image(frame);
    }
}

void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    if (!acceptImageFormat(FORMAT_RAW))
        return;
    push_latest(raw_buf, img_msg);
}

// 兼容 sensor_msgs/msg/CompressedImage。
// 压缩流(jpeg/png)在解码线程 imdecode 直接解码为灰度图,
// 与 raw 路径的 cv_bridge mono8 转换完全分离
void compressed_img_callback(const sensor_msgs::msg::CompressedImage::SharedPtr img_msg)
{
    if (!acceptImageFormat(FORMAT_COMPRESSED))
        return;
    push_latest(comp_buf, img_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("feature_tracker");

    readParameters(n);

    // OpenCV 内部并行线程数:单线程化。跟踪分辨率(降采样后)下并行收益为负,且默认
    // 8 线程池在满载机器上 worker 被其他进程抢占,主线程 join 停滞会造成 track 段
    // 耗时尖峰(实测偶发 80~100ms,典型 ~6ms);单线程消除常驻 pool 并稳定耗时
    cv::setNumThreads(CV_NUM_THREADS);

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    if (FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if (!trackerData[i].fisheye_mask.data)
            {
                RCUTILS_LOG_INFO("load mask fail");
            }
            else
                RCUTILS_LOG_INFO("load mask success");
        }
    }

    // 按 image_type 只订阅一种图像话题(见 parameters 中 IMAGE_TYPE):
    //   - "compressed":订阅 base + "/compressed" 上的 CompressedImage
    //   - "raw":订阅 base 话题上的 sensor_msgs/msg/Image
    // image_topic 若已带 "/compressed" 后缀则先剥离得到 base 话题;
    // 订阅 QoS 由 yaml 的 image_sub_qos 配置
    std::string base_topic = IMAGE_TOPIC;
    const std::string compressed_suffix = "/compressed";
    if (base_topic.size() > compressed_suffix.size() &&
        base_topic.compare(base_topic.size() - compressed_suffix.size(), compressed_suffix.size(), compressed_suffix) == 0)
    {
        base_topic = base_topic.substr(0, base_topic.size() - compressed_suffix.size());
    }
    rclcpp::SubscriptionBase::SharedPtr sub_img;
    if (IMAGE_TYPE == "compressed")
    {
        RCUTILS_LOG_INFO("subscribe compressed image: %s", (base_topic + compressed_suffix).c_str());
        sub_img = n->create_subscription<sensor_msgs::msg::CompressedImage>(base_topic + compressed_suffix, IMAGE_SUB_QOS, compressed_img_callback);
    }
    else
    {
        RCUTILS_LOG_INFO("subscribe raw image: %s", base_topic.c_str());
        sub_img = n->create_subscription<sensor_msgs::msg::Image>(base_topic, IMAGE_SUB_QOS, img_callback);
    }

    pub_img = n->create_publisher<sensor_msgs::msg::PointCloud>("feature", 1000);
    pub_match = n->create_publisher<sensor_msgs::msg::Image>("feature_img", 1000);
    pub_restart = n->create_publisher<std_msgs::msg::Bool>("restart", 1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */

    // 流水线线程:解码线程与跟踪线程(发布等全部逻辑移出 executor 回调)
    std::thread decode_thread{decode_loop};
    std::thread track_thread{track_loop};
    rclcpp::spin(n);

    // 退出:唤醒并回收流水线线程(joinable 线程不 join 直接析构会 std::terminate)
    running.store(false);
    con.notify_all();
    decode_thread.join();
    track_thread.join();
    return 0;
}

// new points velocity is 0, pub or not?
// track cnt > 1 pub?
