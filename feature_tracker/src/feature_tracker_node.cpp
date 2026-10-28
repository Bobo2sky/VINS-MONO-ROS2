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

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::msg::Image::ConstPtr> img_buf;

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

// 上一帧的解码/格式转换耗时(ms):compressed 路径 imdecode、raw 路径 cv_bridge,
// 原计时 t_r 不含此段,补全后由 process_image 的 30 帧窗口汇总输出
static double g_decode_ms = 0.0;

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

// 直接接收 cv::Mat(raw 路径经 cv_bridge 转换、compressed 路径 imdecode 后调用),
// 避免压缩路径 8MP 全图的双重拷贝(toImageMsg + toCvCopy)
void process_image(const cv::Mat &raw_img, const std_msgs::msg::Header &header)
{
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

    // 降采样跟踪:解码/转换后 resize 到 (COL, ROW) 再进入光流/特征检测
    // (解码端已按 DOWNSAMPLE 减半输出时尺寸一致,跳过 resize)
    cv::Mat track_img = raw_img;
    double resize_ms = 0.0;
    if (DOWNSAMPLE > 1 && (raw_img.cols != COL || raw_img.rows != ROW))
    {
        TicToc t_resize;
        cv::Mat scaled;
        cv::resize(track_img, scaled, cv::Size(COL, ROW), 0, 0, cv::INTER_AREA);
        track_img = scaled;
        resize_ms = t_resize.toc();
    }

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

    // 30 帧窗口分段耗时汇总(INFO 级可见;decode 为计时盲区的补全)
    {
        static double sum_decode = 0, sum_resize = 0, sum_lk = 0, sum_reject = 0,
                      sum_mask = 0, sum_gftt = 0, sum_undist = 0, sum_pub = 0, sum_total = 0;
        static int frame_cnt = 0;
        sum_decode += g_decode_ms;
        sum_resize += resize_ms;
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
            RCUTILS_LOG_INFO("ft timing(ms, 30f avg): decode %.1f resize %.1f lk %.1f rejectF %.1f mask %.1f gftt %.1f undist %.1f pub %.1f | total %.1f",
                             sum_decode / frame_cnt, sum_resize / frame_cnt, sum_lk / frame_cnt,
                             sum_reject / frame_cnt, sum_mask / frame_cnt, sum_gftt / frame_cnt,
                             sum_undist / frame_cnt, sum_pub / frame_cnt, sum_total / frame_cnt);
            if (sum_total / frame_cnt > 100.0)
                RCUTILS_LOG_WARN("feature tracker avg total %.1fms exceeds 100ms frame budget",
                                 sum_total / frame_cnt);
            sum_decode = sum_resize = sum_lk = sum_reject = sum_mask = 0;
            sum_gftt = sum_undist = sum_pub = sum_total = 0;
            frame_cnt = 0;
        }
    }
    // 单帧超预算立即告警(10Hz 下预算 100ms)
    if (total_ms + g_decode_ms > 100.0)
        RCUTILS_LOG_WARN("feature tracker frame costs %.1fms (decode %.1fms + track %.1fms), exceeds 100ms budget",
                         total_ms + g_decode_ms, g_decode_ms, total_ms);
}

void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    if (!acceptImageFormat(FORMAT_RAW))
        return;
    TicToc t_dec;
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    g_decode_ms = t_dec.toc();
    process_image(ptr->image, img_msg->header);
}

// 兼容 sensor_msgs/msg/CompressedImage。
// 压缩流(jpeg/png)独立解码:imdecode 直接解码为灰度图,
// 与 raw 路径的 cv_bridge mono8 转换完全分离
void compressed_img_callback(const sensor_msgs::msg::CompressedImage::SharedPtr img_msg)
{
    if (!acceptImageFormat(FORMAT_COMPRESSED))
        return;
    TicToc t_dec;
    cv::Mat buf(1, img_msg->data.size(), CV_8UC1, const_cast<uint8_t *>(img_msg->data.data()));
    // 解码阶段直接 1/DOWNSAMPLE 降尺度(libjpeg DCT scaling),免去全尺寸解码 + resize;
    // 仅压缩路径且倍数为 2/4/8 时生效,失败(如格式不支持缩放解码)回退全尺寸
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
                             img_msg->format.c_str());
        gray = cv::imdecode(buf, cv::IMREAD_GRAYSCALE);
    }
    g_decode_ms = t_dec.toc();
    if (gray.empty())
    {
        RCUTILS_LOG_ERROR("compressed image decode failed (format: %s)", img_msg->format.c_str());
        return;
    }
    process_image(gray, img_msg->header);
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
    rclcpp::spin(n);
    return 0;
}

// new points velocity is 0, pub or not?
// track cnt > 1 pub?