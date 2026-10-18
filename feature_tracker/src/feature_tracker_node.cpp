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

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

void process_image(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    if (first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * (1e-9);
        last_image_time = img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * (1e-9);
        return;
    }

    // detect unstable camera stream
    if (img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * (1e-9) - last_image_time > 1.0 || img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * (1e-9) < last_image_time)
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

    last_image_time = img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * (1e-9);
    // frequency control
    if (round(1.0 * pub_count / (img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * (1e-9) - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * (1e-9) - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * (1e-9);
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

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

    cv::Mat show_img = ptr->image;
    // cv::imshow("img", show_img);
    // cv::waitKey(0);
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        RCUTILS_LOG_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * (1e-9));
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
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

    if (PUB_THIS_FRAME)
    {
        pub_count++;
        sensor_msgs::msg::PointCloud::SharedPtr feature_points(new sensor_msgs::msg::PointCloud);
        sensor_msgs::msg::ChannelFloat32 id_of_point;
        sensor_msgs::msg::ChannelFloat32 u_of_point;
        sensor_msgs::msg::ChannelFloat32 v_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

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
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            // cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    // draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    // char name[10];
                    // sprintf(name, "%d", trackerData[i].ids[j]);
                    // cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            // cv::imshow("vis", stereo_img);
            // cv::waitKey(5);
            pub_match->publish(*(ptr->toImageMsg()));
        }
    }
    // RCUTILS_LOG_INFO("whole feature tracker processing costs: %fms", t_r.toc());
}

void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    if (!acceptImageFormat(FORMAT_RAW))
        return;
    process_image(img_msg);
}

// 兼容 sensor_msgs/msg/CompressedImage。
// 压缩流(jpeg/png)独立解码:imdecode 直接解码为灰度图,
// 与 raw 路径的 cv_bridge mono8 转换完全分离
void compressed_img_callback(const sensor_msgs::msg::CompressedImage::SharedPtr img_msg)
{
    if (!acceptImageFormat(FORMAT_COMPRESSED))
        return;
    cv::Mat buf(1, img_msg->data.size(), CV_8UC1, const_cast<uint8_t *>(img_msg->data.data()));
    cv::Mat gray = cv::imdecode(buf, cv::IMREAD_GRAYSCALE);
    if (gray.empty())
    {
        RCUTILS_LOG_ERROR("compressed image decode failed (format: %s)", img_msg->format.c_str());
        return;
    }
    // 包装为 mono8 Image 消息,复用后续特征跟踪处理主体
    cv_bridge::CvImage cv_img(img_msg->header, sensor_msgs::image_encodings::MONO8, gray);
    process_image(cv_img.toImageMsg());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("feature_tracker");

    readParameters(n);

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