#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
// 图像输入模式:"raw" = sensor_msgs/msg/Image,"compressed" = sensor_msgs/msg/CompressedImage
std::string IMAGE_TYPE;
// 图像订阅 QoS(由 yaml 的 image_sub_qos 配置)
rclcpp::QoS IMAGE_SUB_QOS{rclcpp::KeepLast(100)};
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
// 图像降采样倍数(仅 PINHOLE 模型):解码后 resize 到 1/DOWNSAMPLE 分辨率再跟踪,
// 内参/min_dist/ROW/COL 同比缩放,归一化坐标畸变与分辨率无关
int DOWNSAMPLE = 1;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

template <typename T>
T readParam(rclcpp::Node::SharedPtr n, std::string name)
{
    T ans;
    std::string default_value = "";
    n->declare_parameter<std::string>(name, default_value);
    if (n->get_parameter(name, ans))
    {
        RCLCPP_INFO_STREAM(n->get_logger(), "Loaded " << name << ": " << ans);
    }
    else
    {
        RCLCPP_ERROR_STREAM(n->get_logger(), "Failed to load " << name);
        rclcpp::shutdown();
    }
    return ans;
}

// 从 yaml 的 <name> 节点解析订阅 QoS(reliability / depth),
// 缺省为 reliable + KeepLast(default_depth)
static rclcpp::QoS readSubQos(cv::FileStorage &fsSettings, const std::string &name, int default_depth)
{
    cv::FileNode node = fsSettings[name];
    int depth = default_depth;
    std::string reliability = "reliable";
    if (!node.empty())
    {
        if (!node["depth"].empty())
            depth = (int)node["depth"];
        if (!node["reliability"].empty())
            node["reliability"] >> reliability;
    }
    rclcpp::QoS qos{rclcpp::KeepLast(depth)};
    if (reliability == "best_effort")
        qos.best_effort();
    else
        qos.reliable();
    return qos;
}

void readParameters(rclcpp::Node::SharedPtr &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    RCUTILS_LOG_INFO("config_file: %s", config_file.c_str());
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        RCUTILS_LOG_ERROR("ERROR: Wrong path to settings");
    }
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    // 图像输入模式与订阅 QoS(image_type 非法或缺省时按 raw 处理)
    IMAGE_TYPE = "raw";
    if (!fsSettings["image_type"].empty())
        fsSettings["image_type"] >> IMAGE_TYPE;
    if (IMAGE_TYPE != "raw" && IMAGE_TYPE != "compressed")
    {
        RCUTILS_LOG_WARN("unknown image_type: %s, fall back to raw", IMAGE_TYPE.c_str());
        IMAGE_TYPE = "raw";
    }
    IMAGE_SUB_QOS = readSubQos(fsSettings, "image_sub_qos", 100);
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    // 降采样跟踪:非 PINHOLE 模型不支持时按 1 处理(内参缩放仅实现 PINHOLE)
    DOWNSAMPLE = 1;
    if (!fsSettings["downsample"].empty())
        DOWNSAMPLE = (int)fsSettings["downsample"];
    std::string model_type;
    if (!fsSettings["model_type"].empty())
        fsSettings["model_type"] >> model_type;
    if (DOWNSAMPLE > 1 && model_type != "PINHOLE")
    {
        RCUTILS_LOG_WARN("downsample only supports PINHOLE (model_type: %s), keep full resolution", model_type.c_str());
        DOWNSAMPLE = 1;
    }
    if (DOWNSAMPLE < 1)
    {
        RCUTILS_LOG_WARN("invalid downsample: %d, fall back to 1", DOWNSAMPLE);
        DOWNSAMPLE = 1;
    }
    ROW /= DOWNSAMPLE;
    COL /= DOWNSAMPLE;
    MIN_DIST /= DOWNSAMPLE;
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20; // 调整WINDOW_SIZE的大小会影响VINS-Mono的性能，增大窗口大小可以提高精度，但会增加计算量和延迟。默认WINDOW_SIZE=20，原始图像是20hz，1s的时间滑窗
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();
}
