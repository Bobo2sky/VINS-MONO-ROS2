#pragma once
#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui/highgui.hpp>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;


extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string IMAGE_TYPE;
extern rclcpp::QoS IMAGE_SUB_QOS;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int DOWNSAMPLE;
// 压缩图解码阶段直接按 DOWNSAMPLE 缩放输出(IMREAD_REDUCED_*),0 = 关闭走全尺寸解码+resize
extern int DECODE_REDUCED;
// OpenCV 内部并行线程数(cv::setNumThreads),默认 1:
// 跟踪分辨率下并行负收益,且满载机器上线程池 worker 被抢占会导致 join 停滞(track 段耗时尖峰)
extern int CV_NUM_THREADS;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

void readParameters(rclcpp::Node::SharedPtr &n);
