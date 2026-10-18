# CLAUDE.md — VINS-Mono ROS2

单目相机 + IMU 的视觉惯性里程计(VIO),含回环检测。

## 构建与运行环境

- 依赖:OpenCV 4、Ceres Solver、Eigen3、cv_bridge、DBoW2(源码内置)
- 本仓库开发环境为 Ubuntu + ROS2
- 构建:

```bash
cd /workspace/vins_mono
source /opt/ros/humble/setup.bash
colcon build
```

- 修改任何 launch 或 `config_pkg/config/` 下的 yaml 后需要重新 `colcon build`(配置经 `install/` 目录共享)

## 包结构与职责

| 包 | 职责 |
| --- | --- |
| `feature_tracker` | 订阅原始图像,光流跟踪特征点(cv::calcOpticalFlowPyrLK + RANSAC F 矩阵剔除外点),去畸变后以归一化坐标发布 PointCloud |
| `vins_estimator` | 核心 VIO:IMU 预积分、滑窗优化(Ceres)、初始化(视觉-惯性对齐)、边缘化、发布里程计 |
| `pose_graph` | 回环检测(DBoW2 + BRIEF 描述子)、重定位、全局位姿图 |
| `camera_model` | camodocal 相机模型库(PINHOLE / MEI / KANNALA_BRANDT / SCARAMUZZA),无节点 |
| `ar_demo` | AR 演示节点 |
| `benchmark_publisher` | 发布 EuRoC 真值轨迹 |
| `config_pkg` | 所有配置 yaml、rviz 配置、回环字典(`support_files/brief_k10L6.bin`)。**ROS2 移植说明**:`get_package_share_directory` 只解析 `install/` 下的包,故将配置从 ROS1 的 vins_estimator 中拆出为独立包 |

## 数据流(核心链路)

```text
图像话题 (按 yaml 的 image_type 订阅 raw Image 或 CompressedImage)
  └─ feature_tracker/src/feature_tracker_node.cpp
       img_callback / compressed_img_callback(由 image_type 决定,见下节)
       → cv_bridge 转 mono8 → FeatureTracker::readImage (feature_tracker.cpp)
         (CLAHE 可选均衡 / LK 光流 / rejectWithF / setMask / goodFeaturesToTrack)
       → undistortedPoints() 经 camodocal 相机模型去畸变(归一化坐标)
       → 发布 /feature_tracker/feature (PointCloud: x,y,z=1 + id/u/v/velocity 通道)

IMU 话题 (/imu0)
  └─ vins_estimator/src/estimator_node.cpp: imu_callback
       → imu_buf + predict() 高频 IMU 前向传播(发布 /vins_estimator/imu_propagate)

/feature_tracker/feature + /feature_tracker/restart
  └─ estimator_node.cpp: feature_callback / restart_callback
       → process 线程 getMeasurements() 按 estimator.td 做 IMU-图像时间对齐
       → Estimator::processImage (estimator.cpp)
         → FeatureManager::addFeatureCheckParallax(关键帧判定)
         → 滑窗 solveOdometry / solveInit / solveVI (Ceres: IMU 预积分因子 + 重投影因子)
         → slideWindow + 边缘化 (marginalization_factor.cpp)
       → visualization.cpp 发布 odometry / keyframe_pose / point_cloud / tf 等

/vins_estimator/keyframe_pose + keyframe_point + 原始图像
  └─ pose_graph/src/pose_graph_node.cpp
       → 建关键帧 KeyFrame (keyframe.cpp, BRIEF 描述子)
       → DBoW2 检索回环 → optimize → 发布 /pose_graph/pose_graph、match_points
       → estimator 端 relocalization_callback 接收重定位约束
```

关键常量:`NUM_OF_CAM = 1`(硬编码单目,feature_tracker 与 vins_estimator 的 parameters.h)、`WINDOW_SIZE = 20`、`FOCAL_LENGTH = 460`(归一化平面视差换算)。

## 话题一览(输入/输出)

节点发布/订阅的相对话题经 launch 中的 namespace 解析为全名(`/feature_tracker`、`/vins_estimator`、`/pose_graph`)。`<imu_topic>` / `<image_topic>` 为 yaml 中配置的输入话题名。

### feature_tracker

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `<image_topic>`(`image_type: raw` 时) | `sensor_msgs/msg/Image` | 输入 | 原始灰度图,cv_bridge 转 mono8;QoS 由 `image_sub_qos` 配置 |
| `<image_topic>/compressed`(`image_type: compressed` 时) | `sensor_msgs/msg/CompressedImage` | 输入 | jpeg/png 压缩图,`cv::imdecode` 独立解码;QoS 由 `image_sub_qos` 配置 |
| `/feature_tracker/feature` | `sensor_msgs/msg/PointCloud` | 输出 | 特征点:points(x, y, 1) 为去畸变归一化坐标;channels 依次为 `id`(全局特征 ID)、`u`/`v`(像素坐标)、`velocity_x`/`velocity_y`(归一化平面速度) |
| `/feature_tracker/feature_img` | `sensor_msgs/msg/Image` | 输出 | 特征跟踪可视化(RGB,按跟踪长度渐变色);仅 `show_track: 1` 时发布 |
| `/feature_tracker/restart` | `std_msgs/msg/Bool` | 输出 | 图像时间戳跳变/回退时置 true,通知 estimator 重置 |

### vins_estimator

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `<imu_topic>` | `sensor_msgs/msg/Imu` | 输入 | IMU 原始数据;QoS 由 `imu_sub_qos` 配置 |
| `/feature_tracker/feature` | `sensor_msgs/msg/PointCloud` | 输入 | 特征点(见上表) |
| `/feature_tracker/restart` | `std_msgs/msg/Bool` | 输入 | 重置信号 |
| `/pose_graph/match_points` | `sensor_msgs/msg/PointCloud` | 输入 | 快速重定位约束(仅 `fast_relocalization: 1` 时有数据) |
| `/vins_estimator/imu_propagate` | `nav_msgs/msg/Odometry` | 输出 | IMU 前向传播高频里程计:pose 为 world 系姿态/位置,twist 为体系速度 + 陀螺原始角速度 |
| `/vins_estimator/total_mileage` | `geometry_msgs/msg/PointStamped` | 输出 | VIO 总里程(米):`point.x` 存累计路径长度,stamp 为图像帧时间;首帧不计原点偏移,帧间跳变 >10m 不累加(重置保护) |
| `/vins_estimator/odometry` | `nav_msgs/msg/Odometry` | 输出 | 滑窗优化后的 VIO 里程计(world 系,每图像帧一次) |
| `/vins_estimator/path` | `nav_msgs/msg/Path` | 输出 | 滑窗位姿轨迹(world 系) |
| `/vins_estimator/relocalization_path` | `nav_msgs/msg/Path` | 输出 | 漂移校正后的重定位轨迹 |
| `/vins_estimator/point_cloud` | `sensor_msgs/msg/PointCloud` | 输出 | 当前滑窗 3D 路标点(world 系) |
| `/vins_estimator/history_cloud` | `sensor_msgs/msg/PointCloud` | 输出 | 边缘化保留的历史路标点(world 系) |
| `/vins_estimator/key_poses` | `visualization_msgs/msg/Marker` | 输出 | 滑窗关键位姿点集 |
| `/vins_estimator/camera_pose` | `nav_msgs/msg/Odometry` | 输出 | 最新相机位姿(world 系) |
| `/vins_estimator/camera_pose_visual` | `visualization_msgs/msg/MarkerArray` | 输出 | 相机位姿箭头可视化 |
| `/vins_estimator/keyframe_pose` | `nav_msgs/msg/Odometry` | 输出 | 关键帧 IMU 位姿(world 系),供 pose_graph 建关键帧 |
| `/vins_estimator/keyframe_point` | `sensor_msgs/msg/PointCloud` | 输出 | 关键帧对应 3D 路标(world 系) |
| `/vins_estimator/extrinsic` | `nav_msgs/msg/Odometry` | 输出 | 在线估计的相机-IMU 外参(pose 字段存 RIC/tic) |
| `/vins_estimator/relo_relative_pose` | `nav_msgs/msg/Odometry` | 输出 | 重定位时滑窗首帧相对回环帧的位姿 |

TF 广播:`world → body` 与 `world → camera`(均在 pubOdometry 同路径刷新,每图像帧一次,stamp 为图像帧时间)。

### pose_graph

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `<image_topic>`(按 `image_type` 单一格式) | `sensor_msgs/msg/Image` 或 `CompressedImage` | 输入 | 关键帧原始图像,订阅机制同 feature_tracker |
| `/vins_estimator/imu_propagate` | `nav_msgs/msg/Odometry` | 输入 | fast relocalization 时更新旧关键帧初始位姿 |
| `/vins_estimator/odometry` | `nav_msgs/msg/Odometry` | 输入 | VIO 里程计(持续更新关键帧 VIO 位姿 / 绘制 no_loop_path) |
| `/vins_estimator/keyframe_pose` | `nav_msgs/msg/Odometry` | 输入 | 触发建立关键帧 |
| `/vins_estimator/keyframe_point` | `sensor_msgs/msg/PointCloud` | 输入 | 关键帧路标 |
| `/vins_estimator/extrinsic` | `nav_msgs/msg/Odometry` | 输入 | 相机-IMU 外参(重定位坐标转换用) |
| `/vins_estimator/relo_relative_pose` | `nav_msgs/msg/Odometry` | 输入 | 快速重定位相对位姿 |
| `/pose_graph/match_image` | `sensor_msgs/msg/Image` | 输出 | **仅成功闭环时**发布的匹配可视化(需 `save_image: 1`);无回环则该话题无消息 |
| `/pose_graph/match_points` | `sensor_msgs/msg/PointCloud` | 输出 | 重定位约束:points 为(旧关键帧归一化 x, y, 特征 ID);channel `t_q_index` 8 元素 = 旧帧 T(3) + Q(4) + 当前关键帧 index(1) |
| `/pose_graph/pose_graph_path` | `nav_msgs/msg/Path` | 输出 | 闭环优化后全局轨迹 |
| `/pose_graph/base_path` | `nav_msgs/msg/Path` | 输出 | 基准轨迹(第一序列) |
| `/pose_graph/path_<i>` | `nav_msgs/msg/Path` | 输出 | 第 i 个序列的轨迹(`load_previous_pose_graph` 多序列) |
| `/pose_graph/pose_graph` | `visualization_msgs/msg/MarkerArray` | 输出 | 位姿图可视化(顶点 + 回环边) |
| `/pose_graph/no_loop_path` | `nav_msgs/msg/Path` | 输出 | 未闭环的 VIO 原始轨迹(漂移对比) |
| `/pose_graph/camera_pose_visual` | `visualization_msgs/msg/MarkerArray` | 输出 | 关键帧相机位姿可视化 |
| `/pose_graph/key_odometrys` | `visualization_msgs/msg/Marker` | 输出 | 关键帧里程计点集 |

### 辅助节点

- `benchmark_publisher`:输入 `/vins_estimator/odometry`(时间对齐用);输出 `benchmark_publisher/odometry` / `path`(EuRoC 真值)
- `ar_demo`:输入 `/vins_estimator/camera_pose` + `/vins_estimator/point_cloud`(及图像话题);输出 `ar_demo/AR_object`(`visualization_msgs/msg/MarkerArray`)

## 图像/IMU 输入接口(image_type 选择格式 + QoS 配置)

`feature_tracker` 与 `pose_graph` 按 yaml 的 `image_type` **只订阅一种**图像话题:

- `image_type: "raw"`(缺省):订阅 base 话题上的 `sensor_msgs/msg/Image`
- `image_type: "compressed"`:订阅 base 话题 + `/compressed`(image_transport 命名约定)上的 `sensor_msgs/msg/CompressedImage`
- `image_topic` 若已带 `/compressed` 后缀会先剥离得到 base 话题;`image_type` 非法值 WARN 后按 raw 处理

处理行为(见 `feature_tracker_node.cpp` / `pose_graph_node.cpp`):

- raw 路径:cv_bridge 转 mono8(含 `8UC1` 特判);压缩路径:`cv::imdecode` 直接解码为灰度图(jpeg/png 均可,解码失败丢帧并记 ERROR)——两条路径完全独立
- 图像与 IMU 订阅 QoS 由 yaml 的 `image_sub_qos` / `imu_sub_qos` 配置(`reliability: reliable|best_effort` + `depth`),缺省为 reliable:图像 depth 100(feature_tracker)/ 2000(pose_graph),IMU depth 2000

## 图像频率说明(10Hz 相机)

当前传感器为 10Hz 压缩图像(原 EuRoC 为 20Hz),已评估**无需改代码**:

- `freq: 10`(特征发布上限)与 10Hz 输入正好匹配,每帧都会发布;设 `freq: 0` 可严格跟随输入频率
- `WINDOW_SIZE = 20` 在 10Hz 下滑窗覆盖 2s(原 20Hz 为 1s),初始化与优化正常
- 数据集 IMU ≈ 100Hz,每图像帧约 10 个 IMU 样本,满足 VINS 最低要求(EuRoC 为 200Hz 更佳),初始化收敛略慢
- `max_solver_time: 0.04` 在 10Hz(每帧实际预算 100ms)下余量充足
- `keyframe_parallax: 10`:10Hz 下帧间运动约为 20Hz 的 2 倍,关键帧密度自动适配
- 精度要求高时:按实际 IMU 手册/Allan 方差更新 `acc_n/gyr_n/acc_w/gyr_w`(当前为 EuRoC ADIS16448 的值)

## 配置说明(config_pkg/config/euroc/euroc_config.yaml)

- `imu_topic` / `image_topic`:输入话题;`output_path`:输出目录
- `image_type`:图像输入格式 `"raw"` / `"compressed"`(决定唯一订阅的图像话题,见「图像/IMU 输入接口」节)
- `image_sub_qos` / `imu_sub_qos`:订阅 QoS(`reliability: reliable|best_effort`、`depth`),分别作用于图像与 IMU 订阅
- 相机模型:`model_type: PINHOLE`(可选 MEI / KANNALA_BRANDT / SCARAMUZZA,由 `camera_model` 包 `CameraFactory` 按 yaml 分发)
  - PINHOLE 畸变参数支持 **k1, k2, p1, p2, k3, k4, k5, k6**(OpenCV rational 模型:k1/k2/k3 为分子径向项,k4/k5/k6 为分母项);k3~k6 缺省为 0,兼容旧 5 参数配置
  - 内参:`projection_parameters: fx/fy/cx/cy`;`image_width/height` 决定 feature tracker 的 ROW/COL
  - 注意:强畸变 rational 标定的可逆域有限(极端边角超出时去畸变结果被夹取到模型边界,输出有限且连续)
- 外参(cam→IMU):`extrinsicRotation` / `extrinsicTranslation`;`estimate_extrinsic`:
  - `0` 信任给定外参不变;`1` 在初值附近在线优化;`2` 完全在线标定(不给 R,T,初始阶段做旋转运动)
- 特征跟踪:`max_cnt`(特征数)、`min_dist`、`freq`(发布频率)、`F_threshold`、`equalize`、`fisheye`(加载 `config_pkg/config/fisheye_mask.jpg` 圆形 mask)
- IMU 噪声:`acc_n / gyr_n / acc_w / gyr_w / g_norm`(越准越好,可用 IMU 手册或 Allan 方差)
- 回环:`loop_closure`、`load_previous_pose_graph`、`fast_relocalization`、`pose_graph_save_path`、`support_path`
- 时延:`estimate_td` / `td`(相机-IMU 时间偏移);卷帘快门:`rolling_shutter` / `rolling_shutter_tr`

## 运行(EuRoC 数据集)

```bash
# 终端 1:特征跟踪 + rviz
ros2 launch feature_tracker vins_feature_tracker.launch.py
# 终端 2:后端 + 回环
ros2 launch vins_estimator euroc.launch.py
# 终端 3:播放 ROS2 格式的 bag(ROS1 bag 需用 python rosbags 转换)
ros2 bag play <path_to_MH_01_easy>
# 可选:真值
ros2 launch benchmark_publisher benchmark_publisher.launch.py
```

- launch 文件中的 `config_path` 指向 `config_pkg/config/<xx>/<xx>_config.yaml`,换自己的标定即新建 yaml 并改此路径(随后重新 colcon build)
- 回环运行中按 `s` 保存位姿图,`n` 标记新序列
- 实时传感器:把 yaml 的 `image_topic`/`imu_topic` 改为驱动发布的话题即可(图像 raw 或 compressed 均可)
