cmake_minimum_required(VERSION 3.5)
project(camera_model)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV 4 REQUIRED)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

ament_export_include_directories(include)
ament_export_libraries(camera_model)
ament_export_dependencies(std_msgs)

include_directories("include")

add_executable(Calibration 
    src/intrinsic_calib.cc
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

add_library(camera_model
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

# 只链接实际用到的 OpenCV 模块；${OpenCV_LIBS} 会带入 Debian OpenCVConfig
# 的全部模块（含 viz->VTK->GDAL 链路），J6 板端运行库闭包无法满足。
set(VINS_OPENCV_LIBS
  opencv_core opencv_imgproc opencv_calib3d opencv_features2d
  opencv_video opencv_imgcodecs opencv_highgui)
target_link_libraries(Calibration ${Boost_LIBRARIES} ${VINS_OPENCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(camera_model ${Boost_LIBRARIES} ${VINS_OPENCV_LIBS} ${CERES_LIBRARIES})

# Install
install(TARGETS
    Calibration
    DESTINATION lib/${PROJECT_NAME}
    )


install(TARGETS Calibration camera_model
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS Calibration
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
  
install(DIRECTORY include/
    DESTINATION include/
)

ament_package()