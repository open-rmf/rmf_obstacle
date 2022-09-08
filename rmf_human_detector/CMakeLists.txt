cmake_minimum_required(VERSION 3.8)
project(rmf_human_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rmf_obstacle_msgs REQUIRED)
find_package(rmf_building_map_msgs REQUIRED)

if(${OpenCV_VERSION} GREATER 4.0)
  add_definitions(-D_CV4)
else()
  add_definitions(-D_CV3)
endif()

if(OpenCV_CUDA_VERSION)
  message (STATUS "OpenCV was compiled with GPU support, use GPU")
  add_definitions(-D_USE_GPU)
else()
  message (STATUS "OpenCV was not compiled with GPU support")
endif()

#=============================================================

add_executable(human_detector src/HumanDetector.cpp src/YoloDetector.cpp)

target_include_directories(human_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(human_detector PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  human_detector
  rclcpp
  sensor_msgs
  image_geometry
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  cv_bridge
  OpenCV
  rmf_obstacle_msgs
  rmf_building_map_msgs
)

install(TARGETS human_detector
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
