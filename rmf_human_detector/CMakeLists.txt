cmake_minimum_required(VERSION 3.8)
project(rmf_human_detector)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_building_map_msgs REQUIRED)
find_package(rmf_obstacle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

#=============================================================

add_executable(human_detector src/HumanDetector.cpp src/YoloDetector.cpp)

target_include_directories(human_detector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(human_detector
  cv_bridge
  image_geometry
  OpenCV
  rclcpp
  rmf_building_map_msgs
  rmf_obstacle_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
)

install(TARGETS human_detector
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  assets
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
