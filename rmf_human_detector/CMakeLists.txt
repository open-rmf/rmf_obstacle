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
find_package(rclcpp_components REQUIRED)
find_package(rmf_building_map_msgs REQUIRED)
find_package(rmf_obstacle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

#=============================================================

add_library(human_detector_component SHARED src/HumanDetector.cpp src/YoloDetector.cpp)

message("Image geometry: " ${image_geometry_INCLUDE_DIRS})

target_include_directories(human_detector_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_building_map_msgs_INCLUDE_DIRS}
    ${rmf_obstacle_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
)

target_link_libraries(human_detector_component
  PUBLIC
    ${cv_bridge_LIBRARIES}
    ${image_geometry_LIBRARIES}
    ${OpenCV_LIBRARIES}
    rclcpp::rclcpp
    image_geometry::image_geometry
    cv_bridge::cv_bridge
    tf2_geometry_msgs::tf2_geometry_msgs
    ${rclcpp_components_LIBRARIES}
    ${rmf_building_map_msgs_LIBRARIES}
    ${rmf_obstacle_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${tf2_LIBRARIES}
    ${tf2_msgs_LIBRARIES}
    ${tf2_ros_LIBRARIES}
)

rclcpp_components_register_node(human_detector_component
  PLUGIN "rmf_human_detector::HumanDetector"
  EXECUTABLE human_detector_node
)

install(DIRECTORY
  launch
  assets
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS human_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
