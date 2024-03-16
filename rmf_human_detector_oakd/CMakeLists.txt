cmake_minimum_required(VERSION 3.8)
# add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core EXCLUDE_FROM_ALL)

project(rmf_human_detector_oakd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rmf_obstacle_msgs REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

#===============================================================================
add_library(rmf_human_detector_oakd SHARED src/HumanDetector.cpp)

target_include_directories(rmf_human_detector_oakd
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${rmf_obstacle_msgs_INCLUDE_DIRS}
    ${depthai_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(rmf_human_detector_oakd
  PUBLIC
    rclcpp::rclcpp
    ${rclcpp_components_LIBRARIES}
    depthai::opencv
    ${rmf_obstacle_msgs_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_compile_features(rmf_human_detector_oakd INTERFACE cxx_std_17)

rclcpp_components_register_node(rmf_human_detector_oakd
  PLUGIN "rmf_human_detector_oakd::HumanDetector"
  EXECUTABLE human_detector)

install(
  TARGETS rmf_human_detector_oakd
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
