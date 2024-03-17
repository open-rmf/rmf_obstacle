cmake_minimum_required(VERSION 3.8)
project(rmf_obstacle_detector_laserscan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)

set(dep_pkgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rmf_obstacle_msgs
  sensor_msgs
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

#===============================================================================
add_library(${PROJECT_NAME} SHARED src/LaserscanDetector.cpp)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${rclcpp_components_LIBRARIES}
    ${rmf_obstacle_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_lifecycle_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${rmf_obstacle_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "LaserscanDetector"
  EXECUTABLE laserscan_detector)

#===============================================================================if(BUILD_TESTING)
install(
  TARGETS
    ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
