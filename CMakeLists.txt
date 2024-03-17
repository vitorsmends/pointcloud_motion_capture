cmake_minimum_required(VERSION 3.8)
project(pointcloud_motion_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Open3D REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(point_cloud_reader src/point_cloud_reader.cpp)
add_executable(motion_capture src/motion_capture.cpp)

ament_target_dependencies(point_cloud_reader 
  rclcpp 
  sensor_msgs 
  Open3D 
  pcl_conversions
)

ament_target_dependencies(motion_capture 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  Open3D 
  pcl_conversions
)

target_link_libraries(point_cloud_reader 
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(motion_capture 
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS
  point_cloud_reader
  motion_capture
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
