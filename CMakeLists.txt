cmake_minimum_required(VERSION 3.8)
project(motion_capture_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Open3D REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_LIBRARY_DIRS}
  )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(point_cloud_reader src/point_cloud_reader.cpp)

ament_target_dependencies(point_cloud_reader ament_cmake rclcpp sensor_msgs Open3D pcl_conversions)

target_link_libraries(point_cloud_reader Open3D::Open3D ${PCL_LIBRARIES})

install(TARGETS
  point_cloud_reader
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
