cmake_minimum_required(VERSION 3.5)
project(gnss_odom_publisher)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(gnss_odom_publisher src/gnss_odom_publisher.cpp)
ament_target_dependencies(gnss_odom_publisher rclcpp nav_msgs)


install(TARGETS
  gnss_odom_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()