cmake_minimum_required(VERSION 3.8)
project(rosflight_timing_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosflight_msgs REQUIRED)

add_executable(timing_driver src/timing_driver.cpp)
target_compile_features(timing_driver PUBLIC cxx_std_17)
ament_target_dependencies(timing_driver rclcpp rosflight_msgs)

install(TARGETS timing_driver DESTINATION lib/${PROJECT_NAME})

ament_package()
