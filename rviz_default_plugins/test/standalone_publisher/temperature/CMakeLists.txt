cmake_minimum_required(VERSION 3.5)
project(temperature_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(temperature_publisher temperature_standalone_publisher.cpp)
ament_target_dependencies(temperature_publisher rclcpp sensor_msgs)

install(TARGETS
        temperature_publisher
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()