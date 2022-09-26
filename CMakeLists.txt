cmake_minimum_required(VERSION 3.8)
project(bno055
        VERSION 1.0.0
        DESCRIPTION "TODO(tarjei)"
        LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Add imu node
add_executable(
        bno055_i2c
        src/bno055_i2c_driver.cpp
        src/bno055_i2c_node.cpp
        src/watchdog.cpp
)
ament_target_dependencies(
        bno055_i2c
        rclcpp
        std_msgs
	      sensor_msgs
	      diagnostic_msgs
)
target_include_directories(
        bno055_i2c PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Install executables
install(TARGETS
        bno055_i2c
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        include
        launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()


