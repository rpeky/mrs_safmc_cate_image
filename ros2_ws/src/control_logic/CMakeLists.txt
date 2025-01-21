cmake_minimum_required(VERSION 3.8)
project(control_logic)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependency search
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# add aruco stuff
add_library(aruco_utils
	../aruco/aruco_samples_utility.hpp
	../aruco/image_converter.cpp
)

# add offboard_control.cpp
add_executable(offboard_controller src/offboard_controller.cpp)

# link aruco and offboard
target_link_libraries(offboard_controller
	aruco_utils
)

# link the dependencies
ament_target_dependencies(offboard_controller rclcpp px4_msgs geometry_msgs tf2 tf2_ros geometry_msgs)

# install executable
install(TARGETS offboard_controller DESTINATION lib/${PROJECT_NAME})

ament_package()
