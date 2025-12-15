cmake_minimum_required(VERSION 3.8)
project(webots_ros2_my_robot)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(geometry_msgs REQUIRED)

install(
    DIRECTORY launch my_robot_driver resource worlds
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
