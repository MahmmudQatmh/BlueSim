cmake_minimum_required(VERSION 3.8)

project(bluesim_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

install(
  PROGRAMS
    bluesim_ros2/rover_command.py
    bluesim_ros2/boat_command.py
    bluesim_ros2/drone_command.py
    bluesim_ros2/blueboat_experiment.py 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
