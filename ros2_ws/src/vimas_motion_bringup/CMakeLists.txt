cmake_minimum_required(VERSION 3.8)
project(vimas_motion_bringup)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/head_vio_visualizer.py
    scripts/motion_replay_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
