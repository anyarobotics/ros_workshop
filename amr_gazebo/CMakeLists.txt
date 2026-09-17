cmake_minimum_required(VERSION 3.8)
project(amr_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  worlds
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
