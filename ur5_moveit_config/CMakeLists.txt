cmake_minimum_required(VERSION 3.8)
project(ur5_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/pick_and_place.py
    scripts/detect_objects.py
    scripts/pixel_to_world.py
    scripts/test_ik.py
    scripts/test_scan_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
