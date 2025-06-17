cmake_minimum_required(VERSION 3.0.2)
project(aruco_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/camera_calibration_node.py
  scripts/aruco_detector.py
  scripts/localization_manager.py
  scripts/motion_commander.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
