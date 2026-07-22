cmake_minimum_required(VERSION 3.0.2)
project(uav_kd_data_collector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/record_teacher_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
