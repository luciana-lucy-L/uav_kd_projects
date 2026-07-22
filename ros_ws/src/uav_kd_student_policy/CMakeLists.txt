cmake_minimum_required(VERSION 3.0.2)
project(uav_kd_student_policy)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/run_bc_policy.py
  scripts/run_bc_red_wp_policy.py
  scripts/run_traj_kd_policy.py
  scripts/run_self_red_wp_policy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
