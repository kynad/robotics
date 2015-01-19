cmake_minimum_required(VERSION 2.8.3)
project(patrol_bot)

# Find catkin
find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package()

# Install move_base_config files
install(DIRECTORY move_base_config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install stage_config files
install(DIRECTORY stage_config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install the rviz files
install(FILES single_robot.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
