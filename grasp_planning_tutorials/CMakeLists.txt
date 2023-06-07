cmake_minimum_required(VERSION 2.8.3)
project(grasp_planning_tutorials)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

file(GLOB SCRIPT_PROGRAMS node_scripts/*.py)
catkin_install_python(PROGRAMS ${SCRIPT_PROGRAMS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts
)

install(DIRECTORY graspit_data launch models scripts worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
