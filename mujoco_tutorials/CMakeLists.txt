cmake_minimum_required(VERSION 2.8.3)
project(mujoco_tutorials)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
  )

find_package(cmake_modules REQUIRED)

catkin_package()
