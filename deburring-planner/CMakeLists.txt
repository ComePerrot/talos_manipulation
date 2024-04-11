cmake_minimum_required(VERSION 3.10)

SET(PROJECT_NAMESPACE ComePerrot)
SET(PROJECT_NAME deburring_deep_planner)
SET(PROJECT_DESCRIPTION "Deep neural network posture planner for Talos deburing")
SET(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/talos-manipulation)

# Project definition
project(${PROJECT_NAME})

set(CATKIN_COMPONENTS
  # ropsy
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_COMPONENTS}
)

# Installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)

catkin_install_python(PROGRAMS node/nn_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
