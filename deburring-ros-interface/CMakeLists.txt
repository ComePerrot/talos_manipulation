cmake_minimum_required(VERSION 3.10)

SET(PROJECT_NAMESPACE ComePerrot)
SET(PROJECT_NAME deburring_ros_interface)
SET(PROJECT_DESCRIPTION "Ros interface for Talos manipulation MPC")
SET(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/talos-manipulation)

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/../cmake")
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/../cmake/base.cmake")
  if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    message(
      FATAL_ERROR
        "\nPlease run the following command first:\ngit submodule update --init\n"
    )
  else()
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

# JRL-cmakemodule setup
include("${JRL_CMAKE_MODULES}/base.cmake")

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

add_compile_options(-Wno-error)

# Project dependencies
add_project_dependency(Eigen3 REQUIRED)
add_project_dependency(deburring_mpc REQUIRED)

set(CATKIN_COMPONENTS
  roscpp
  roslib
  realtime_tools
  eigen_conversions
  linear_feedback_controller_msgs
  pal_statistics
  tf2_ros
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_COMPONENTS}
)

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

SET(${PROJECT_NAME}_SOURCES
    src/ros_interface.cpp
    src/onnx_interface.cpp
)
SET(${PROJECT_NAME}_HEADERS
    include/${PROJECT_NAME}/ros_interface.h
    include/${PROJECT_NAME}/onnx_interface.h
)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC deburring_mpc::deburring_mpc Eigen3::Eigen ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

add_executable(ros_deburring src/ros_deburring.cpp)
target_link_libraries(ros_deburring ${PROJECT_NAME} deburring_mpc::deburring_mpc Eigen3::Eigen ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  # find_package(rostest REQUIRED)
  # add_rostest(test/ros-mpc-test.test)
  catkin_add_gtest(test_onnx_interface test/test_onnx_interface.cpp)
  target_link_libraries(test_onnx_interface gtest gtest_main ${PROJECT_NAME})
endif()

# Installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
install(TARGETS ros_deburring RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY test DESTINATION share/${PROJECT_NAME})
  
  

