cmake_minimum_required(VERSION 3.10)

# Project properties
SET(PROJECT_NAMESPACE ComePerrot)
SET(PROJECT_NAME deburring_mpc)
SET(PROJECT_DESCRIPTION "Manipulation for Talos")
SET(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/talos-manipulation)

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)

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
include("${JRL_CMAKE_MODULES}/boost.cmake")

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

add_compile_options(-Wno-error)

# Project dependencies
if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(eigenpy 2.7.10 REQUIRED)
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
  # workaround eigenpy v3.1.0 cmake issue
  include("${JRL_CMAKE_MODULES}/python.cmake")
endif()
add_project_dependency(crocoddyl REQUIRED)
add_project_dependency(yaml-cpp REQUIRED)
if(BUILD_WITH_MULTITHREADS)
  add_project_dependency(OpenMP REQUIRED)
endif()

include("${JRL_CMAKE_MODULES}/python.cmake")

SET(${PROJECT_NAME}_HEADERS
    include/${PROJECT_NAME}/robot_designer.hpp
    include/${PROJECT_NAME}/fwd.hpp
    include/${PROJECT_NAME}/python_fwd.hpp
    include/${PROJECT_NAME}/ocp.hpp
    include/${PROJECT_NAME}/mpc.hpp
)

SET(${PROJECT_NAME}_SOURCES
    src/robot_designer.cpp
    src/ocp.cpp
    src/ocp_cost_library.cpp
    src/ocp_debug.cpp
    src/ocp_params.cpp
    src/ocp_problem_helper.cpp
    src/ocp_problem_maker.cpp
    src/mpc.cpp
    src/mpc_debug.cpp
    src/mpc_params.cpp
    src/mpc_state_machine.cpp
    src/mpc_variable_posture.cpp
)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC crocoddyl::crocoddyl
                                             yaml-cpp)
if(BUILD_WITH_MULTITHREADS)
  target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Python Bindings
if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif()

# Unit tests
if(BUILD_TESTING)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework)
  add_subdirectory(tests)
endif()

# Installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
