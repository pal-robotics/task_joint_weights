#
#  Copyright (c) 2013, PAL Robotics, S.L. Author: Gennaro Raiola
#  Copyright 2010 CNRS
#

INCLUDE(../cmake/python.cmake)

SET(LIBRARY_NAME ${PROJECT_NAME})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

# provide path to dependency libraries
LINK_DIRECTORIES(${DYNAMIC_GRAPH_PYTHON_LIBRARY_DIRS} ${PYTHON_LIBRARY_DIRS})

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  TaskJointWeights.cpp
  )

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

# Add dynamic-graph-python compilation flags and link to dependency libraries
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} dynamic-graph-python)

INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

SET(NEW_ENTITY_CLASS
  TaskJointWeights
  )

PYTHON_INSTALL_ON_SITE("dynamic_graph/sot/core" "meta_task_joint_weights.py")

DYNAMIC_GRAPH_PYTHON_MODULE("/sot/core/task_joint_weights" ${LIBRARY_NAME} wrap)
