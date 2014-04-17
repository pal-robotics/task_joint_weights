#
#  Copyright (c) 2013, PAL Robotics, S.L. Author: Gennaro Raiola
#  Copyright 2010 CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)


project(task_joint_weights)

find_package(catkin REQUIRED COMPONENTS sot_cmake jrl-mal dynamic-graph dynamic-graph-python sot-dyninv sot-core)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
#        CATKIN_DEPENDS jrl-mal dynamic-graph dynamic-graph-python sot-core
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(task_joint_weights
        src/TaskJointWeights.cpp
        include/dynamic-graph/TaskJointWeights/TaskJointWeights.hh
)
target_link_libraries(task_joint_weights ${catkin_LIBRARIES})

# ADD SUBDIRECTORY FOR PYTHON WRAPPER EXPORT
add_subdirectory(src)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

