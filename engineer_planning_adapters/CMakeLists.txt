cmake_minimum_required(VERSION 3.10)
project(engineer_planning_adapters)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
)

catkin_package(
    INCLUDE_DIRS
    include
    CATKIN_DEPENDS
    roscpp
    moveit_core
    moveit_ros_planning
    pluginlib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/special_joint_constraint_adapter.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES planning_adapter_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)