cmake_minimum_required(VERSION 3.10)
project(auto_exchange_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
	moveit_core
	pluginlib
	roscpp
	std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS
	include
	${EIGEN3_INCLUDE_DIR}
  	LIBRARIES
  	CATKIN_DEPENDS
	roscpp
	std_msgs
  	DEPENDS
)

include_directories(include)
include_directories(
	SYSTEM
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

#add_executable(test_auto_exchange  src/test_auto_exchange.cpp)
#target_link_libraries(test_auto_exchange
#	${catkin_LIBRARIES}
#	${Boost_LIBRARIES}
#)

add_library(moveit_auto_exchange_planner_plugin
	src/auto_exchange_manager.cpp
	src/auto_exchange_planner.cpp
	src/auto_exchange_context.cpp)
set_target_properties(moveit_auto_exchange_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_auto_exchange_planner_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Mark executables and/or libraries for installation
install(
	TARGETS
	moveit_auto_exchange_planner_plugin
	ARCHIVE DESTINATION
	${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION
	${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION
	${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
	DIRECTORY
	include
	DESTINATION
	${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Mark roslaunch files for installation
install(
	DIRECTORY
	launch
	DESTINATION
	${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#catkin_lint ignore uninstalled_plugin
install(FILES auto_exchange_planner_plugin_description.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

