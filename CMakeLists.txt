cmake_minimum_required(VERSION 3.0.2)
project(treebot)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_visual_tools
  actionlib_msgs
  moveit_simple_controller_manager
  std_msgs
)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

find_package(ompl REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES convert_pcl
  CATKIN_DEPENDS roscpp std_msgs moveit_core moveit_ros_planning_interface moveit_visual_tools moveit_simple_controller_manager actionlib sensor_msgs
  DEPENDS EIGEN3
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

#########################
# Treebot using moveit. #
#########################

set(TREEBOT_NONMAIN_SRC
        src/look_forward.cpp
        src/conversions.cpp
        src/moveit_interaction.cpp
        src/state_spaces.cpp
        src/FourierSeriesPlanner.cpp
        src/DroneControlSpace.cpp
        src/RealtimeTrajectoryExecutor.h
        src/RealtimeTrajectoryExecutor.cpp)

add_executable(treebot src/treebot.cpp ${TREEBOT_NONMAIN_SRC})
set_property(TARGET treebot PROPERTY CXX_STANDARD 17)
target_link_libraries(treebot ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(treebot ${catkin_EXPORTED_LIBRARIES})
endif()

##############################
# Treebot path-follower node #
##############################

add_executable(treebot_followpath src/treebot_followpath.cpp)
set_property(TARGET treebot_followpath PROPERTY CXX_STANDARD 17)
target_link_libraries(treebot_followpath ${catkin_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(treebot_followpath ${catkin_EXPORTED_LIBRARIES})
endif()

#################
# Treebot tests #
#################

add_executable (treebot_tests test/test.cpp ${TREEBOT_NONMAIN_SRC})
target_include_directories(treebot_tests PRIVATE src)
target_link_libraries (treebot_tests ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})

#################################
# Empty joint message publisher #
#################################

add_executable(sinusoidal_occupancy src/sinusoidal_occupancy.cpp)
target_link_libraries(sinusoidal_occupancy ${catkin_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(sinusoidal_occupancy ${catkin_EXPORTED_LIBRARIES})
endif()