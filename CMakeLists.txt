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

set(TREEBOT_NONMAIN_SRC src/look_forward.cpp src/conversions.cpp src/moveit_interaction.cpp src/state_spaces.cpp src/FourierSeriesPlanner.cpp src/DroneControlSpace.cpp)

add_executable(treebot src/treebot.cpp ${TREEBOT_NONMAIN_SRC})
target_link_libraries(treebot ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(treebot ${catkin_EXPORTED_LIBRARIES})
endif()

#########################
# Treebot BFMT test thing #
#########################

add_executable(treebot_bfmt src/ompl_bfmt_testcase.cpp)
target_link_libraries(treebot_bfmt ${OMPL_LIBRARIES})

#################
# Treebot tests #
#################

add_executable (treebot_tests test/test.cpp ${TREEBOT_NONMAIN_SRC})
target_include_directories(treebot_tests PRIVATE src)
target_link_libraries (treebot_tests ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OMPL_LIBRARIES})

###########################################
# CopelliaSim trajectory execution plugin #
###########################################

set(COPPELLIA_ROOT /home/werner/CoppeliaSim_Edu_V4_1_0_Ubuntu18_04)
add_library(simExtTreebotController
        src/treebot_controller.cpp
        ${COPPELLIA_ROOT}/programming/common/simLib.cpp)
target_include_directories(simExtTreebotController PRIVATE
        ${COPPELLIA_ROOT}/programming/include
        ${COPPELLIA_ROOT}/programming/simMath)
target_link_libraries(simExtTreebotController ${catkin_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(simExtTreebotController ${catkin_EXPORTED_LIBRARIES})
endif()## Create the nodelet tutorial library

#################################
# Empty joint message publisher #
#################################

add_executable(dummy_joint_messages src/dummy_joint_messages.cpp)
target_link_libraries(dummy_joint_messages ${catkin_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(dummy_joint_messages ${catkin_EXPORTED_LIBRARIES})
endif()