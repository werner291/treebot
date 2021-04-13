cmake_minimum_required(VERSION 3.0.2)
project(treebot)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES convert_pcl
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

## Create the nodelet tutorial library
add_executable(treebot src/treebot.cpp)
target_link_libraries(treebot ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(treebot ${catkin_EXPORTED_LIBRARIES})
endif()

#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)
#
## Mark the nodelet library for installations
#install(TARGETS convert_pcl
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#
### Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES convert_pcl.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})