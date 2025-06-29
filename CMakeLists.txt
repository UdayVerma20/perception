cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(perception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  pcl_conversions
  pcl_ros
  #pcl_segmentation
  pcl_msgs
  sensor_msgs
  geometry_msgs
  gazebo_msgs
  # cv_bridge
  # image_transport
)

## System dependencies are found with CMake's conventions
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(OpenCV REQUIRED)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

### Generate messages in the 'msg' folder
add_message_files(
  FILES
  CoordinateList.msg
  Coordinates.msg
  uday.msg
  imu.msg
  pid_input.msg
  path_coordinates.msg
  landmark_points.msg
  stateHeader.msg
  deltaHeader.msg
   
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  gazebo_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception
  CATKIN_DEPENDS message_runtime 
  roscpp std_msgs sensor_msgs rospy gazebo_msgs
  # geometry_msgs  cv_bridge image_transport
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 #include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
#  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/perception.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

#Extra
add_executable (reconstruction Extra/reconstruction.cpp)
target_link_libraries (reconstruction ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable (SensorFusion Extra/SensorFusion.cpp)
target_link_libraries (SensorFusion ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable (CameraBypass Extra/CameraBypass.cpp)
target_link_libraries (CameraBypass ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable (VoxelDownsample Extra/VoxelDownsample.cpp)
target_link_libraries (VoxelDownsample ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(Img Extra/Img.cpp)
target_link_libraries(Img ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(checkorder Extra/checkorder.cpp)
target_link_libraries(checkorder ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#Clustering
add_executable (Clustering Clustering/Clustering.cpp)
target_link_libraries (Clustering ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable (CustomClustering Clustering/CustomClustering.cpp)
target_link_libraries (CustomClustering ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable (customsim Clustering/customsim.cpp)
target_link_libraries (customsim ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#GroundRemoval
add_executable (DonSegmentation GroundRemoval/DonSegmentation.cpp)
target_link_libraries (DonSegmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable (ProgMorphFilter GroundRemoval/ProgMorphFilter.cpp)
target_link_libraries (ProgMorphFilter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable (LidarHeightRemoval GroundRemoval/LidarHeightRemoval.cpp)
target_link_libraries (LidarHeightRemoval ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(GroundRemoval GroundRemoval/GroundRemoval.cpp)
target_link_libraries(GroundRemoval ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(Lu GroundRemoval/Lu.cpp)
target_link_libraries(Lu ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(Amz GroundRemoval/Amz.cpp)
target_link_libraries(Amz ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(lusim GroundRemoval/lusim.cpp)
target_link_libraries(lusim ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#Mixed
add_executable (Mixed Merged/Mixed.cpp)
target_link_libraries (Mixed ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable (MixedCustom Merged/MixedCustom.cpp)
target_link_libraries (MixedCustom ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(AmzCluster Merged/AmzCluster.cpp)
target_link_libraries(AmzCluster ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(Amzconcentric Merged/Amzconcentric.cpp)
target_link_libraries(Amzconcentric ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_perception.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
