cmake_minimum_required(VERSION 3.0.2)
project(jaka_ros_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#set_target_properties(jaka_ros_driver PROPERTIES LINKER_LANGUAGE CXX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   geometry_msgs
   roscpp
   rospy
   sensor_msgs
   std_msgs
   message_generation
   std_srvs
   robot_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
#   ServoL.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Move.srv
#   Move_Jog.srv
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
   geometry_msgs   sensor_msgs   std_msgs std_srvs robot_msgs
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
   INCLUDE_DIRS include
   LIBRARIES jaka_ros_driver
   CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs message_runtime std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   src/libs
   src
)

## Declare a C++ library
#        add_library(${PROJECT_NAME}
##        src/libs/config.cpp
#                        src/connect_robot.cpp
#                        src/libs/JAKAZuRobot.h
#                        src/libs/jkerr.h
#                        src/libs/jktypes.h
#                   )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/jaka_ros_driver_node.cpp)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_jaka_ros_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#add_executable(publish_joint_states src/publish_joint_states.cpp)
#target_link_libraries(publish_joint_states ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
#target_link_libraries(publish_joint_states ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libz.a)
#target_link_libraries(publish_joint_states ${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)

add_executable(connect_robot src/connect_robot.cpp)
target_link_libraries(connect_robot ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
add_dependencies(connect_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(move_joint_client src/move_joint_client.cpp)
# target_link_libraries(move_joint_client ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(move_joint_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(move_line_client src/move_line_client.cpp)
# target_link_libraries(move_line_client ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(move_line_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(stop_move_client src/stop_move_client.cpp)
# target_link_libraries(stop_move_client ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(stop_move_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(enable_servo_client src/enable_servo_client.cpp)
# target_link_libraries(enable_servo_client ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(enable_servo_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(enable_teach_drag src/enable_teach_drag.cpp)
# target_link_libraries(enable_teach_drag ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(enable_teach_drag ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(subscriber_servo_line src/subscriber_servo_line.cpp)
# target_link_libraries(subscriber_servo_line ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(subscriber_servo_line ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(set_user_frame_client src/set_user_frame_client.cpp)
# target_link_libraries(set_user_frame_client ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(set_user_frame_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(set_tcp_client src/set_tcp_client.cpp)
# target_link_libraries(set_tcp_client ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(set_tcp_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(set_payload_client src/set_payload_client.cpp)
# target_link_libraries(set_payload_client ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(set_payload_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(clear_error_client src/clear_error_client.cpp)
# target_link_libraries(clear_error_client ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(clear_error_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(set_collision_client src/set_collision_client.cpp)
# target_link_libraries(set_collision_client ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(set_collision_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(subscriber_tool_point src/subscriber_tool_point.cpp)
# target_link_libraries(subscriber_tool_point ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(subscriber_tool_point ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(subscriber_joint_states src/subscriber_joint_states.cpp)
# target_link_libraries(subscriber_joint_states ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(subscriber_joint_states ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(subscriber_robot_state src/subscriber_robot_state.cpp)
# target_link_libraries(subscriber_robot_state ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(subscriber_robot_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(move_jog_client src/move_jog_client.cpp)
# target_link_libraries(move_jog_client ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(move_jog_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(disconnect_robot src/disconnect_robot.cpp)
# target_link_libraries(disconnect_robot ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
# add_dependencies(disconnect_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(robot_client src/robot_client.cpp)
target_link_libraries(robot_client  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
add_dependencies(robot_client  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(robot_sub src/robot_sub.cpp)
target_link_libraries(robot_sub ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/src/libs/libjakaAPI.so)
add_dependencies(robot_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})