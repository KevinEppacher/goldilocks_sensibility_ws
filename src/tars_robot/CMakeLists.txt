cmake_minimum_required(VERSION 3.0.2)
project(tars_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
)


find_package(OpenCV REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
  ${matplotlib_SOURCE_DIR}
  include
  include/classes
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES custom_lib
  CATKIN_DEPENDS roscpp rospy std_msgs dynamic_reconfigure
)

# Add a library for sensor fusion
add_library(custom_lib
  include/classes/Measurement.cpp
  include/classes/Robot.cpp
)

# Link the sensor fusion library with the catkin libraries
target_link_libraries(custom_lib ${catkin_LIBRARIES})


############################################################
# CMakelist for advertise_topics_for_UR node
############################################################

# Add an executable for the advertise_topics_for_UR program
add_executable(advertise_topics_for_UR src/advertise_topics_for_UR.cpp)

# Link the advertise_topics_for_UR program with the sensor fusion library and catkin libraries
target_link_libraries(advertise_topics_for_UR custom_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} Python3::Python Python3::NumPy)

# Add dependencies for the advertise_topics_for_UR program
add_dependencies(advertise_topics_for_UR ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

############################################################
# CMakelist for call_ur_setup_program node
############################################################

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/call_ur_setup_program.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################################
# CMakelist for call_ur_measurement_program node
############################################################

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/call_ur_measurement_program.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################################
# CMakelist for open_moveit_robot node
############################################################

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/open_moveit_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################################
# CMakelist for start_sensibility_measurements node
############################################################

# Add an executable for the start_sensibility_measurements program
add_executable(start_sensibility_measurements src/start_sensibility_measurements.cpp)

# Link the start_sensibility_measurements program with the sensor fusion library and catkin libraries
target_link_libraries(start_sensibility_measurements custom_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} Python3::Python Python3::NumPy)

# Add dependencies for the start_sensibility_measurements program
add_dependencies(start_sensibility_measurements ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
