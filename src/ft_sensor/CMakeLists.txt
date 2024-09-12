cmake_minimum_required(VERSION 3.0.2)
project(ft_sensor)

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
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs dynamic_reconfigure
)

############################################################
# CMakelist for airskin_state node
############################################################

# Add an executable for the airskin_state program
add_executable(airskin_state src/airskin_state.cpp)

# Link the airskin_state program with the sensor fusion library and catkin libraries
target_link_libraries(airskin_state custom_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} Python3::Python Python3::NumPy)

# Add dependencies for the advertise_topics_for_UR program
add_dependencies(airskin_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

############################################################
# CMakelist for zero_ft_sensor node
############################################################

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/zero_ft_sensor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)