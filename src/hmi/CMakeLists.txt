cmake_minimum_required(VERSION 3.0.2)
project(hmi)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  cmake_modules
  moveit_commander 
  moveit_core
  moveit_fake_controller_manager
  moveit_msgs
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_robot_interaction
  moveit_visual_tools
)

# Ensure compatibility for CMP0072
cmake_policy (SET CMP0072 OLD)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
  include/imgui
  include/imgui/backends
  include/hmi
  ${GLFW3_INCLUDE_DIR}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hmi_custom_lib
  LIBRARIES hmi_sensibility_classes
  CATKIN_DEPENDS roscpp rospy std_msgs
)

# Add a library for sensor fusion
add_library(hmi_custom_lib
  include/imgui/imgui.cpp
  include/imgui/imgui_draw.cpp
  include/imgui/imgui_tables.cpp
  include/imgui/imgui_widgets.cpp
  include/imgui/backends/imgui_impl_glfw.cpp
  include/imgui/backends/imgui_impl_opengl3.cpp
)

# Add a library for sensor fusion
add_library(hmi_sensibility_classes
  include/hmi/Main_Application.cpp
  include/hmi/Control_Panel.cpp
  include/hmi/Measurement_Panel.cpp
  include/hmi/Plot_Panel.cpp
)

# Link the sensor fusion library with the catkin libraries
target_link_libraries(hmi_custom_lib ${catkin_LIBRARIES} glfw OpenGL::GL)
target_link_libraries(hmi_sensibility_classes ${catkin_LIBRARIES} glfw OpenGL::GL)

find_package(moveit_visual_tools REQUIRED)
include_directories(${moveit_visual_tools_INCLUDE_DIRS})


############################################################
# CMakelist for advertise_topics_for_UR node
############################################################

# Add an executable for the main node
add_executable(main src/main.cpp)

# Link the main executable with the sensor fusion library and catkin libraries
target_link_libraries(main hmi_custom_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} Python3::Python Python3::NumPy)

# Add dependencies for the main executable
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

############################################################
# CMakelist for advertise_topics_for_UR node
############################################################

# Add an executable for the hmi node
add_executable(hmi src/hmi.cpp)

# Link the hmi executable with the sensor fusion library and catkin libraries
target_link_libraries(hmi hmi_custom_lib hmi_sensibility_classes ${catkin_LIBRARIES} ${OpenCV_LIBS} Python3::Python Python3::NumPy)

# Add dependencies for the hmi executable
add_dependencies(hmi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Additional include directories for the project
target_include_directories(${PROJECT_NAME} PUBLIC include/imgui include/imgui/backends ${OPENGL_INCLUDE_DIR})
