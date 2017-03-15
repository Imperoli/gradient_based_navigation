cmake_minimum_required(VERSION 2.4.6)
project(gradient_based_navigation)

find_package(catkin REQUIRED COMPONENTS
  #actionlib
  geometry_msgs
  roscpp
  roslib
  rospy
  std_msgs
  std_srvs
  tf
  dynamic_reconfigure
)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/GradientBasedNavigation.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# make sure configure headers are built before any node using them

add_executable(gradient_based_navigation src/gradient_based_navigation.cpp)
add_dependencies(gradient_based_navigation ${PROJECT_NAME}_gencfg)

add_executable(joystick_control src/joystick_control.cpp)
add_executable(sendAttractivePoints src/sendAttractivePoints.cpp)
add_executable(keyboard_control src/keyboard_control.cpp)

target_link_libraries(gradient_based_navigation ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(joystick_control ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(sendAttractivePoints ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(keyboard_control ${catkin_LIBRARIES} ${OpenCV_LIBS})

