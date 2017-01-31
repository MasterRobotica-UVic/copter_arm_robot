cmake_minimum_required(VERSION 2.8.3)
project(copter_arm_robot)

find_package(catkin REQUIRED
	roscpp
	controller_manager
	hardware_interface
	joint_limits_interface
	transmission_interface
	control_toolbox
	urdf
	rosserial_arduino
	rosserial_client
	std_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_executable(fan_hwiface
	src/fan_hwiface.cpp
)

target_link_libraries(fan_hwiface
  ${catkin_LIBRARIES}
)

##############
## Firmware ##
##############

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware fan_arduino ALL)
rosserial_add_client_target(firmware fan_arduino-upload)