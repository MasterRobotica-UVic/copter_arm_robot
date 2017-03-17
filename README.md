# Copter Arm Project

Project designed to learn PID control, relying on ros_control + gazebo for sim/real scenarios, heavily inspired by the example given in the [Introduction to Control System Design - A First Look](https://www.edx.org/course/introduction-control-system-design-first-mitx-6-302-0x) online course from MIT.

ToDo: link to video

## Hardware

<img src="https://docs.google.com/drawings/d/1uBFrBO40Ob0HcuH2-PCgJ1T4L5mbRx1GE2E4xeAF_D4/pub?w=960&h=720">

The list of components required to mount the real platform is:

* Linear potentiomer
* Arduino
* H bridge motor driver 
* Motor DC + Helix (They must agree, shaft, speed, power, etc.)
* 2x 2A / 7VDC supply
* Wires
* Protoboard (optional)
* Chienesse stick, wooden clothespin, ramplug and a wood strip (or whatever you find that suit a similar purpose)

If you don't have the money or the skills to build it, don't worry, you can use the simulation, see the Usage section below.

## Software

1. The package is built using the [Gazebo](http://gazebosim.org/)/[ROS](http://www.ros.org/) ecosystem, in particular, the ROS/Kinetic distro. Installation instruction can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).
2. After the installation, you need to create a ROS workspace, following [these instructions](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
3. In addition, this package requires the forked version of the [`teleop_tools/mouse_teleop`](https://github.com/MasterRobotica-UVic/teleop_tools) package. Thus, you need to clone the package using the following command in a terminal (after steps 1 and 2): `roscd && cd ../src/ && git clone https://github.com/MasterRobotica-UVic/teleop_tools.git`
4. Finally, get this package usingt he following command in a terminal: `roscd && cd ../src/ && git clone https://github.com/MasterRobotica-UVic/copter_arm_robot.git`
5. You are good to build: `roscd && cd .. && catkin_make`

## Usage

For the real scenario, connect everything, power up and type:

`roslaunch copter_arm_robot copter_arm_real.launch`

Or use the simulated scenario:

`roslaunch copter_arm_robot copter_arm_sim.launch`


## Description of the real scenario

The arduino board doesn't do any computation, it is just a hardware interface between PC and motor driver. If you take a peek in the code running as [firmware](firmware/fan_arduino.cpp) is pretty simple, it only [subscribes to the motor command](firmware/fan_arduino.cpp#L29) and [publishes the measured angle](firmware/fan_arduino.cpp#L30). The former is [forwarded directly to the pin](firmware/fan_arduino.cpp#L18-L22) where the motor driver is connected to, and the latter comes directly from the [pin where the potentiometer](firmware/fan_arduino.cpp#L24-L27) is connected to.

This part of the code is mainly composed of three functions. The [`init()`](src/fan_hwiface.cpp#L50-L93) function that takes care of initilize subscribers, publishers, and populate all (typical) memebers of a `hardware_interface::RobotHW` object. The [`read()`](src/fan_hwiface.cpp#L95-L107) that updates the measured values from the Arduino to the shared memory for the controller, and the [`write()`](src/fan_hwiface.cpp#L109-L124) that updates the commanded values from the controller shared memory for the Aruino.

The calibrated values of the potentiometer are in fact computed here as well [here](src/fan_hwiface.cpp#L171-L174). The gain and bias values are parameters that can be set in [this file](config/pot_calibration.yaml), where there is a brief explanation of how to calibrate a linear potentiometer. Note that, changing these values does not require to re-build the project.

It is worth to note that, in our scenario, there is a free joint (the one with the potentiometer) and an actuated joint (the one with the motor/helix couple), from the [URDF](http://wiki.ros.org/urdf) point of view. Thus, if one takes a look of the [controller configuration](config/controllers.yaml) there are two controllers that basically forward commands, and this is because in the ROS controls project there is no way to set such scenario for a PID control.

To overcome this issue, the generic [`pid`](http://wiki.ros.org/pid) is used. This is reflected in the [launch file](launch/copter_arm_real.launch#L36-L47). For this to work, the [`/control_effor`](launch/copter_arm_real.launch#L11) topic is remapped, the [`/setpoint`](launch/copter_arm_real.launch#L54) transformed from the mouse readings, and the [`/state`](launch/copter_arm_real.launch#L56) from the `/joint_states` topic.
