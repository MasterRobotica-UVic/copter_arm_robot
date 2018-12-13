#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include <WProgram.h>
#endif

#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <ros/time.h>
#include <ros.h>

int msrPin = 2;
int directionPin = 29;
int cmdPin = 3;
std_msgs::Int16 angle_;
int directDrivePin = 0;
int inputPin = 1;
std_msgs::Float64 joystick_;

// debug
std_msgs::Int16 cmd_mapped_;

void updateCommand(const std_msgs::Int16& cmd_msg)
{
	// convert from signed to unsigned with direction
	int cmd;
	if( cmd_msg.data > 0 )
	{
		digitalWrite(directionPin, HIGH);
		cmd = map(cmd_msg.data, 0, 255, 0, 255);
	}
	else
	{
		digitalWrite(directionPin, LOW);
		cmd = map(cmd_msg.data, -255, 0, 255, 0);
	}
	cmd_mapped_.data = cmd;
	analogWrite(cmdPin, cmd);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> sub_command_("arduino_cmd", &updateCommand);
ros::Publisher pub_angle_("potentiometer_msr", &angle_);
ros::Publisher pub_cmd_mapped_("arduino_cmd_mapped", &cmd_mapped_);
ros::Publisher pub_joystick_setpoint_("joystick_setpoint", &joystick_);

void updateAngle()
{
	angle_.data = analogRead(msrPin);
}

void updateJoystick()
{
	// from -90 to 90 degress, but in radians
	joystick_.data = map(analogRead(inputPin), 0, 1024, -512, 512)*1.57/512.0;

}

void publish()
{
	pub_angle_.publish(&angle_);
	pub_cmd_mapped_.publish(&cmd_mapped_);
	pub_joystick_setpoint_.publish(&joystick_);
}

void setup()
{
	nh.initNode();
	nh.subscribe(sub_command_);
	nh.advertise(pub_angle_);
	nh.advertise(pub_cmd_mapped_);
	nh.advertise(pub_joystick_setpoint_);

	// try one more time after 2 seconds
	while(!nh.connected())
	{
		nh.spinOnce();
		delay(2000);
	}

	digitalWrite(directionPin, HIGH);
	pinMode(directionPin, OUTPUT);
	analogWrite(cmdPin, 0);
}

void loop()
{
	// direct drive via joystick, only write if there is a significant inpu to not interfere with PID
	int input = analogRead(directDrivePin);
	int cmd;
	if( input > 522 )
	{
		digitalWrite(directionPin, HIGH);
		cmd = map(input, 522, 1024, 0, 255);
		analogWrite(cmdPin, cmd);
	}
	else if ( input < 502 )
	{
		digitalWrite(directionPin, LOW);
		cmd = map(input, 0, 502, 255, 0);
		analogWrite(cmdPin, cmd);
	}

	// ros section
	if( nh.connected() )
	{
		updateAngle();
		updateJoystick();
		publish();
		nh.spinOnce();
	}
	else
	{
		analogWrite(cmdPin, 0);
	}

	delay(2);
}
