#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include <WProgram.h>
#endif

#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <ros/time.h>
#include <ros.h>

int potPin = 0;
int enablePin = 2;
std_msgs::UInt16 angle_;

ros::NodeHandle nh;

void updateCommand(const std_msgs::UInt8& cmd_msg)
{
	digitalWrite(enablePin, HIGH);
	analogWrite(3, cmd_msg.data);
}

void updateAngle()
{
	angle_.data = analogRead(potPin);
}

ros::Subscriber<std_msgs::UInt8> sub_command_("arduino_cmd", &updateCommand);
ros::Publisher pub_angle_("potentiometer_msr", &angle_);

void setup()
{
	nh.initNode();
	nh.subscribe(sub_command_);
	nh.advertise(pub_angle_);

	while(!nh.connected())
	{
		nh.spinOnce();
		delay(100);
	}

	pinMode(enablePin, OUTPUT);
}

void loop()
{
	updateAngle();
	pub_angle_.publish(&angle_);
	nh.spinOnce();
	delay(1);
}