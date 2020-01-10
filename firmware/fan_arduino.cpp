#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <ros/time.h>
#include <ros.h>

int msrPin = A1;
int directionPin = 2;
int cmdPin = 9;
std_msgs::UInt16 angle_;

void updateCommand(const std_msgs::UInt16& cmd_msg)
{
  // ensure 0 to 255 value
  int cmd;
  //digitalWrite(directionPin, HIGH);
  //cmd = map(cmd_msg.data, 0, 255, 0, 255);
  analogWrite(cmdPin, cmd_msg.data);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt16> sub_command_("arduino_cmd", &updateCommand);
ros::Publisher pub_angle_("potentiometer_msr", &angle_);

void updateAndPublishAngle()
{
  angle_.data = analogRead(msrPin);
  pub_angle_.publish(&angle_);
}

void setup()
{
  nh.initNode();
  nh.subscribe(sub_command_);
  nh.advertise(pub_angle_);

  // try one more time after 2 seconds
  while(!nh.connected())
  {
    nh.spinOnce();
    delay(2000);
  }

  pinMode(directionPin, OUTPUT);
  digitalWrite(directionPin, HIGH);
  // see https://etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
  TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
  analogWrite(cmdPin, 0);
}

void loop()
{
  // ros section
  if( nh.connected() )
  {
    updateAndPublishAngle();
    nh.spinOnce();
  }
  else
  {
    analogWrite(cmdPin, 0);
  }

  delay(2);
}
