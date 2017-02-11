#include <wiringPi.h>
#include "ros/ros.h"
#include "max_msgs/MovementCommand.h"

// Function to move the robot forward
void forward()
{
    digitalWrite(0, LOW);
    digitalWrite(1, HIGH);
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
}

// Function to move the robot backward
void backward()
{
    digitalWrite(0, HIGH);
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
}

// Function to stop the robot
void stop() 
{
    digitalWrite(0, LOW);
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
}

// Function to rotate the robot to the right
void right()
{
    digitalWrite(0, LOW);
    digitalWrite(1, HIGH);
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
}

// Function to rotate the robot to the left
void left()
{
    digitalWrite(0, HIGH);
    digitalWrite(1, LOW);
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
}

// Callback function to the Movement Command subscriber
void movementCommandCb(const max_msgs::MovementCommand::ConstPtr& msg)
{
  switch( msg.command )
  {
	  case 0:
		forward();
		break;
	  case 1:
		backward();
		break;
	  case 2:
		stop();
		break;
	  case 3:
		left();
		break;
	  case 4:
		right();
		break;
	  default:
		stop();
  }
}

int main(int argc, char** argv)
{
  // Configure GPIO pins
  wiringPiSetup();
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  
  // Init the ROS node
  ros::init(argc, argv, "motor_driver");
  ros::NodeHandle n;
  
  // Subscribe to the movement command topic
  ros::Subscriber movement_command_sub = n.subscribe("movement_command", 1000, movementCommandCb);
  
  ros::spin();
  
  return 0;
}
