#include <wiringPi.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#define IN1 0
#define IN2 1
#define IN3 2
#define IN4 3
#define ENA 4
#define ENB 5

#define MAX_LINEAR 5.0
#define MAX_ANGULAR 5.0

#define R 0.065
#define L 0.132

// Function to drive the left wheel
void driveLeftWheel( double speed )
{
  if ( speed > 0 )
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else if ( speed > 0 )
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if ( speed == 0 )
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  softPwmWrite(ENA, speed);
}

// Function to drive the right wheel
void driveRightWheel( double speed )
{
  // Drive the left wheel forward with increading velocity
  if ( speed > 0 )
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else if ( speed > 0 )
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if ( speed == 0 )
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  softPwmWrite(ENB, speed);
}

// Callback function to the command velocity
void cmdVelCb(const geometry_msgs::Twist::ConstPtr& msg)
{
  double phi1, phi2;
  phi1 = (1.0 / R) * msg->linear.x + (L / R) * msg->angular.z;
  phi2 = (1.0 / R) * msg->linear.x + (-1.0 * L / R) * msg->angular.z;


  phi1 = phi1 * 100.0 / 87.077;
  phi2 = phi2 * 100.0 / 87.077;


  driveLeftWheel( phi1 );
  driveRightWheel( phi2 );
}

int main(int argc, char** argv)
{
  // Configure GPIO pins
  wiringPiSetup();
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  softPwmCreate(ENA, 0, 100);
  softPwmCreate(ENB, 0, 100);
  
  // Init the ROS node
  ros::init(argc, argv, "motor_driver");
  ros::NodeHandle n;
  
  // Subscribe to the movement command topic
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1000, cmdVelCb);
  
  ros::spin();
  
  return 0;
}
