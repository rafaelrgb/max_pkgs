#include <math.h>
#include <wiringPi.h>
#include "ros/ros.h"

#define D01 6
#define D02 7

int rpm1, rpm2;
int pulsos1, pulsos2;
unsigned int timeold;

unsigned int pulsos_por_volta = 20;

void contador1()
{
    pulsos1++;
}

void contador2()
{
    pulsos2++;
}

int main(int argc, char** argv)
{
  // Configure GPIO pins
  wiringPiSetup();
  pinMode(D01, INPUT);
  pinMode(D02, INPUT);
  wiringPiISR(D01, INT_EDGE_FALLING, contador1);
  wiringPiISR(D02, INT_EDGE_FALLING, contador2);

  //Initialize variables
  pulsos1 = pulsos2 = 0;
  rpm1 = rpm2 = 0;
  timeold = 0;

  // Init the ROS node
  ros::init(argc, argv, "velocity_sensor");
  ros::NodeHandle n;

  ROS_INFO_STREAM("Programa iniciado.");

  while( ros::ok() )
  {
    if(millis() - timeold >= 1000)
    {
      rpm1 = (60 * 1000 / pulsos_por_volta) / (millis() - timeold) * pulsos1;
      rpm2 = (60 * 1000 / pulsos_por_volta) / (millis() - timeold) * pulsos2;
      timeold = millis();
      pulsos1 = pulsos2 = 0;

      ROS_INFO_STREAM("RPM1 = " << rpm1 << " | " << "RPM2 = " << rpm2 << ";");
    }
  }

  return 0;
}
