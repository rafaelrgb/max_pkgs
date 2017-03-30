#include "joystick.hh"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#define MAX_LINEAR 5.0
#define MAX_ANGULAR 5.0

int main(int argc, char **argv)
{
	// Create an instance of Joystick
	Joystick joystick("/dev/input/js0");

	// Ensure that it was found and that we can use it
	if (!joystick.isFound())
	{
    printf("Joystick open failed.\n");
		exit(1);
	}
  
  // Init the ros node
	ros::init(argc, argv, "max_teleop_node");
	ros::NodeHandle n;
	
	// Create a publisher to send the movement command msgs
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Rate loop_rate(1000);

  double x = 0.0, z = 0.0;

  while (ros::ok())
	{
		// Create the movement command msg
    geometry_msgs::Twist msg;
		
		// Attempt to sample an event from the joystick
		JoystickEvent event;
		if (joystick.sample(&event))
		{
      if (event.isAxis())
		  {
        if (event.number == 3)
        {
          x = -1.0 * static_cast<double>(event.value) * MAX_LINEAR / 32767.0;
        }
        if (event.number == 2)
        {
          z = static_cast<double>(event.value) * MAX_ANGULAR / 32767.0;
        }

        // Publish the message
        msg.linear.x = x;
        msg.angular.z = z;
        cmd_vel_pub.publish(msg);
		  }
		}

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
