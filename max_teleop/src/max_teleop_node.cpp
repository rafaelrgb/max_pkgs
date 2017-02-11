#include "joystick.hh"
#include "ros/ros.h"
#include "max_msgs/MovementCommand.h"

int main(int argc, char **argv)
{
	// Create an instance of Joystick
	Joystick joystick("/dev/input/js0");

	// Ensure that it was found and that we can use it
	if (!joystick.isFound())
	{
		printf("open failed.\n");
		exit(1);
	}
  
	// Init te ros node
	ros::init(argc, argv, "max_teleop_node");
	ros::NodeHandle n;
	
	// Create a publisher to send the movement command msgs
	ros::Publisher movement_command_pub = n.advertise<max_msgs::MovementCommand>("movement_command", 1000);

	ros::Rate loop_rate(10);


  // Variable to store the last command
  int command = max_msgs::MovementCommand::STOP;

	while (ros::ok())
	{
		// Create the movement command msg
    max_msgs::MovementCommand msg;
		
		// Attempt to sample an event from the joystick
		JoystickEvent event;
		if (joystick.sample(&event))
		{
		  if (event.isButton())
		  {
        if (event.value == 0)
        {
          command = max_msgs::MovementCommand::STOP;
        } else if (event.number == 0)
        {
          command = max_msgs::MovementCommand::FORWARD;
        } else if (event.number == 1)
        {
          command = max_msgs::MovementCommand::RIGHT;
        } else if (event.number == 2)
        {
          command = max_msgs::MovementCommand::BACKWARD;
        } else if (event.number == 3)
        {
          command = max_msgs::MovementCommand::LEFT;
        }
		  }
		}
		
		// Publish the message
    msg.command = command;
		movement_command_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
