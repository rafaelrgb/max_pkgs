#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iomanip>

void boolMessageReceived( const std_msgs::Bool& msg ) {
	if( msg.data )
	{
		ROS_INFO_STREAM("True");
	}
	else
	{
		ROS_INFO_STREAM("False");
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "teste");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("teste", 1000,
		&boolMessageReceived);
	
	while(ros::ok())
	{
			ros::spinOnce();
	}
}
