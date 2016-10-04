#include <ros/ros.h>

#include <bob_ros_implementations/ros_system_handle.h>

using namespace bob;

int main(int argc, char** argv)
{
	ROSSystemHandle systemHandle(argc, argv, "slam");

	while (ros::ok())
	{
		ros::Duration(1.0).sleep();
	}

	return(0);
}

