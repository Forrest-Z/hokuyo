#include <ros/ros.h>
#include <bob_ros_implementations/ros_transform_handle.h>
#include <bob_toolbox/easy_print.h>
#include <bob_toolbox/logging.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "transform_test");
	ros::NodeHandle n;

	LOG_TEST("TEST");
	ROSTransformHandle transformHandle;
	while (ros::ok())
	{
		LOG_TEST("Odom pose: " << transformHandle.getOdomPose());	
		LOG_TEST("Localized pose: " << transformHandle.getLocalizedPose());	
		ros::Duration(0.1).sleep();
	}
	LOG_TEST("TEST");
}
