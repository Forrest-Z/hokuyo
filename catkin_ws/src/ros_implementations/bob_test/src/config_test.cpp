#include <bob_config/config.h>
#include <ros/ros.h>
#include <bob_toolbox/logging.h>

using namespace bob;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "config_test");
	ros::NodeHandle nh;

	ros::Duration(1.0).sleep();
	

	float configedEgg = Config::SLAM_TF_DELAY;
	LOG_TEST("Egg: " << configedEgg);

	float radius = Config::ROBOT_RADIUS;
	LOG_TEST("Radius: " << radius);

//	float test = oldConfig("FAKE")("TEST");

	ros::spin();
}
