#include <ros/ros.h>

#include <bob_wall_following/wall_follower.h>
#include <bob_ros_implementations/ros_system_handle.h>

using namespace bob;

int main(int argc, char** argv)
{
	ROSSystemHandle systemHandle(argc, argv, "lidar_listener_test");	
	
	WallFollower follower(systemHandle.getControlHandle().simpleCommander, systemHandle.getSensorHandle(), LeftSide);
	follower.run();

	return 0;
}
