#include <ros/ros.h>

#include <bob_ros_implementations/ros_system_handle.h>

#include <bob_high_level/demo.h>

using namespace bob;
	
int main(int argc, char** argv)
{
	ROSSystemHandle systemHandle(argc, argv, "demo");

	// Running demo with map
	Demo demo(systemHandle.getSensorHandle(), systemHandle.getLockableMap(), systemHandle.getVelocityPublisher());
	demo();

	return 0;
}

