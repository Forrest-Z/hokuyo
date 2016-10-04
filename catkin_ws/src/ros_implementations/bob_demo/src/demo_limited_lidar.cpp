#include <ros/ros.h>

#include <bob_ros_implementations/ros_flexible_system_handle.h>
#include <bob_ros_implementations/ros_limited_lidar_handle.h>

#include <bob_high_level/demo.h>

#include <bob_system/system_utilities.h>
#include <bob_toolbox/angular_range.h>

#include <bob_ros_implementations/configured_system_handles.h>

using namespace bob;
	
int main(int argc, char** argv)
{
	SimpleAngularRange scanRange(-M_PI / 2, M_PI / 2);
	std::unique_ptr<ISystemHandle> systemHandle = limitedLidarSystemHandle(scanRange);

	// Running demo with map
	//Demo demo(systemHandle.getSensorHandle(), systemHandle.getLockableMap(), systemHandle.getVelocityPublisher());
	//demo();

	while (true)
	{
		systemUtilities->sleep(1.0);
	}

	return 0;
}

