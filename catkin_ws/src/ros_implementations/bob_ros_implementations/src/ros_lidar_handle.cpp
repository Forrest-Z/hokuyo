#include <bob_ros_implementations/ros_lidar_handle.h>

#include <bob_toolbox/angular_range.h>

namespace bob
{

	LidarScan ROSLidarHandle::getLidarData() const
	{
		const sensor_msgs::LaserScan& scan = lidarListener.mostRecentScan();
		return lidarProcessor.process(scan);
	}

	LidarScan ROSLidarHandle::getLidarData(const SimpleAngularRange& angularRange, CircularDirection direction) const
	{
		const sensor_msgs::LaserScan& scan = lidarListener.mostRecentScan();
		return lidarProcessor.process(scan, angularRange, direction);
	}


}

