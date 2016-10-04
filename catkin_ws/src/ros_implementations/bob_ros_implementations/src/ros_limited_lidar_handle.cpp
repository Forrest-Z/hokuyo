#include <bob_ros_implementations/ros_limited_lidar_handle.h>

#include <bob_toolbox/angular_range.h>
#include <bob_toolbox/logging.h>

namespace bob
{

	LidarScan ROSLimitedLidarHandle::getLidarData() const
	{
		return ROSLidarHandle::getLidarData(limit, CounterClockwise);
	}

	LidarScan ROSLimitedLidarHandle::getLidarData(const SimpleAngularRange& angularRange, CircularDirection direction) const
	{
		SimpleAngularRange actualRange(angularRange);
		if (actualRange.limitBy(limit))
		{
			return ROSLidarHandle::getLidarData(actualRange, direction);
		}
		else
		{
			// The range has been clipped to nothing
			return LidarScan();
		}	
	}

}

