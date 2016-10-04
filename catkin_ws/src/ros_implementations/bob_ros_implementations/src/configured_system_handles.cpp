#include <bob_ros_implementations/configured_system_handles.h>

#include <bob_ros_implementations/ros_flexible_system_handle.h>
#include <bob_ros_implementations/ros_limited_lidar_handle.h>

#include <bob_sensor/isensor_handle.h>
#include <bob_sensor/itransform_handle.h>

#include <memory>

namespace bob
{

	std::unique_ptr<ISystemHandle> limitedLidarSystemHandle(SimpleAngularRange limit)
	{
		std::unique_ptr<ROSFlexibleSystemHandle> systemHandle(new ROSFlexibleSystemHandle());

		// Switching out the lidar to the limited version
		std::unique_ptr<IScanSensorHandle> limitedLidarHandle(new ROSLimitedLidarHandle(systemHandle->getSensorHandle().getTransformHandle(), limit));
		systemHandle->resetScanSensorHandle(std::move(limitedLidarHandle));

		systemHandle->start();

		return std::move(systemHandle);
	}

}

