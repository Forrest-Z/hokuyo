#include <bob_slam/slam_ros.h>

#include <bob_lidar/util_functions.h>

#include <bob_sensor/isensor_handle.h>

#include <bob_config/config.h>

#include <bob_system/system_utilities.h>

#include <bob_sensor/iscan_sensor_handle.h>

namespace bob
{

	SlamROS::SlamROS(ISensorHandle& sensorHandle, MapSender& mapSender) :
		sensorHandle(sensorHandle),
		slamProcessor(mapSender, sensorHandle.getTransformNotifier()),
		mapSender(mapSender)
	{}

	void SlamROS::operator()()
	{
		while (systemUtilities->ok())
		{
			LidarScan laserData = sensorHandle.getScanSensorHandle().getLidarData();

			Pose2D odomPose = sensorHandle.getTransformHandle().getOdomPose();

			slamProcessor.addScan(odomPose, laserData);

			mapSender.update();
		}
	}

}
