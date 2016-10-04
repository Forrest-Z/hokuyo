#include <bob_ros_implementations/lidar_util.h>

namespace bob
{

	LidarConfiguration determineLidarConfiguration(const ITransformHandle& transformHandle, const ROSLidarListener& lidarListener)
	{
		transformHandle.waitForLidarTransform();

		bool inverted;
		float zeroAngle;
		transformHandle.getLidarPose(inverted, zeroAngle);

		return determineLidarConfiguration(lidarListener, zeroAngle, inverted);
	}

	LidarConfiguration determineLidarConfiguration(const ROSLidarListener& lidarListener, float zeroAngle, bool inverted)
	{
		lidarListener.waitForScan();
		sensor_msgs::LaserScan msg = lidarListener.mostRecentScan();
		return determineLidarConfiguration(msg, zeroAngle, inverted);
	}

	LidarConfiguration determineLidarConfiguration(const sensor_msgs::LaserScan& msg, float zeroAngle, bool inverted)
	{
		LidarConfiguration config;
		config.robotAngleIncrement = msg.angle_increment;
		if (inverted)
		{
			// Laser frame is inverted
			config.robotAngleIncrement *= -1;
			config.robotAngleMin = zeroAngle - msg.angle_min;	
		}
		else
		{
			config.robotAngleMin = zeroAngle + msg.angle_min;	
		}

		config.robotAngleMax = config.robotAngleMin + msg.ranges.size() * config.robotAngleIncrement;
		return config;
	}

}
