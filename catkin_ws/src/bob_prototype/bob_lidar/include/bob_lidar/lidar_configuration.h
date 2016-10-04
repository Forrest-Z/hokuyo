#ifndef _BOB_LIDAR_LIDAR_CONFIGURATION_H_
#define _BOB_LIDAR_LIDAR_CONFIGURATION_H_

namespace bob
{

	struct LidarConfiguration
	{
		//! Scan params in ROBOT frame
		float robotAngleMin, robotAngleMax;
		float robotAngleIncrement;
	};

}

#endif
