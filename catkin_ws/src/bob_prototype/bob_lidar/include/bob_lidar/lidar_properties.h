#ifndef _BOB_LIDAR_LIDAR_PROPERTIES_H_
#define _BOB_LIDAR_LIDAR_PROPERTIES_H_

namespace bob
{

	struct LidarProperties
	{
		//! Null constructor to allow delayed initialization
		LidarProperties() {}

		LidarProperties(float minAngle, float angleIncrement, int beamCount) :
			minAngle(minAngle),
			angleIncrement(angleIncrement),
			beamCount(beamCount)
		{}


		float minAngle;
		float angleIncrement;

		int beamCount;
		
	};

}

#endif
