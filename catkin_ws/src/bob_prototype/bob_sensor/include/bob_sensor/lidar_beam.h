#ifndef _BOB_SENSOR_LIDAR_BEAM_H_
#define _BOB_SENSOR_LIDAR_BEAM_H_

namespace bob
{

	//! \brief An individual Lidar Beam
	//! Used as a return value during iteration
	struct LidarBeam
	{
		LidarBeam(float angle=0, float range=0) :
		angle(angle),
		range(range) {}

		float angle;
		float range;
	};


}

#endif
