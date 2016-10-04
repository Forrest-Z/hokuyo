#ifndef _BOB_WALL_FOLLOWING_WALL_FOLLOW_STUCK_CONDITION_H_
#define _BOB_WALL_FOLLOWING_WALL_FOLLOW_STUCK_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>

#include <bob_control/wall_follow_side.h>

#include <bob_sensor/lidar_scan.h>

namespace bob
{

	class WallFollowStuckCondition : public IStopCondition
	{

		public:

			WallFollowStuckCondition(WallFollowSide side) :
				side(side)
		{}

			LidarBeam beamToClear()
			{
				return beam;
			}

		private:

			virtual bool condition(const ISensorHandle& sensorHandle);

			LidarBeam beam;
			WallFollowSide side;

	};

}

#endif
