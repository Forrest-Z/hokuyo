#ifndef _BOB_WALL_FOLLOWING_WALL_FOLLOW_BACK_CONDITION_H_
#define _BOB_WALL_FOLLOWING_WALL_FOLLOW_BACK_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>
#include <bob_toolbox/world_point.h>

namespace bob
{
	class WallFollowBackCondition: public IStopCondition
	{
		public:
			WallFollowBackCondition(WorldPoint startPosition, float moveOutRadius, float moveBackRadius):
				startPosition(startPosition),
				moveOutRadius(moveOutRadius),
				moveBackRadius(moveBackRadius),
				moveOut(false)
			{}

		private:
			virtual bool condition(const ISensorHandle& sensorHandle);
			
			WorldPoint startPosition;

			float moveOutRadius;
			float moveBackRadius;

			bool moveOut;
	};

}

#endif
