#ifndef _BOB_CONTROL_DISTANCE_ROBOT_DIRECTION_CONDITION_H_
#define _BOB_CONTROL_DISTANCE_ROBOT_DIRECTION_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>

namespace bob
{

	class DistanceRobotDirectionCondition : public IStopCondition
	{

		public:

			DistanceRobotDirectionCondition(float minDistance, float directionAngle) :
			minDistance(minDistance),
			angle(directionAngle)
			{}

		private:

			virtual bool condition(const ISensorHandle& sensorHandle);

			float minDistance;

			float angle;

	};

}

#endif
