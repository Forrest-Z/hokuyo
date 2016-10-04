#ifndef _BOB_CONTROL_DISTANCE_WORLD_DIRECTION_CONDITION_H_
#define _BOB_CONTROL_DISTANCE_WORLD_DIRECTION_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>

namespace bob
{

	class DistanceWorldDirectionCondition : public IStopCondition
	{

		public:

			DistanceWorldDirectionCondition(float angle, float minDistance) :
				angle(angle),
				minDistance(minDistance)
				{}

		private:

			virtual bool condition(const ISensorHandle& sensorHandle);

			float angle;

			float minDistance;

	};

}

#endif
