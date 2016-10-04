#ifndef _BOB_CONTROL_FRONT_OBSTACLE_CONDITION_H_
#define _BOB_CONTROL_FRONT_OBSTACLE_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>

namespace bob
{

	class FrontObstacleCondition : public IStopCondition
	{

		public:

			FrontObstacleCondition(float minDistance) :
			minDistance(minDistance) {}

	
		private:

			virtual bool condition(const ISensorHandle& sensorHandle);			

			float minDistance;

	};

}

#endif
