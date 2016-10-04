#ifndef _BOB_CONTROL_POINTING_TO_GOAL_CONDITION_H_
#define _BOB_CONTROL_POINTING_TO_GOAL_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>
#include <bob_toolbox/world_point.h>

namespace bob
{

	class PointingToGoalCondition: public IStopCondition
	{

		public:

			PointingToGoalCondition(WorldPoint goal): 
				goal(goal),
				initialized(false) {}

		private:

			virtual bool condition(const ISensorHandle& sensorHandle);
			
			WorldPoint goal;
			
			float initialAngleDiff;
			
			bool initialized;
	};

}

#endif
