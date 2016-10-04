#ifndef _BOB_CONTROL_PROXIMITY_CONDITION_H_
#define _BOB_CONTROL_PROXIMITY_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>
#include <bob_toolbox/pose2d.h>

namespace bob
{

	class ProximityCondition : public IStopCondition
	{

		public:

			ProximityCondition(WorldPoint point, float distance) : 
				point(point),
				distance(distance)
		{}

		private:

			virtual bool condition(const ISensorHandle& sensorHandle);	

			WorldPoint point;

			float distance;

	};

}

#endif
