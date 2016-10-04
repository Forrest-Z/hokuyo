#ifndef _BOB_CONTROL_HALF_SPACE_CONDITION_H_
#define _BOB_CONTROL_HALF_SPACE_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>

#include <bob_toolbox/map_half_space.h>

namespace bob
{

	class HalfSpaceCondition : public IStopCondition
	{

		public:

			HalfSpaceCondition(MapHalfSpace space) :
				space(space)
		{}

		private:

			virtual bool condition(const ISensorHandle& sensorHandle);

			MapHalfSpace space;
			
	};

}

#endif
