#ifndef _BOB_CONTROL_HEADING_CONDITION_H_
#define _BOB_CONTROL_HEADING_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>

namespace bob
{

	class HeadingCondition : public IStopCondition
	{

		public:

			HeadingCondition(float desiredHeading, float tolerance) :
				desiredHeading(desiredHeading),
				tolerance(tolerance),
				initialized(false)
		{}

		private:	

			virtual bool condition(const ISensorHandle& sensorHandle);

			float desiredHeading;
		
			float tolerance;
			
			float initialDifference;

			bool initialized;

	};

}

#endif
