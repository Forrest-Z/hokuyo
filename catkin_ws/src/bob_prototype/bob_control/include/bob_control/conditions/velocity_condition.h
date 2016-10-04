#ifndef _BOB_CONTROL_VELOCITY_CONDITION_H_
#define _BOB_CONTROL_VELOCITY_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>
#include <bob_toolbox/velocity2d.h>

namespace bob
{

	class VelocityCondition: public IStopCondition
	{

		public:

			VelocityCondition(Velocity2D targetVel): targetVel(targetVel) {}

		private:

			virtual bool condition(const ISensorHandle& sensorHandle); 

			Velocity2D targetVel;

	};

}

#endif
