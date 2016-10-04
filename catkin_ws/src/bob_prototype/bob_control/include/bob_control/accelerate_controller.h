#ifndef _BOB_CONTROL_ACCELERATE_CONTROLLER_H_
#define _BOB_CONTROL_ACCELERATE_CONTROLLER_H_

#include <bob_control/icontroller.h>

namespace bob
{

	class AccelerateController : public IController
	{
		public:
			AccelerateController(Velocity2D targetVel, float linearAccel, float angularAccel, float decelFactor): 
			targetVel(targetVel), 
			linearSpeedLimit(0.15),
			angularSpeedLimit(5.4),
			linearAccel(linearAccel), 
			angularAccel(angularAccel),
			linearDecel(linearAccel * decelFactor),
			angularDecel(angularAccel * decelFactor) {}
	
			virtual Velocity2D nextCommand(const ISensorHandle& sensorHandle);
			
			float desiredFrequency()
			{
				return 20;
			}

		private:

			float sign(float x) { return x < 0.0 ? -1.0 : 1.0; };

			Velocity2D targetVel;

			float linearSpeedLimit;
			float angularSpeedLimit;
			float linearAccel;
			float angularAccel;
			float linearDecel;
			float angularDecel;
	};

}

#endif
