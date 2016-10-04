#ifndef _BOB_CONTROL_LINEAR_ACCELERATE_CONTROLLER_H_
#define _BOB_CONTROL_LINEAR_ACCELERATE_CONTROLLER_H_

#include <bob_control/icontroller.h>
#include <bob_toolbox/world_point.h>

namespace bob
{

	class LinearAccelerateController: public IController
	{

		public:

			LinearAccelerateController(WorldPoint startPoint, float distance, float targetVel) :
			startPoint(startPoint),
			distance(distance),
			targetVel(targetVel),
			period(1.0 / desiredFrequency()),
			state(Initializing) {}


			virtual Velocity2D nextCommand(const ISensorHandle& sensorHandle);

		private:

			enum State
			{
				Initializing,
				Accelerating,
				Finished
			};
			
			WorldPoint startPoint;

			float distance;
			float initialAccelDir;
			float lastCommand;
			float targetVel;
			float period;
			
			State state;

	};

}

#endif
