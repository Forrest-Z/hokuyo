#ifndef _BOB_CONTROL_VELOCITY_REPEATOR_H_
#define _BOB_CONTROL_VELOCITY_REPEATOR_H_

#include <bob_control/icontroller.h>

#include <bob_toolbox/velocity2d.h>

namespace bob
{

	class VelocityRepeater : public IController
	{

		public:

			VelocityRepeater(Velocity2D velCommand):velCommand(velCommand) {};

			virtual Velocity2D nextCommand(const ISensorHandle& sensorHandle);

		private:	

			Velocity2D velCommand;

	};

}

#endif
