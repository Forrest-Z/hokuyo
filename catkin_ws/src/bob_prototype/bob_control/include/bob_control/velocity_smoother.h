#ifndef _BOB_CONTROL_VELOCITY_SMOOTHER_H_
#define _BOB_CONTROL_VELOCITY_SMOOTHER_H_

#include <bob_control/icontroller.h>
#include <bob_toolbox/velocity2d.h>
#include <bob_toolbox/pose2d.h>
#include <bob_config/robot_controller_config.h>

namespace bob
{

	class VelocitySmoother : public IController
	{

		public:

			VelocitySmoother(IController& controller, RobotControllerConfig& config); 
			
			virtual Velocity2D nextCommand(const ISensorHandle& sensorHandle);

		private:

			float sign(float x) { return x < 0.0 ? -1.0 : 1.0; };
			
			Velocity2D lastCommand;
			IController& controller;
			
			float speedLimV, accelLimV, decelLimV;
			float speedLimW, accelLimW, decelLimW;

	};

}

#endif
