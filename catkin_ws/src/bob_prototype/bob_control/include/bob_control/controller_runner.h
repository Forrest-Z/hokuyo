#ifndef _BOB_CONTROL_CONTROLLER_RUNNER_H_
#define _BOB_CONTROL_CONTROLLER_RUNNER_H_

#include <bob_toolbox/linear_range.h>
#include <bob_config/config.h>

namespace bob
{

	class Runnable;
	class ISensorHandle;
	class IVelocityPublisher;
	class IStopCondition;
	class IController;

	//! This class is designed to be used with IController objects.

	//! It has a loop which will execute a controller at a certain rate.
	//! It gathers state of the system and then provides it to the controller
	//! in question.
	class ControllerRunner
	{

		public:

			ControllerRunner(const ISensorHandle& sensorHandle, IVelocityPublisher& velocityPublisher) :
				sensorHandle(sensorHandle),
				velocityPublisher(velocityPublisher),
				safeLinearVelocityRange(-Config::LINEAR_VELOCITY_BOUNDS, Config::LINEAR_VELOCITY_BOUNDS),
				safeAngularVelocityRange(-Config::ANGULAR_VELOCITY_BOUNDS, Config::ANGULAR_VELOCITY_BOUNDS)
		{}


			void run(Runnable& runnable);

		private:

			void run(IController& controller, IStopCondition& stopCondition);

			IVelocityPublisher& velocityPublisher;
	
			const ISensorHandle& sensorHandle;
	
			LinearRange<float> safeLinearVelocityRange;
			LinearRange<float> safeAngularVelocityRange;

	};

}

#endif
