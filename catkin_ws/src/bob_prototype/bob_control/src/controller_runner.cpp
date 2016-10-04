#include <bob_control/controller_runner.h>

#include <bob_toolbox/pose2d.h>
#include <bob_toolbox/velocity2d.h>

#include <bob_sensor/isensor_handle.h>
#include <bob_sensor/iodometry_handle.h>
#include <bob_sensor/itransform_handle.h>

#include <bob_control/icontroller.h>
#include <bob_control/conditions/istop_condition.h>
#include <bob_control/ivelocity_publisher.h>
#include <bob_control/runnable.h>

#include <bob_system/system_utilities.h>

namespace bob
{

	void ControllerRunner::run(IController& controller, IStopCondition& stopCondition)
	{
		RateTimer rateTimer = systemUtilities->rateTimer(controller.desiredFrequency());	

		while (systemUtilities->ok())
		{
			rateTimer->startTimer();

			if (stopCondition.isSatisfied(sensorHandle))
			{
				break;
			}

			// Get the next command from the controller
			Velocity2D newCommand = controller.nextCommand(sensorHandle);
	
			// Limit the command to within the safe range
			newCommand.x = safeLinearVelocityRange.limit(newCommand.x); 
			newCommand.w = safeAngularVelocityRange.limit(newCommand.w);

			// Publish the new command
			velocityPublisher.publish(newCommand);
			
			rateTimer->sleepRemaining();
		}
	}
	
	void ControllerRunner::run(Runnable& runnable)
	{
		run(*(runnable.controller), *(runnable.condition));
	}

}
