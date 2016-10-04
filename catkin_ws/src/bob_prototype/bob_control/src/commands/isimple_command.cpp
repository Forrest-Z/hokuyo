#include <bob_control/commands/isimple_command.h>

#include <bob_control/conditions/bumper_condition.h>

namespace bob
{

	Runnable ISimpleCommand::generateRunnable(const ISensorHandle& sensorHandle)
	{
		IStopCondition* obstacleCondition = NULL;
		IStopCondition* goalCondition = NULL;
			
		OredCondition::shared_ptr condition = this->generateCondition(sensorHandle, obstacleCondition, goalCondition);
		IController::shared_ptr controller = this->generateController(sensorHandle);

		Runnable runnable(controller, condition);
		runnable.obstacleCondition = obstacleCondition;
		runnable.goalCondition = goalCondition;

		if (bumperEnabled)
		{
			// Add bumper if enabled
			BumperCondition::shared_ptr bumper(new BumperCondition());
			condition->add(bumper);
			runnable.bumperCondition = bumper.get();
		}
		
		return runnable;
	}

}

