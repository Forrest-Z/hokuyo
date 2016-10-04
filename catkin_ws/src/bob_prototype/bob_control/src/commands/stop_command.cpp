#include <bob_control/commands/stop_command.h>
#include <bob_control/velocity_repeater.h>
#include <bob_control/conditions/controller_loop_condition.h>

namespace bob
{

	IController::shared_ptr StopCommand::generateController(const ISensorHandle& sensorHandle) const
	{
		VelocityRepeater::shared_ptr velocityRepeater(new VelocityRepeater(Velocity2D()));
		return velocityRepeater; 
	}

	OredCondition::shared_ptr StopCommand::generateCondition(const ISensorHandle& sensorHandle, IStopCondition*& obstacleCondition, IStopCondition*& goalCondition) const
	{	
		OredCondition::shared_ptr condition(new OredCondition());

		// Execute once;
		ControllerLoopCondition::shared_ptr controllerLoopCondition(new ControllerLoopCondition(1));
		condition->add(controllerLoopCondition);		

		goalCondition = controllerLoopCondition.get();
		
		return condition;
	}
	
}

