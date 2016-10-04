#include <bob_control/commands/heading_rotation_command.h>

#include <bob_control/conditions/ored_condition.h>
#include <bob_control/conditions/heading_condition.h>
#include <bob_control/rotation_controller.h>

namespace bob
{

	IController::shared_ptr HeadingRotationCommand::generateController(const ISensorHandle& sensorHandle) const
	{
		RotationController::shared_ptr controller(new RotationController());
		controller->initAngularGoal(angle);
		return controller; 
	}

	OredCondition::shared_ptr HeadingRotationCommand::generateCondition(const ISensorHandle& sensorHandle, IStopCondition*& obstacleCondition, IStopCondition*& goalCondition) const
	{
		OredCondition::shared_ptr condition(new OredCondition());
		condition->add(additionalCondition);

		HeadingCondition::shared_ptr headingCondition(new HeadingCondition(angle, 0.05));	
		condition->add(headingCondition);		

		goalCondition = headingCondition.get();
		return condition;

	}

}

