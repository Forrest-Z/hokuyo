#include <bob_control/commands/velocity_repeater_command.h>

#include <bob_control/conditions/ored_condition.h>
#include <bob_control/conditions/front_obstacle_condition.h>
#include <bob_control/velocity_repeater.h>

namespace bob
{

	IController::shared_ptr VelocityRepeaterCommand::generateController(const ISensorHandle& sensorHandle) const
	{
		VelocityRepeater::shared_ptr velocityRepeator(new VelocityRepeater(velocity));	
		return velocityRepeator;
	}

	OredCondition::shared_ptr VelocityRepeaterCommand::generateCondition(const ISensorHandle& sensorHandle, IStopCondition*& obstacleCondition, IStopCondition*& goalCondition) const
	{
		OredCondition::shared_ptr condition(new OredCondition());
		condition->add(additionalCondition);
		return condition;
	}
}

