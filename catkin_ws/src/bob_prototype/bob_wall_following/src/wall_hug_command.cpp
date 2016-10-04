#include <bob_wall_following/wall_hug_command.h>
#include <bob_wall_following/wall_hugger.h>

namespace bob
{

	IController::shared_ptr WallHugCommand::generateController(const ISensorHandle& sensorHandle) const
	{
		IController::shared_ptr controller(new WallHugger(side));
		return controller;
	}

	OredCondition::shared_ptr WallHugCommand::generateCondition(const ISensorHandle& sensorHandle, IStopCondition*& obstacleCondition, IStopCondition*& goalCondition) const
	{
		OredCondition::shared_ptr condition(new OredCondition());
		condition->add(additionalCondition);
		return condition;
	}
}

