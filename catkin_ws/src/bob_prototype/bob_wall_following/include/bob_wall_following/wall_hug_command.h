#ifndef _BOB_WALL_FOLLOWING_WALL_HUG_COMMAND_H_
#define _BOB_WALL_FOLLOWING_WALL_HUG_COMMAND_H_

#include <bob_sensor/isensor_handle.h>
#include <bob_control/commands/isimple_command.h>
#include <bob_control/conditions/null_condition.h>
#include <bob_control/wall_follow_side.h>

namespace bob
{

	struct WallHugCommand : public ISimpleCommand
	{

		WallHugCommand(WallFollowSide side, IStopCondition::shared_ptr additionalCondition = NullCon) : 
			side(side),
			additionalCondition(additionalCondition)
		{}

		WallFollowSide side;

		IStopCondition::shared_ptr additionalCondition;

		virtual IController::shared_ptr generateController(const ISensorHandle& sensorHandle) const;

		virtual OredCondition::shared_ptr generateCondition(const ISensorHandle& sensorHandle, IStopCondition*& obstacleCondition, IStopCondition*& goalCondition) const;

	};

}

#endif
