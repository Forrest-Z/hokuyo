#ifndef _BOB_CONTROL_VELOCITY_REPEATER_COMMAND_H_
#define _BOB_CONTROL_VELOCITY_REPEATER_COMMAND_H_

#include <bob_sensor/isensor_handle.h>
#include <bob_control/commands/isimple_command.h>
#include <bob_control/conditions/null_condition.h>
#include <bob_toolbox/velocity2d.h>

namespace bob
{

	struct VelocityRepeaterCommand : public ISimpleCommand
	{

		explicit VelocityRepeaterCommand(Velocity2D velocity, IStopCondition::shared_ptr additionalCondition = NullCon) : 
			velocity(velocity),
			additionalCondition(additionalCondition)
		{}

		Velocity2D velocity;

		IStopCondition::shared_ptr additionalCondition;

		virtual IController::shared_ptr generateController(const ISensorHandle& sensorHandle) const;

		virtual OredCondition::shared_ptr generateCondition(const ISensorHandle& sensorHandle, IStopCondition*& obstacleCondition, IStopCondition*& goalCondition) const;

	};

}

#endif
