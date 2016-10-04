#ifndef _BOB_CONTROL_STOP_COMMAND_H_
#define _BOB_CONTROL_STOP_COMMAND_H_

#include <bob_control/commands/isimple_command.h>
#include <bob_control/conditions/null_condition.h>
#include <bob_control/runnable.h>

namespace bob
{

	struct StopCommand : public ISimpleCommand
	{

		virtual IController::shared_ptr generateController(const ISensorHandle& sensorHandle) const;

		virtual OredCondition::shared_ptr generateCondition(const ISensorHandle& sensorHandle, IStopCondition*& obstacleCondition, IStopCondition*& goalCondition) const;

	};

}

#endif
