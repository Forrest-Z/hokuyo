#ifndef _BOB_CONTROL_TRACK_LINE_COMMAND_H_
#define _BOB_CONTROL_TRACK_LINE_COMMAND_H_

#include <bob_control/commands/isimple_command.h>
#include <bob_control/conditions/istop_condition.h>
#include <bob_control/conditions/null_condition.h>
#include <bob_control/runnable.h>

#include <bob_toolbox/line.h>

namespace bob
{

	struct TrackLineCommand : public ISimpleCommand
	{
		TrackLineCommand(Line line, WorldPoint goal, float speed, float safetyDistance, IStopCondition::shared_ptr additionalCondition = NullCon) :
			line(line),
			goal(goal),
			speed(speed),	
			safetyDistance(safetyDistance),
			additionalCondition(additionalCondition)
		{}

		virtual IController::shared_ptr generateController(const ISensorHandle& sensorHandle) const;

		virtual OredCondition::shared_ptr generateCondition(const ISensorHandle& sensorHandle, IStopCondition*& obstacleCondition, IStopCondition*& goalCondition) const;

		Line line;
		WorldPoint goal;
		float speed;
		float safetyDistance;
		IStopCondition::shared_ptr additionalCondition;
	};

}

#endif
