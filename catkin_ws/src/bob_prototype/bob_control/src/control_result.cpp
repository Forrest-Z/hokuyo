#include <bob_control/control_result.h>

#include <bob_control/runnable.h>

#include <bob_control/conditions/istop_condition.h>

namespace bob
{

	ControlResult checkStopCondition(Runnable& runnable)
	{
		if (satisfiedAndNotNull(runnable.goalCondition))
		{
			return ControlResult::ReachedGoal;
		}
		else if (satisfiedAndNotNull(runnable.obstacleCondition))
		{
			return ControlResult::ObstacleSafety;
		}
		else if (satisfiedAndNotNull(runnable.bumperCondition))
		{
			return ControlResult::BumperHit;
		}
		else
		{
			return ControlResult::AdditionalCondition;
		}
	}

	bool satisfiedAndNotNull(IStopCondition* condition)
	{
		return (condition != NULL) && (condition->wasSatisfied());
	}
}

