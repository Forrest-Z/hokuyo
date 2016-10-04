#ifndef _BOB_CONTROL_SIMPLE_COMMANDER_RESULT_H_
#define _BOB_CONTROL_SIMPLE_COMMANDER_RESULT_H_


namespace bob
{

	class Runnable;
	class IStopCondition;

	enum ControlResult 
	{
		ReachedGoal,
		ObstacleSafety,
		AdditionalCondition, 
		BumperHit
	};

	ControlResult checkStopCondition(Runnable& runnable);

	bool satisfiedAndNotNull(IStopCondition* condition);

}

#endif
