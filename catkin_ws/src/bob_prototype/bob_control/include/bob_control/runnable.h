#ifndef _BOB_CONTROL_RUNNABLE_H_
#define _BOB_CONTROL_RUNNABLE_H_

#include <bob_control/conditions/istop_condition.h>
#include <bob_control/icontroller.h>
#include <bob_control/conditions/ored_condition.h>

namespace bob
{

	struct Runnable
	{
		Runnable(IController::shared_ptr controller, OredCondition::shared_ptr condition) : 
		controller(controller),
		condition(condition),
		obstacleCondition(NULL),
		goalCondition(NULL),
		bumperCondition(NULL)
		{}	

		OredCondition::shared_ptr condition;
		IController::shared_ptr controller;

		IStopCondition* obstacleCondition;
		IStopCondition* goalCondition;
		IStopCondition* bumperCondition;

	};

}

#endif
