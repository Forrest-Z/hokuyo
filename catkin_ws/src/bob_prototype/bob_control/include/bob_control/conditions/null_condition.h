#ifndef _BOB_CONTROL_NULL_CONDITION_H_
#define _BOB_CONTROL_NULL_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>

namespace bob
{

	class NullCondition : public IStopCondition
	{
		virtual bool condition(const ISensorHandle& sensorHandle)
		{
			return false;
		} 

	};

	extern NullCondition::shared_ptr NullCon;

}

#endif
