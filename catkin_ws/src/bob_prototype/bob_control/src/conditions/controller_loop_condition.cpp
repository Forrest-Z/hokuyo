#include <bob_control/conditions/controller_loop_condition.h>

namespace bob
{
	bool ControllerLoopCondition::condition(const ISensorHandle& sensorHandle)
	{
		if(count++ == numOfLoopToExecute)
			return true;
		
		return false;	
	}

}

