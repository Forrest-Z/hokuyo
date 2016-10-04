#include <bob_control/conditions/ored_condition.h>

namespace bob
{

	bool OredCondition::condition(const ISensorHandle& sensorHandle)
	{
		bool result = false;
		for (std::vector<IStopCondition::shared_ptr >::iterator conditionItr = conditions.begin();
			conditionItr != conditions.end();
			++conditionItr)
		{
			if ((*conditionItr)->isSatisfied(sensorHandle))
			{
				// Don't return just yet. Need to run "isSatisfied" on all 
				// the conditions so that their states are updated
				result = true;
			}			
		}
		return result;
	}

}

