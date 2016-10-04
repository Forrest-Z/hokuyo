#include <bob_control/conditions/anded_condition.h>

namespace bob
{

	bool AndedCondition::condition(const ISensorHandle& sensorHandle)
	{
		bool result = true;
		for (std::vector<IStopCondition::shared_ptr >::iterator conditionItr = conditions.begin();
			conditionItr != conditions.end();
			++conditionItr)
		{
			if (!((*conditionItr)->isSatisfied(sensorHandle)))
			{
				// Don't return yet. Need to run resetAll()  
				// To clear any conditions that were set
				result = false;
			}			
		}

		if (!result)
			resetAll();

		return result;
	}

	void AndedCondition::resetAll()
	{
		for (std::vector<IStopCondition::shared_ptr >::iterator conditionItr = conditions.begin();
			conditionItr != conditions.end();
			++conditionItr)
		{
			(*conditionItr)->reset();
		} 
	}
}

