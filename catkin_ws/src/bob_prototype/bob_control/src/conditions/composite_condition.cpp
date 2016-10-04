#include <bob_control/conditions/composite_condition.h>

namespace bob
{

	void CompositeCondition::add(IStopCondition::shared_ptr condition)
	{
		conditions.push_back(condition);
	}
}

