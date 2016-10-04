#include <bob_control/conditions/function_pointer_condition.h>

namespace bob
{

	bool FunctionPointerCondition::condition(const ISensorHandle& sensorHandle)
	{
		return fcnPtr(sensorHandle);
	}

}

