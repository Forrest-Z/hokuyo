#include <bob_control/conditions/bumper_condition.h>

#include <bob_sensor/ibumper_handle.h>

namespace bob
{

	bool BumperCondition::condition(const ISensorHandle& sensorHandle)
	{
		return sensorHandle.getBumperHandle().hitSomething();	
	}

}

