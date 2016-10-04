#ifndef _BOB_CONTROL_BUMPER_CONDITION_H_
#define _BOB_CONTROL_BUMPER_CONDITION_H_

#include <bob_control/conditions/bumper_condition.h>
#include <bob_control/conditions/istop_condition.h>
#include <bob_sensor/isensor_handle.h>

namespace bob
{

	class BumperCondition : public IStopCondition
	{

		public:

			BumperCondition()
			{}

		private:

			virtual bool condition(const ISensorHandle& sensorHandle);

	};

}

#endif
