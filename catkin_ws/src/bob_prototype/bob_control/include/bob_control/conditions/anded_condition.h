#ifndef _BOB_CONTROL_ANDED_CONDITION_H_
#define _BOB_CONTROL_ANDED_CONDITION_H_

#include <bob_control/conditions/composite_condition.h>
#include <bob_sensor/isensor_handle.h>

namespace bob
{

	class AndedCondition : public CompositeCondition 
	{

		public:

			virtual bool condition(const ISensorHandle& sensorHandle);
		
		private:

			void resetAll();

	};

}

#endif
