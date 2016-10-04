#ifndef _BOB_CONTROL_FUNCTION_POINTER_CONDITION_H_
#define _BOB_CONTROL_FUNCTION_POINTER_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>

namespace bob
{

	class FunctionPointerCondition : public IStopCondition
	{

		public:

			FunctionPointerCondition(bool (*fcnPtr)(const ISensorHandle&)) :
				fcnPtr(fcnPtr)
		{}

			virtual bool condition(const ISensorHandle& sensorHandle);


		private:

			bool (*fcnPtr)(const ISensorHandle&);
	};

}

#endif
