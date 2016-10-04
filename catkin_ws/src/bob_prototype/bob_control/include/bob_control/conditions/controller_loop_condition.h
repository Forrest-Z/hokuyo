#ifndef _BOB_CONTROL_CONTROLLER_LOOP_CONDITION_H_
#define _BOB_CONTROL_CONTROLLER_LOOP_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>

namespace bob
{
	
	//! \brief This condition allows user to count the controller loop and 
	//! exit when certain loops have been executed.
	class ControllerLoopCondition : public IStopCondition
	{
		public: 

			explicit ControllerLoopCondition(size_t numOfLoopToExecute) : 
				numOfLoopToExecute(numOfLoopToExecute),
				count(0)
			{}
		
		private:	
	
			virtual bool condition(const ISensorHandle& sensorHandle);
			
			size_t numOfLoopToExecute;
			size_t count;

	};

}

#endif
