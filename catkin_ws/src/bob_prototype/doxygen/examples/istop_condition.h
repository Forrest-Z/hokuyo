#ifndef _BOB_CONTROL_ISTOP_CONDITION_H_
#define _BOB_CONTROL_ISTOP_CONDITION_H_

#include <bob_sensor/isensor_handle.h>
#include <boost/shared_ptr.hpp>

namespace bob
{

	//! \brief This is an abstract class that facilitates polling during controller operation.
	//! The condition is polled at the frequency of the controller (ie 10Hz).

	//! It is important that classes which implement this interface do not
	//! conduct processor-intensive tasks. Time consuming tasks will take longer than
	//! the period of the control loop. This will cause the control loop to extend longer
	//! than it's period, causing it to work incorrectly. If you have a time consuming
	//! task that you want to check for, wrap it in a separate thread, and then have
	//! your class check a mutex-guarded variable inside your implementation of 
	//! condition(...). The mutex-guarded check should be fast, even if your algorithm
	//! inside the thread is relatively slow.
	class IStopCondition
	{

		public:

			//! A shared pointer to a IStopCondition object
			typedef boost::shared_ptr<IStopCondition> shared_ptr;

			//! Basic constructor. Note that the condition always starts off not satisfied
			IStopCondition() : satisfied(false) {}

			//! This function is called in the control loop. It updates condition and returns
			//! the updated value. If the condition has been triggered, it will be detected.
			inline bool isSatisfied(const ControlState& robotState)	
			{
				if (this->condition(robotState))
					satisfied = true;

				return satisfied;
			}

			//! Check if the condition was satisfied in the control loop. This will not	
			//! update the condition, in case it has now turned back to false.
			inline bool wasSatisfied() const
			{
				return satisfied;
			}
			
			//! Resets the condition to false
			void reset()
			{
				satisfied = false;
			}

		private:

			//! This method is overloaded in derived classes to define the condition
			//! \param robotState Information about the state of the system, used for determining if the condition is true
			//! \returns True if the condition was satisfied, false otherwise
			virtual bool condition(const ISensorHandle& sensorHandle) = 0;
		
			bool satisfied;

	};

}

#endif
