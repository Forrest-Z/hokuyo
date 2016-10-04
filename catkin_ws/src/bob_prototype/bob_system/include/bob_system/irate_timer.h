#ifndef _BOB_SYSTEM_IRATE_TIMER_H_
#define _BOB_SYSTEM_IRATE_TIMER_H_

#include <memory>

namespace bob
{

	//! \brief A timer used to schedule events at a particular periodic rate.
	//! The timer is meant to be used in a loop. At the beginning of the loop, call startTimer().
	//! When done executing some code, call sleepRemaining(), which will block until the remaining
	//! time is exhausted (if any is left). This ensures that the code executes at a set period,
	//! so long as the execution time of the code does not grow larger than 1 / frequency.
	//! In that case, the timer has no effect.
	class IRateTimer
	{

		public:

			//! \brief Start the timer
			virtual void startTimer() = 0;

			//! \brief Block to main the desired frequency
			virtual void sleepRemaining() = 0;

			//! \brief Virtual destructor allows deletion via pointer to base class
			virtual ~IRateTimer()
			{}

	};

	//! Convienience typedef for smart pointer to IRateTimer
	typedef std::unique_ptr<IRateTimer> RateTimer;

}

#endif
