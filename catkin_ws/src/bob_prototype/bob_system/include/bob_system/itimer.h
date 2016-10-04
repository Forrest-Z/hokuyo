#ifndef _BOB_SYSTEM_ITIMER_H_
#define _BOB_SYSTEM_ITIMER_H_

#include <memory>

namespace bob
{

	//! \brief Used to time the length of events in real-world time.
	class ITimer
	{

		public:

			//! \brief Starting the timer to begin timing an event.
			virtual void startTimer() = 0;

			//! \brief Obtain the time elapsed, in seconds, since starting the timer.
			//! \return The length of time that has passed since calling startTimer(), in seconds
			virtual float timeElapsed() = 0;

			virtual ~ITimer()
			{}

	};

	//! Convienience typedef for ITimer
	typedef std::unique_ptr<ITimer> Timer;

}

#endif
