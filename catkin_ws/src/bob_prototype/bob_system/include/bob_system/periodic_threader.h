#ifndef _BOB_TOOLBOX_PERIODIC_THREADER_H_
#define _BOB_TOOLBOX_PERIODIC_THREADER_H_

#include <boost/thread.hpp>

#include <bob_toolbox/periodic_thread.h>

#include <bob_system/irate_timer.h>

namespace bob
{

	//! A class that will periodically call a functor on a separate thread

	//! This class is used to create threads which run periodically between sleep
	//! cycles. The class is also designed to join the thread when the object goes
	//! out of scope, which prevents crashing due to null references.
	//!
	//! To use this class, create an object which inherits PeriodicThread.
	//! Then, create a new PeriodicThreader object and pass in your PeriodicThread
	//! and a desired period. The thread will run until your PeriodicThreader goes
	//! out of scope.
	//!
	//! Typically, a mutex protected member of the PeriodicThread will allow you
	//! to get some data from the PeriodicThread.
	class PeriodicThreader
	{

		public:

			//! Creates a new object, but doesn't start the thread 
			PeriodicThreader(PeriodicThread& threadable, float period);

			//! Starts the thread loop
			void start();

			//! Destructs the object and joins the thread, if it's currently running
			~PeriodicThreader();

		private:

			void threadLoop();

			bool running;

			PeriodicThread& threadable;

			boost::thread thread;

			RateTimer rateTimer;

	};

}

#endif
