#ifndef _BOB_TOOLBOX_PERIODIC_THREAD_H_
#define _BOB_TOOLBOX_PERIODIC_THREAD_H_

namespace bob
{

	//! An interface representing objects that can be run using a PeriodicThreader object
	
	//! This class is an interface for objects which are run on a thread at a particular frequency.
	class PeriodicThread
	{

		public:

			//! This is thread functor. Must be implemented in derived classes.
			
			//! Note: This function should run to completion in a reasonable period of time. 
			//! If it runs forever the periodic nature of the design will not be upheld.
			virtual void operator()() = 0;

	};

}

#endif
