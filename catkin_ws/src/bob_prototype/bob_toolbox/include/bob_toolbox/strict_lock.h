#ifndef _BOB_TOOLBOX_STRICT_LOCK_H_
#define _BOB_TOOLBOX_STRICT_LOCK_H_

#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

namespace bob
{

	//! Wrapper for lock_guard
	//! Stores a reference to the mutex that it has locked
	class StrictLock
	{

		public:

			StrictLock(boost::mutex& mutex) :
				held(mutex),
				guard(mutex)
		{}

			inline bool owns(const boost::mutex& mutex)
			{
				return &held == &mutex;
			}

		private:

			boost::lock_guard<boost::mutex> guard;

			const boost::mutex& held;



	};

}

#endif
