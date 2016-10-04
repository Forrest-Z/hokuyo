#ifndef _BOB_TOOLBOX_INTERNALLY_LOCKED_H_
#define _BOB_TOOLBOX_INTERNALLY_LOCKED_H_

#include <boost/thread/mutex.hpp>

namespace bob
{

	//! \brief A class which stores data to be accessed via internal mutex locking.
	//!
	//! Note: there is another similar class called ExternallyLocked
	//!
	//! Internal locking is general easier to use for the client.
	//! However, external locking has advantages. External locking
	//! allows the user to perform multiple operations on the data
	//! before releasing the lock. 	
	//!
	//! External locking can also be more efficient because locking a
	//! mutex takes non-zero time. External locking can often reduce
	//! the number of times a mutex must be locked. 
	template <typename DataType>
		class InternallyLocked
		{

			public:

				//! \brief Constructor which specifies initial value of resource
				//! \param initial The initial value of the protected data
				InternallyLocked(DataType initial) :
					data(initial)
			{}

				//! \brief Constructor which uses default constructor of resource
				InternallyLocked() {}

				//! \brief Gets the data in a thread safe manner
				//! \return The mutex protected value
				DataType getValue() const
				{
					boost::mutex::scoped_lock(mutex);
					return data;
				}

				//! \brief Sets the data in a thread safe manner
				//! \param newValue The new value for the protected data
				void setValue(const DataType& newValue)
				{
					boost::mutex::scoped_lock(mutex);
					data = newValue;
				}

			private:

				//! Mutex-protected data
				DataType data;
		
				//! Mutex used to protect the data
				mutable boost::mutex mutex;

		};

}

#endif
