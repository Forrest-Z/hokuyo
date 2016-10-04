#ifndef _BOB_TOOLBOX_EXTERNALLY_LOCKED_H_
#define _BOB_TOOLBOX_EXTERNALLY_LOCKED_H_

#include <memory>
#include <boost/thread/mutex.hpp>
#include <assert.h>
#include <bob_toolbox/strict_lock.h>

namespace bob
{

	template <typename ResourceType>
	class ExternallyLocked
	{

		public:

			ExternallyLocked(ResourceType* data) : 
			lockedData(data)
			{}

			inline boost::mutex& getLock() const
			{
				return mutex;
			}
			
			inline ResourceType& getLockedResource(StrictLock& lock)
			{
				assert(lock.owns(mutex));
				return *lockedData;
			}

			inline const ResourceType& getLockedResource(StrictLock& lock) const
			{
				assert(lock.owns(mutex));
				return *lockedData;
			}

		protected:

			//! Protected because derived classes can add special behavior concerning this object
			std::auto_ptr<ResourceType> lockedData;	

		private:

			mutable boost::mutex mutex;

	};

}

#endif
