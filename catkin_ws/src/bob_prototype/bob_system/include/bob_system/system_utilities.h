#ifndef _BOB_SYSTEM_ISYSTEM_H_
#define _BOB_SYSTEM_ISYSTEM_H_

#include <memory>

#include <bob_system/irate_timer.h>
#include <bob_system/itimer.h>

namespace bob
{

	//! \brief Provides core system functionality. This is an abstraction which was designed
	//! to make the code portable. To port the code to a new platform, create a derived
	//! class that implements the virtual methods in the class, and assign systemUtilities
	//! with an instance of that class. The default implementations of these functions won't
	//! do anything except output some logging explaining that the system wasn't correctly
	//! configured.
	class SystemUtilities
	{

		public:

			//! \brief Check if the system is running correctly
			//! \return True if the system is still running
			virtual bool ok() const;

			//! \brief Block execution for a specified length of time
			//! \param seconds The length of time to block execution, in seconds 
			virtual void sleep(float seconds);

			//! \brief Obtain a system-specific RateTimer implementation
			//! \return A RateTimer implementation specific to the platform
			virtual RateTimer rateTimer(float frequency);

			//! \brief Obtain a system-specific Timer implementation
			//! \return A Timer implementation specific to the platform
			virtual Timer timer();

	};

	//! Provides global access to core system functionality
	extern std::unique_ptr<SystemUtilities> systemUtilities;

}

#endif
