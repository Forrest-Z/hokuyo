#ifndef _BOB_FREERTOS_IMPLEMENTATIONS_FREERTOS_BUMPER_HANDLE_H_
#define _BOB_FREERTOS_IMPLEMENTATIONS_FREERTOS_BUMPER_HANDLE_H_

#include <bob_sensor/ibumper_handle.h>

namespace bob
{

	// Currently a "dummy" implementation - bumpers disabled

	class FreeRTOSBumperHandle : public IBumperHandle
	{

		public:

			virtual BumperState getState() const 
			{
				return NoEvent;
			}

			virtual bool hitSomething() const
			{
				return false;
			}

			virtual void clearFlags()	
			{
			}
	
	};

}

#endif
