#ifndef _BOB_FREERTOS_IMPLEMENTATIONS_FREERTOS_VELOCITY_PUBLISHER_H_
#define _BOB_FREERTOS_IMPLEMENTATIONS_FREERTOS_VELOCITY_PUBLISHER_H_

#include <bob_control/ivelocity_publisher.h>

#include <bob_toolbox/velocity2d.h>

namespace bob
{

	class FreeRTOSVelocityPublisher : public IVelocityPublisher
	{

		public:

			virtual void publish(const Velocity2D& command) const;

	};

}

#endif
