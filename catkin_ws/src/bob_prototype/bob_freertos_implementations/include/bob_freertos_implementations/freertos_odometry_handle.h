#ifndef _BOB_FREERTOS_IMPLEMENTATIONS_FREERTOS_ODOMETRY_HANDLE_H_
#define _BOB_FREERTOS_IMPLEMENTATIONS_FREERTOS_ODOMETRY_HANDLE_H_

#include <bob_sensor/iodometry_handle.h>

#include <bob_toolbox/velocity2d.h>

namespace bob
{

	class FreeRTOSOdometryHandle : public IOdometryHandle
	{

		public:

			virtual Velocity2D getRobotVel() const
			{
				// Needs a valid implementation...
				return Velocity2D();
			}

	};

}

#endif
