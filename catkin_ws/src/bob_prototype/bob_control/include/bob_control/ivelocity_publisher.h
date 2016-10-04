#ifndef _BOB_CONTROL_IVELOCITY_PUBLISHER_H_
#define _BOB_CONTROL_IVELOCITY_PUBLISHER_H_

namespace bob
{

	class Velocity2D;

	//! \brief Interface representing a method to communicate velocity commands to the robot.
	class IVelocityPublisher
	{

		public:

			//! \brief Sends a velocity to the robot
			//! \param command The velocity to send
			virtual void publish(const Velocity2D& command) const = 0;

	};

}

#endif
