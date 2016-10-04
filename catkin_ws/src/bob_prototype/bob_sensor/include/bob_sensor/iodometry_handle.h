#ifndef _BOB_SENSOR_IODOMETRY_HANDLE_H_
#define _BOB_SENSOR_IODOMETRY_HANDLE_H_

namespace bob
{

	class Velocity2D;

	//! \brief Provides handle to odometry information, giving information about current robot speed.
	class IOdometryHandle
	{

		public:

			//! \brief Get the current robot velocity
			//! \return The current velocity of the robot
			virtual Velocity2D getRobotVel() const = 0;
	
			virtual ~IOdometryHandle()	
			{}

	};

}

#endif
