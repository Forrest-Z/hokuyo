#ifndef _BOB_WALL_FOLLOWING_WALL_HUGGER_H_
#define _BOB_WALL_FOLLOWING_WALL_HUGGER_H_

#include <bob_control/icontroller.h>

#include <bob_sensor/isensor_handle.h>
#include <bob_sensor/lidar_scan.h>
#include <bob_control/wall_follow_side.h>

#include <bob_toolbox/velocity2d.h>
#include <bob_toolbox/low_pass_filter.h>

#include <bob_config/config.h>

#include <exception>
#include <iostream>

namespace bob
{

	//! \brief Implements a controller that tries to drive beside a wall and maintain a fixed
	//! distance between itself and the wall. This controller is used for wall following,
	//! but it cannot function as one on its own. The reason for this is that the system uses
	//! PID control in order to hug the wall. This is fine if the situation is relatively linear,
	//! but nonlinearities like corners become a problem and must be handled separately.
	class WallHugger : public IController
	{

		public:

			//! \brief Constructor
			//! \param sideToFollow The side that the controller will follow
			WallHugger(WallFollowSide sideToFollow) : 
				lowPassFilter(0.2, 1/desiredFrequency()),
				side(sideToFollow)
			{
				// Variables set using config
				P = Config::WALL_FOLLOW_P;
				D = Config::WALL_FOLLOW_D;
				hugDistance = Config::WALL_FOLLOW_DISTANCE;
				speed = Config::WALL_FOLLOW_SPEED;
			}

			//! \brief Implemented virtual function
			virtual Velocity2D nextCommand(const ISensorHandle& sensorHandle);

		private:

			//! Struct packaging input signal to controller
			struct ControllerInput
			{

				//! Constructor
				ControllerInput() : 
					error(0), 
					errorDerivative(0) 
				{}

				//! The error in the system
				float error;

				//! The derivative of the error
				float errorDerivative;	
			};

			//! Calculates the input to the controller
			//! \param shortestBeam The shortest beam from the robot to the wall
			//! \param robotVelocity Current robot velocity
			ControllerInput calculateInput(const LidarBeam& shortestBeam, const Velocity2D& robotVelocity);

			//! Finds and returns the shortest lidar beam in the most recent available data
			//! \param sensorHandle Access to sensor data of robot
			LidarBeam shortestBeam(const ISensorHandle& sensorHandle);

			//! Low pass filter used to smooth robot movement
			LowPassFilter lowPassFilter;

			//! Intended distance to wall
			float hugDistance;

			//! Intended forward speed during wall following
			float speed;			

			//! Proportional component of PD controller
			float P;

			//! Derivative component of PD controller
			float D;

			//! Parameterization of wall follow action
			WallFollowSide side;

	};
}

#endif
