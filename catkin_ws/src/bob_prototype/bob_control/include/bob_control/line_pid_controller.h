#ifndef _BOB_CONTROL_LINE_PID_CONTROLLER_H_
#define _BOB_CONTROL_LINE_PID_CONTROLLER_H_

#include <bob_toolbox/velocity2d.h>
#include <bob_toolbox/pose2d.h>
#include <bob_toolbox/world_point.h>
#include <bob_toolbox/line.h>

#include <bob_sensor/isensor_handle.h>
#include <bob_control/icontroller.h>

#include <boost/shared_ptr.hpp>

namespace bob
{

	class LinePIDController: public IController
	{

		public:

			LinePIDController(boost::shared_ptr<IController> controller, Line line, bool moveForward); 
			
			virtual Velocity2D nextCommand(const ISensorHandle& sensorHandle);

			inline virtual float desiredFrequency()
			{
				return controller->desiredFrequency();
			}

		private: 

			float headingVelocity(Pose2D robotPose, Velocity2D robotVelocity);
		
			boost::shared_ptr<IController> controller;

			//! These variables define the line along which the robot will travel
			Line line;

			//! If true, the robot will be moving in it's +ve x direction, -ve x direction otherwise
			//! This is calculated based on the given goal
			bool movingForward;
			
			float currLinearVel;

	};

}

#endif
