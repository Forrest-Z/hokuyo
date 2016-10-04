#ifndef _BOB_CONTROL_ROTATION_CONTROLLER_H_
#define _BOB_CONTROL_ROTATION_CONTROLLER_H_

#include <bob_control/icontroller.h>

#include <bob_toolbox/velocity2d.h>
#include <bob_toolbox/pose2d.h>

#include <bob_config/config.h>

#include <boost/shared_ptr.hpp>

namespace bob
{

	class RotationController : public IController
	{

		public:

			typedef boost::shared_ptr<RotationController> shared_ptr;

			//! Given a robot pose, what velocity command to send to the robot
			virtual Velocity2D nextCommand(const ISensorHandle& sensorHandle);

			void initAngularGoal(float angle);
			//void initTimedRotation(float time);

			virtual float desiredController() const
			{
				return Config::ROTATION_CONTROLLER_FREQUENCY;
			}


		private:

			float lastRotationSent;


			//! Generic RotationGoal interface
			struct RotationGoal
			{
				//! Generate the next command based on the current state of the system
				virtual Velocity2D nextCommand(Pose2D robotPose, float lastRotation, float frequency)=0;	

				//! Create a polymorphic copy of the object
				virtual RotationGoal* clone() const =0;
			};	

			struct AngularGoal : public RotationGoal
		{
			AngularGoal(float angle) : angle(angle) {}

			//! The angle to rotate towards
			float angle;

			virtual Velocity2D nextCommand(Pose2D robotPose, float lastRotation, float frequency);	
			virtual RotationGoal* clone() const { return new AngularGoal(*this); }
		};

		/*
			struct TimedGoal : public RotationGoal
		{
			TimedGoal(float duration) : stopTime(now() + Duration(duration)), stopAcceleTime(now() + Duration(duration / 2))  {}

			//! The time when the controller come to a complete stop
			Time stopTime;
			Time stopAcceleTime;

			virtual Velocity2D nextCommand(Pose2D robotPose, float lastRotation, float frequency);	
			virtual RotationGoal* clone() const {return new TimedGoal(*this); } 
		};
		*/

			//! Stores the goal that will be used to polymorphically determine the behavior of the controller
			//! see: state design pattern
			std::auto_ptr<RotationGoal> goal;

			//! Initialize the controller for a rotation action
			void initRotation(const RotationGoal& goal);
	};

}

#endif
