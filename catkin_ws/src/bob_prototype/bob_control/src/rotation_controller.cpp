#include <bob_control/rotation_controller.h>

#include <bob_config/config.h>

#include <bob_toolbox/velocity2d.h>
#include <bob_toolbox/pose2d.h>
#include <bob_toolbox/sign.h>
#include <bob_toolbox/newtonian_equations.h>
#include <bob_toolbox/geometry.h>
#include <bob_toolbox/angles.h>

#include <bob_sensor/isensor_handle.h>
#include <bob_sensor/itransform_handle.h>

namespace bob
{

	void RotationController::initAngularGoal(float angle)
	{
		initRotation(AngularGoal(angle));
	}

	/*
	void RotationController::initTimedRotation(float time)
	{
		initRotation(TimedGoal(time));	
	}
	*/

	void RotationController::initRotation(const RotationController::RotationGoal& rotationGoal)
	{
		lastRotationSent = 0.0;
		goal.reset(rotationGoal.clone());
	}

	Velocity2D RotationController::nextCommand(const ISensorHandle& sensorHandle)
	{
		Pose2D pose = sensorHandle.getTransformHandle().getLocalizedPose();

		// Delegate to abstraction
		Velocity2D command = goal->nextCommand(pose, lastRotationSent, this->desiredFrequency());
		
		// Give minimum output.
		command.w = sign(command.w) * std::max(fabs(command.w), 0.33);

		// Store last command sent
		lastRotationSent = command.w;

		// We will always find commands with this controller
		return command;
	}

	// This function is called when we are processing an angular goal
	Velocity2D RotationController::AngularGoal::nextCommand(Pose2D robotPose, float lastRotation, float frequency)
	{
		Velocity2D command;

		float difference = normalizeAngle(angle - robotPose.theta);

		float angularAcceleration = Config::ROTATION_ACCELERATION;

		// The speed that would get us to stop at constant decceleration
		float speedToStop = stoppingSpeedForDistance(angularAcceleration, std::fabs(difference));

		// The speed resulting from an accelerration
		float speedToAccelerate = std::fabs(lastRotation) + (angularAcceleration / frequency);	

		float maxVelocity = Config::ROTATION_MAX_VELOCITY;
		
		// Choosing the smallest of the max speeds to get our speed
		// Note: we multiply by sign(difference) because are speed calculations were absolute
		command.w = sign(difference) * std::min(std::min(fabs(speedToStop), fabs(speedToAccelerate)), fabs(maxVelocity));

		return command;
	}

	// This function is called when we are processing a timed goal
	/*
	Velocity2D RotationController::TimedGoal::nextCommand(Pose2D robotPose, float lastRotation, float frequency)
	{
		Velocity2D command;

		float timeLeft = (stopTime - now()).toSec();

		float angularAcceleration = Config::ROTATION_ACCELERATION;

		float acceleration = -angularAcceleration;

		// The max speed allowing us to stop on time 
		float speedToStop = timeLeft * acceleration;

		// The max speed that will maintain the acceleration allowances
		float speedToAccelerate = lastRotation + (acceleration / frequency);

		float maxVelocity = Config::ROTATION_MAX_VELOCITY;

		// Take the minimum of the max to get the new speed 
		command.w = sign(acceleration) * std::min(std::min(fabs(speedToStop), fabs(speedToAccelerate)), fabs(maxVelocity));

		return command;
	}
	*/

}
