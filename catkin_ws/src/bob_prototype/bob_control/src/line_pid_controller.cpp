#include <bob_control/line_pid_controller.h>

#include <cmath>

#include <bob_toolbox/newtonian_equations.h>
#include <bob_toolbox/line_geometry.h>
#include <bob_toolbox/velocity2d.h>
#include <limits>
#include <bob_toolbox/geometry.h>

#include <bob_toolbox/angles.h>
#include <bob_toolbox/sign.h>

#include <bob_config/config.h>
#include <bob_sensor/iodometry_handle.h>
#include <bob_sensor/itransform_handle.h>

namespace bob
{

	LinePIDController::LinePIDController(IController::shared_ptr controller, Line line, bool moveForward) :
		controller(controller),
		line(line),
		movingForward(moveForward)
	{}


	Velocity2D LinePIDController::nextCommand(const ISensorHandle& sensorHandle)
	{
		Velocity2D robotVelocity = sensorHandle.getOdometryHandle().getRobotVel();
		Pose2D pose = sensorHandle.getTransformHandle().getLocalizedPose();

		currLinearVel = controller->nextCommand(sensorHandle).x;

		double angularVel = headingVelocity(pose, robotVelocity);
		angularVel = std::min(angularVel, 0.66);
		angularVel = std::max(angularVel, -0.66);		

		return Velocity2D(currLinearVel, angularVel);
	}

	float LinePIDController::headingVelocity(Pose2D pose, Velocity2D robotVelocity)
	{

		// Control algorithm which will adjust the robot back towards desired heading

		// The plant for the system is v/s^2 where v = forward velocity, s = jw
		// The controller used is a PD controller
		// The closed loop tf is H = (s*k2 + k1)/(s^2 + s*k2 + k1)
		// k1 & k2 are chosen by design, resulting in an implicit kp and kd calculated below
		// This controller could use tuning
		float neededHeading;
		if (movingForward)
		{
			neededHeading = line.getAngle();
		}
		else
		{
			neededHeading = line.getAngle() + M_PI;
			
		}

		float angularError = shortestAngularDistance(neededHeading, pose.theta);

		float driftRate = sin(angularError) * robotVelocity.x;			

		float robotDrift = perpendicularDistanceWithSign(line, pose);

		float kp = 70;
		float kd = 140;

		float controlAction = -robotVelocity.x * (kd * driftRate + kp * robotDrift);
//		LOG_CONTROL("controlAction: %f, d: %f, p: %f", controlAction, kd * driftRate, kp * robotDrift);

		return controlAction;
	}
}

