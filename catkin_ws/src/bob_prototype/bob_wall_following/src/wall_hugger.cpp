#include <bob_wall_following/wall_hugger.h>

#include <limits>

#include <bob_config/config.h>
#include <bob_toolbox/angles.h>
#include <bob_toolbox/sign.h>
#include <bob_sensor/iodometry_handle.h>
#include <bob_toolbox/angular_range.h>
#include <bob_sensor/iproximity_sensor.h>

namespace bob
{

	WallHugger::ControllerInput WallHugger::calculateInput(const LidarBeam& shortestBeam, const Velocity2D& robotVelocity)
	{
		float distanceToWall = shortestBeam.range - Config::ROBOT_RADIUS;
		
		distanceToWall = std::min(distanceToWall, 2 * hugDistance);	

		float headingOff;
		if (side == RightSide)	
			headingOff = normalizeAngle(shortestBeam.angle + M_PI/2);
		else
			headingOff = normalizeAngle(shortestBeam.angle - M_PI/2);


		// Calculate PSD input
		float errorDiff;
		if (side == RightSide)		
			errorDiff = robotVelocity.x * sin(-headingOff);
		else
			errorDiff = robotVelocity.x * sin(headingOff);

		ControllerInput result;
		result.errorDerivative = errorDiff;
		result.error = distanceToWall - hugDistance;

		return result;
	}

	Velocity2D WallHugger::nextCommand(const ISensorHandle& sensorHandle)
	{
		LidarBeam beam = shortestBeam(sensorHandle);
		Velocity2D robotVelocity = sensorHandle.getOdometryHandle().getRobotVel();
		ControllerInput input = calculateInput(beam, robotVelocity);	

		float controllerAction = P * input.error + D * input.errorDerivative;

		float speed = 0.1;
/*
		if (fabs(speed * controllerAction) > 0.8)
		{
			speed = 0.8 / fabs(controllerAction);
			if (speed >= 0.05)
			{
				controllerAction *= speed;	
			}
		}
		else
		{
			controllerAction *= speed;	
		}
*/

		if (side == RightSide)
			controllerAction *= -1;

		controllerAction = sign(controllerAction) * std::min(fabs(controllerAction), 1.2);
		controllerAction = lowPassFilter.filterDataPoint(controllerAction);

		return Velocity2D(speed, controllerAction);
	}

	LidarBeam WallHugger::shortestBeam(const ISensorHandle& sensorHandle)
	{
		float examineRange = M_PI / 2;
		float directionToExamine = (side == RightSide) ? (3 * M_PI / 2) : (M_PI / 2);
		SimpleAngularRange toExamine(directionToExamine - examineRange, directionToExamine + examineRange);
		return sensorHandle.getProximitySensor().shortestBeam(toExamine);
	}
}

