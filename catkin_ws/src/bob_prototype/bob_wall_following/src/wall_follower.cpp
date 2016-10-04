#include <bob_wall_following/wall_follower.h>

#include <bob_wall_following/wall_follow_stuck_condition.h>
#include <bob_wall_following/wall_hug_command.h>
#include <bob_control/conditions/ored_condition.h>
#include <bob_wall_following/wall_follow_heading_chooser.h>
#include <bob_control/commands/heading_rotation_command.h>
#include <bob_control/robot_remote.h>

#include <bob_toolbox/pose2d.h>

#include <bob_sensor/lidar_scan.h>
#include <bob_sensor/iproximity_sensor.h>
#include <bob_sensor/itransform_handle.h>
#include <bob_system/system_utilities.h>

#include <bob_config/config.h>

namespace bob
{

	bool ReturnToSensed::condition(const ISensorHandle& sensorHandle)
	{
		LidarBeam shortestBeam = sensorHandle.getProximitySensor().shortestBeam();	

		float normalizedAngle = normalizeAngle(shortestBeam.angle);
		static const float upperThreshold = (M_PI / 2) - 0.3;
		static const float lowerThreshold = M_PI / 4;

		if (shortestBeam.range > (Config::ROBOT_RADIUS + 0.1))
			return false;

		bool beamFound = false;
		if (side == LeftSide)
		{
			beamFound = (normalizedAngle < upperThreshold) && (normalizedAngle > -lowerThreshold);
		}
		else if (side == RightSide)
		{
			beamFound = (normalizedAngle > -upperThreshold) && (normalizedAngle < lowerThreshold);
		}
	
		return beamFound;
	}

	void WallFollower::run(IStopCondition::shared_ptr stopCondition)
	{
		bool done = false;
		while (!done)
		{
			if (currentState == SenseTracking)
				done = sensedAlgorithm(stopCondition);
			else if (currentState == BumperTracking)
				done = bumperAlgorithm(stopCondition);
		}
	}

	bool WallFollower::sensedAlgorithm(IStopCondition::shared_ptr stopCondition)
	{
		// Pre-align to the wall
		LidarBeam shortestBeam = sensorHandle.getProximitySensor().shortestBeam();	

		Pose2D pose = sensorHandle.getTransformHandle().getLocalizedPose();
		float requiredHeading = getRequiredHeading(pose.theta, shortestBeam.angle);
		HeadingRotationCommand command(requiredHeading);
		commander.execute(command);

		// Start wall following
		while (systemUtilities->ok())
		{
			OredCondition::shared_ptr huggingCondition(new OredCondition());

			WallFollowStuckCondition::shared_ptr stuckCondition(new WallFollowStuckCondition(side));
			huggingCondition->add(stuckCondition);

			huggingCondition->add(stopCondition);

			WallHugCommand command(side, huggingCondition);
			ControlResult result = commander.execute(command);

			if (stopCondition->wasSatisfied())
				return true;

			if (result == BumperHit)
			{
				currentState = BumperTracking;
				return false;
			}

			pose = sensorHandle.getTransformHandle().getLocalizedPose();
			WallFollowHeadingChooser wallFollowHeadingChooser(sensorHandle, side); 
			float suggestedHeading = wallFollowHeadingChooser.getHeadingInRobotFrame();

			requiredHeading = pose.theta + suggestedHeading;
			HeadingRotationCommand rotCommand(requiredHeading);
			commander.execute(rotCommand);
		}
	}

	bool WallFollower::bumperAlgorithm(IStopCondition::shared_ptr stopCondition)
	{
		RobotRemote robotRemote(commander, sensorHandle);

		CircularDirection curveDirection;
		OredCondition::shared_ptr condition(new OredCondition());

		ReturnToSensed::shared_ptr returnCondition(new ReturnToSensed(side));
		condition->add(returnCondition);
		condition->add(stopCondition);

		float angleAugment = Config::BLIND_WALL_FOLLOW_ROTATION_ANGLE;
		if (side == RightSide)
		{
			curveDirection = Clockwise;
		}
		else 
		{
			curveDirection = CounterClockwise;
			angleAugment *= -1;
		}
	
		while (systemUtilities->ok())
		{
			// Simple commander backs up for us already

			ControlResult result = robotRemote.headingRotation(angleAugment, condition);

			if (result == BumperHit)
			{
				continue;
			}
			else if (result == AdditionalCondition)
			{
				break;
			}

			result = robotRemote.goForward(Config::BLIND_WALL_FOLLOW_FORWARD_DISTANCE, condition);

			if (result == BumperHit)
			{
				continue;
			}
			else if (result == AdditionalCondition)
			{
				break;
			}

			robotRemote.basicCurve(Config::BLIND_WALL_FOLLOW_CURVE_RADIUS, 0.07, curveDirection, condition);

			if (result == AdditionalCondition)
				break;
		}
		
		if (returnCondition->wasSatisfied())
			currentState = SenseTracking;	

		return (stopCondition->wasSatisfied());
	}

	float WallFollower::getRequiredHeading(float robotHeading, float closestBeamAngle) const
	{
		float angleToMoveBeam;
		if (side == RightSide)
			angleToMoveBeam = M_PI / 2;
		else
			angleToMoveBeam = -M_PI / 2;

		float requiredHeading = normalizeAngle(robotHeading + (closestBeamAngle + angleToMoveBeam));

		return requiredHeading;
	}
}

