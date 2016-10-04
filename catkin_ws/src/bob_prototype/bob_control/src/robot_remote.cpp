#include <bob_control/robot_remote.h>

#include <bob_control/conditions/distance_condition.h>
#include <bob_control/commands/velocity_repeater_command.h>
#include <bob_control/commands/curve_command.h>
#include <bob_control/commands/heading_rotation_command.h>
#include <bob_control/commands/stop_command.h>
#include <bob_control/curve_tools.h>
#include <bob_control/simple_commander.h>

#include <bob_sensor/itransform_handle.h>
#include <bob_sensor/isensor_handle.h>
#include <bob_toolbox/angles.h>
#include <bob_toolbox/line.h>

namespace bob
{

	ControlResult RobotRemote::backUp(float distance, IStopCondition::shared_ptr condition) const 
	{
		float speed = -0.1;
		return executeStraightCommand(distance, speed, condition);
	}

	ControlResult RobotRemote::goForward(float distance, IStopCondition::shared_ptr condition) const
	{
		float speed = 0.1;
		return executeStraightCommand(distance, speed, condition);
	}

	ControlResult RobotRemote::curveOnToLine(Line toCurveTo, IStopCondition::shared_ptr condition) const
	{
		Pose2D startingPose = sensorHandle.getTransformHandle().getLocalizedPose();
		
		// Curve command
		CurvePlanner::CurveSpecifics curveSpecifics = adjustAndGetCurve(toCurveTo, startingPose, simpleCommander, sensorHandle.getTransformHandle());
		CurveCommand curveCommand(curveSpecifics, condition);
		
		// Stop command
		StopCommand stopCommand;
		
		ControlResult result = simpleCommander.execute(curveCommand);
		simpleCommander.execute(stopCommand);
		return result;
	}

	ControlResult RobotRemote::executeStraightCommand(float distance, float speed, IStopCondition::shared_ptr condition) const
	{
		DistanceCondition::shared_ptr distanceCondition(new DistanceCondition(sensorHandle, distance));
		OredCondition::shared_ptr oredCondition(new OredCondition());
		oredCondition->add(distanceCondition);
		oredCondition->add(condition);
		VelocityRepeaterCommand command(Velocity2D(speed, 0), oredCondition);
	
		ControlResult intermediateResult = simpleCommander.execute(command);

		if (distanceCondition->wasSatisfied())
		{
			return ReachedGoal;
		}
		else
		{
			return intermediateResult;
		}

		// Stop command
		/*
		StopCommand stopCommand;

		simpleCommander.execute(command);	
		simpleCommander.execute(stopCommand);
		*/
	}

	ControlResult RobotRemote::basicCurve(float radius, float speed, CircularDirection direction, IStopCondition::shared_ptr condition)
	{
		float angularVelocity = speed / radius;

		if (direction == Clockwise)
		{
			angularVelocity *= -1;
		}

		Velocity2D repeatedVelocity(speed, angularVelocity);

		VelocityRepeaterCommand command(repeatedVelocity, condition);
	
		return simpleCommander.execute(command);	
	}

	ControlResult RobotRemote::headingRotation(float offset, IStopCondition::shared_ptr condition)
	{
		Pose2D robotPose = sensorHandle.getTransformHandle().getLocalizedPose();
		float desiredHeading = robotPose.theta + offset;
		HeadingRotationCommand command(desiredHeading, condition);
		return simpleCommander.execute(command);
	}

}

