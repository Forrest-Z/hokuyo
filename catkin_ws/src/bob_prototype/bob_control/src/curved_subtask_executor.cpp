#include <bob_control/curved_subtask_executor.h>

#include <bob_toolbox/geometry.h>

#include <bob_toolbox/angles.h>
#include <bob_visualization/visualization.h>
#include <bob_toolbox/easy_print.h>

#include <bob_control/curve_planner.h>
#include <bob_control/conditions/pointing_to_goal_condition.h>
#include <bob_control/conditions/distance_condition.h>
#include <bob_control/conditions/ored_condition.h>
#include <bob_control/conditions/half_space_condition.h>
#include <bob_control/commands/curve_command.h>
#include <bob_control/commands/heading_rotation_command.h>
#include <bob_control/simple_commands.h>

namespace bob
{

	ControlResult CurvedSubtaskExecutor::execute(Subtask task, float toCurveTolength, IStopCondition::shared_ptr additionalCondition)
	{
		Pose2D startingPose = transformHandle.getLocalizedPose();
		WorldPoint goal;
		task.toTrack.intersection(task.toCurveTo, goal);

		float distanceToGoal = diagonalDistance<WorldPoint>(startingPose, goal);
		
		// Distance at which we wont even try to drive to the goal because we are so close
		float headingDist = 0.03;

		// Calculate the curveRadius adjusted if the next line is too short
		float proximity = std::min(0.1, toCurveTolength / 2.0);

		if (distanceToGoal < headingDist || proximity < headingDist)
		{
			// Just rotate to next goal - we are too close to current goal
			return rotateAlignWithNextLine(task, additionalCondition);
			
		}
		else if (distanceToGoal <= proximity)
		{
			// Curve without driving straight because we are quite close to goal
			return curveOntoLine(task, additionalCondition);
		}
		else
		{
			// This is the normal operation: driving straight and eventually curving
			return driveAndCurveToPoint(task, proximity, additionalCondition);
		}	
	}

	ControlResult CurvedSubtaskExecutor::rotateAlignWithNextLine(Subtask task, IStopCondition::shared_ptr additionalCondition)
	{
		HeadingRotationCommand command(task.toCurveTo.getAngle(), additionalCondition);
		return simpleCommander.execute(command);
	}

	ControlResult CurvedSubtaskExecutor::driveAndCurveToPoint(Subtask task, float proximity, IStopCondition::shared_ptr additionalCondition)
	{
		WorldPoint goal;
		task.toTrack.intersection(task.toCurveTo, goal);
		Pose2D startingPose = transformHandle.getLocalizedPose();

		// Calculate this just once since it requires atan2 calculation
		float currentLineHeading = task.toTrack.getAngle();

		// The angular difference between current heading and current line
		float headingLineAngleDiff = fabs(normalizeAngle(startingPose.theta - currentLineHeading));

		if (headingLineAngleDiff > 0.2)
		{
			// We are facing away from the goal, so do a stationary rotation to fix that
			// This can happen on the first point, or due to errors
			HeadingRotationCommand command(currentLineHeading, additionalCondition);
			ControlResult headingRotationResult = simpleCommander.execute(command);
			if (headingRotationResult != ReachedGoal)
			{
				return headingRotationResult;
			}
		}

		// Obtain a point in front of the goal, to allow space for curving
		WorldPoint offsetGoal = goal - proximity * unitVector(currentLineHeading);

		// The turn is not too sharp, so move straight and then curve at end
		CurvePlanner curvePlanner(task.toCurveTo, Pose2D(offsetGoal, currentLineHeading));
		CurvePlanner::CurveSpecifics curveSpecifics = curvePlanner.getCurve();
		CurveCommand curveCommand(curveSpecifics, additionalCondition);
		
		if (curveCommand.radius < 0.03)
		{
			// The curve is too tight, so we will just default to stationary heading rotations instead of curves

			// Code for driving straight
			StraightDriver::Command straightCommand(task.toTrack, goal, 0.0, safetyDistance);

			ControlResult straightResult = straightDriver.execute(straightCommand, additionalCondition);
			if (straightResult != ReachedGoal)
			{
				return straightResult;	
			}

			// Execute the heading rotation
			HeadingRotationCommand headingRotationCommand(task.toCurveTo.getAngle(), additionalCondition);
			return simpleCommander.execute(headingRotationCommand);

		}
		else	
		{	
			// Note that the final velocity is equal to the curve velocity, for smooth movement
			StraightSettings settings(0.1, curveCommand.linearVelocity);

			// Move forward to the offset
			StraightDriver::Command straightCommand(task.toTrack, offsetGoal, curveCommand.linearVelocity, safetyDistance);

			ControlResult straightResult = straightDriver.execute(straightCommand, additionalCondition);
			if (straightResult != ReachedGoal)
			{
				return straightResult;	
			}

			// Curve in order to point to next
			return simpleCommander.execute(curveCommand);
		}
	}

	ControlResult CurvedSubtaskExecutor::curveOntoLine(Subtask task, IStopCondition::shared_ptr additionalCondition)
	{
		WorldPoint goal;
		task.toTrack.intersection(task.toCurveTo, goal);
		Pose2D startingPose = transformHandle.getLocalizedPose();

		float angleToPoint = rayAngle<WorldPoint>(startingPose, goal);

		float headingLineAngleDiff  = fabs(normalizeAngle(startingPose.theta - angleToPoint));

		if (headingLineAngleDiff > M_PI / 2)
		{
			// We are facing away from the goal, so do a stationary rotation to fix that
			HeadingRotationCommand command(angleToPoint, additionalCondition);
			ControlResult headingRotationResult = simpleCommander.execute(command);
			if (headingRotationResult != ReachedGoal)
			{
				return headingRotationResult;
			}
		}

		startingPose = transformHandle.getLocalizedPose();
		Line currentLine(startingPose, goal - startingPose);

		// We are close enough to curve onto the goal->next line without driving straight
		CurvePlanner curvePlanner(task.toCurveTo, startingPose);
		CurvePlanner::CurveSpecifics curveSpecifics = curvePlanner.getCurve();
		CurveCommand curveCommand(curveSpecifics, additionalCondition);

		if (curveCommand.radius < 0.03)
		{
			StraightDriver::Command straightCommand(currentLine, goal, 0.15, safetyDistance);

			ControlResult straightResult = straightDriver.execute(straightCommand, additionalCondition);
			if (straightResult != ReachedGoal)
			{
				return straightResult;	
			}

			HeadingRotationCommand command(task.toCurveTo.getAngle(), additionalCondition);
			return simpleCommander.execute(command);
		}
		else
		{
			return simpleCommander.execute(curveCommand);
		}
	}
}

