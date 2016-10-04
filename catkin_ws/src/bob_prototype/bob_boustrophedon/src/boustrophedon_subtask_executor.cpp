#include <bob_boustrophedon/boustrophedon_subtask_executor.h>

#include <bob_boustrophedon/boustrophedon_curve_spaces.h>
#include <bob_boustrophedon/boustrophedon_condition.h>

#include <bob_toolbox/angles.h>
#include <bob_toolbox/logging.h>

#include <bob_control/conditions/ored_condition.h>
#include <bob_control/conditions/distance_world_direction_condition.h>
#include <bob_control/conditions/pointing_to_goal_condition.h>
#include <bob_control/conditions/half_space_condition.h>
#include <bob_control/conditions/anded_condition.h>
#include <bob_control/curved_subtask_executor.h>
#include <bob_control/wall_follow_side.h>


namespace bob
{
	BoustrophedonSubtaskExecutor::DriveCurveResult BoustrophedonSubtaskExecutor::driveAndCurve(Subtask nextAction, float failLength) 
	{
		// Cache some details about the action so we don't need to keep recalculating them
		cacheAction(nextAction);

		// Determine some basic properties of the task
		TaskProperties taskProperties = determineTaskProperties();

		// The spaces that will stop a wall following action
		BoustrophedonCurveSpaces wallFollowSpaces(action, failLength);

		// Initialize curved subtask executor
		CurvedSubtaskExecutor curvedSubtaskExecutor(controlHandle.simpleCommander, sensorHandle.getTransformHandle());
		ControlResult result;
		while((result = curvedSubtaskExecutor.execute(nextAction, failLength)) != ReachedGoal)
		{
			// Path executor failed because we ran into obstacle.
			// Now have to wall follow in order to get around that obstacle

			// Creates a condition from the spaces
			BoustrophedonCondition::shared_ptr condition(new BoustrophedonCondition(wallFollowSpaces, sensorHandle, travellingAngle));

			if (taskProperties.wallFollowSide == LeftSide)
			{
				LOG_BOUSTROPHEDON("Left side wall follow");
			}
			else
			{
				LOG_BOUSTROPHEDON("Right side wall follow");
			}

			// Choose which type of wall following to do, depending on the reason for interruption
			WallFollower::State wallFollowerState;
			if (result == ObstacleSafety)
			{
				wallFollowerState = WallFollower::SenseTracking;
			}
			else if (result == BumperHit)
			{
				wallFollowerState = WallFollower::BumperTracking;
			}

			WorldPoint robotPosition = sensorHandle.getTransformHandle().getLocalizedPose();
			MapHalfSpace inFrontOfRobot(robotPosition, travellingAngle);

			// Do the wall following (until condition reached)
			WallFollower follower(controlHandle.simpleCommander, sensorHandle, taskProperties.wallFollowSide, wallFollowerState);
			follower.run(condition);
			
			if (condition->continueAlongLine())
			{
				LOG_BOUSTROPHEDON("Returned along line");
				if (!inFrontOfRobot.contains(sensorHandle.getTransformHandle().getLocalizedPose()))
				{
					// Wall following ended up behind where robot started
					// Must return fail to stop infinite loops since the robot
					// will hit the obstacle and wall follow back to the current point again
					return WallFollowFailed;
				}
			}
			else if (condition->endOfRectangle())
			{
				LOG_BOUSTROPHEDON("End of rectangle");
				return ReachedEnd;
			}
			else if (condition->followedToNext())
			{
				LOG_BOUSTROPHEDON("Followed to next");
				return WallFollowedToNext;
			}
			else if (condition->backToStart())
			{
				LOG_BOUSTROPHEDON("Back to start");
				return WallFollowFailed;
			}
			else
			{
				LOG_ERROR("Invalid condition in Boustrophedon subtask executor");
			}
		}

		return ReachedEnd;
	}


	void BoustrophedonSubtaskExecutor::cacheAction(Subtask nextAction)
	{
		action = nextAction;
		travellingAngle = action.toTrack.getAngle();
		curveToAngle = action.toCurveTo.getAngle();
	}

	BoustrophedonSubtaskExecutor::TaskProperties BoustrophedonSubtaskExecutor::determineTaskProperties()
	{
		TaskProperties result;

		// Determine a bunch of properties based on the direction we are going to turn
		float angularDiff = normalizeAngle(curveToAngle - travellingAngle);	
		if (angularDiff > 0)
		{
			// Left-hand turn at end of action
			result.wallFollowSide = RightSide;
			result.finalCurveDirection = CounterClockwise;
		}
		else
		{
			// Right-hand turn at end of action
			result.wallFollowSide = LeftSide;
			result.finalCurveDirection = Clockwise;
		}
		return result;
	}

}

