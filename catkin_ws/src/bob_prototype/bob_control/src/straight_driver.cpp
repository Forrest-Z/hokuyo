#include <bob_control/straight_driver.h>


#include <bob_toolbox/geometry.h>
#include <bob_toolbox/map_half_space.h>
#include <bob_toolbox/pose2d.h>

#include <bob_control/commands/isimple_command.h>
#include <bob_control/commands/velocity_repeater_command.h>
#include <bob_control/simple_commands.h>
#include <bob_control/conditions/ored_condition.h>
#include <bob_control/conditions/half_space_condition.h>
#include <bob_control/conditions/front_obstacle_condition.h>
#include <bob_control/conditions/controller_loop_condition.h>

#include <bob_control/line_pid_controller.h>

namespace bob
{

	ControlResult StraightDriver::execute(Command command, IStopCondition::shared_ptr additionalCondition)
	{
		float acceleration = 0.15;
		float linearSpeed = 0.15;

		float accelDistance = 0.05;//(pow(linearSpeed, 2)) / (2 * acceleration);
	
		// Packing the command
		StraightSettings settings;
		settings.safetyDistance = command.safetyDistance;
		settings.finalSpeed = command.finalSpeed;
	
		Pose2D startingPose = transformHandle.getLocalizedPose();
		
		float pathLength = diagonalDistance<WorldPoint>(startingPose, command.goal);
		float pathAngle = command.toTrack.getAngle();
		
		if (pathLength <= 2 * accelDistance)
		{
			accelDistance = pathLength / 2;
			linearSpeed = sqrt(2 * acceleration * accelDistance);
		}
		
		AccelerateCommand accelCommand(command.toTrack, accelDistance, linearSpeed, additionalCondition);
		ControlResult speedupResult = simpleCommander.execute(accelCommand);

		if (speedupResult != ReachedGoal)
		{
			return speedupResult;
		}

		// Constant Linear velocity
		WorldPoint subGoal = command.goal - accelDistance * unitVector(pathAngle);

		TrackLineCommand trackLineCommand(command.toTrack, subGoal, linearSpeed, command.safetyDistance, additionalCondition);
		ControlResult constVelResult = simpleCommander.execute(trackLineCommand);

		if (constVelResult != ReachedGoal)
		{
			return constVelResult;
		}

		// Decelerating
		// Check if current position has already passed the goal
		MapHalfSpace goalSpace(command.goal, pathAngle);
		if (goalSpace.contains(transformHandle.getLocalizedPose()))
		{
			return ReachedGoal;	
		}
		
		float distanceToGoal = diagonalDistance<WorldPoint>(transformHandle.getLocalizedPose(), command.goal);

		AccelerateCommand decelCommand(command.toTrack, distanceToGoal, command.finalSpeed, additionalCondition);
		ControlResult slowdownResult = simpleCommander.execute(decelCommand);
		
		/*
		if (slowdownResult == ObstacleSafety)	
		{
			Pose2D currPose = transformHandle.getLocalizedPose();
			AccelerateCommand gentleStopCommand(command.toTrack, 0.03, 0.0);
			simpleCommander.execute(gentleStopCommand);
		}
		*/		

		// Run final velocity once
		ControllerLoopCondition::shared_ptr runOnceCondition(new ControllerLoopCondition(1));
		VelocityRepeaterCommand velocityRepeater(Velocity2D(command.finalSpeed, 0), runOnceCondition);
		
		simpleCommander.execute(velocityRepeater);		

		return slowdownResult;	
	}

}

