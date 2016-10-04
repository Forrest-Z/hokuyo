#include <bob_control/straight_path_executor.h>


#include <bob_toolbox/geometry.h>
#include <bob_toolbox/angles.h>

#include <bob_control/commands/isimple_command.h>
#include <bob_control/straight_driver.h>
#include <bob_control/commands/heading_rotation_command.h>

#include <bob_config/config.h>
namespace bob
{

	IPathExecutor::Result StraightPathExecutor::run(const std::vector<WorldPoint>& path, IStopCondition::shared_ptr additionalCondition)
	{
		Result result;
		StraightDriver straightDriver(transformHandle, simpleCommander);
		for (std::vector<WorldPoint>::const_iterator itr = path.begin(); itr != path.end(); ++itr)
		{
			Pose2D currentPose = transformHandle.getLocalizedPose();

			float distanceThreshold = 0.1;
		
			// Ignore points that are close to robot already
			if (diagonalDistance<WorldPoint>(currentPose, *itr) < distanceThreshold)
				continue;

			float headingToNextPoint = rayAngle<WorldPoint>(currentPose, *itr);

			float angleThreshold = 0.05;
		
			if (fabs(normalizeAngle(currentPose.theta - headingToNextPoint)) > angleThreshold)
			{
				HeadingRotationCommand rotationCommand(headingToNextPoint, additionalCondition);
				simpleCommander.execute(rotationCommand);
			}
		
			float lineAngle;
			if (itr != path.begin() && path.size() > 1)
			{
				lineAngle = rayAngle(*(itr - 1), *itr);
			}
			else
			{
				lineAngle = rayAngle<WorldPoint>(currentPose, *itr);
			}
			
			StraightDriver::Command driveCommand(Line(*itr, lineAngle), *itr, Config::FRONT_SAFETY_DISTANCE, 0.0);
				
			ControlResult simpleResult = straightDriver.execute(driveCommand, additionalCondition);	
			if (simpleResult != ReachedGoal)
			{
				result.lowLevelResult = simpleResult;
				return result;
			}
			else
			{
				result.lastPointReached = itr;
			}
		}
		return result;
	}

}

