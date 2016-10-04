#include <bob_control/curved_path_executor.h>

#include <bob_toolbox/line.h>
#include <bob_toolbox/logging.h>

#include <bob_toolbox/subtask.h>
#include <bob_toolbox/geometry.h>

namespace bob
{

	IPathExecutor::Result CurvedPathExecutor::run(const std::vector<WorldPoint>& path, IStopCondition::shared_ptr additionalCondition)
	{
		Result result;
		for(std::vector<WorldPoint>::const_iterator itr = path.begin(); itr != path.end(); ++itr)
		{
			if ((itr + 1) == path.end())
			{
				// For laset point just move straight to the end, don't worry about final heading
				Pose2D pose = transformHandle.getLocalizedPose();

				StraightDriver::Command command(Line(pose, *itr - pose), *itr, 0.0, 0.0);
				result.lowLevelResult = straightDriver.execute(command, additionalCondition);	
				if (result.lowLevelResult == ReachedGoal)
				{
					result.lastPointReached = itr;
				}
				return result;
			}
			else
			{
				// Give the line to track and line to curve to as a subtask.
				Subtask task;

				if (itr == path.begin())
				{
					// For first point, use the line between robot current position 
					// and the first waypoint as the line to track.
					WorldPoint robotPosition = transformHandle.getLocalizedPose();
					task.toTrack = Line(robotPosition, *(itr) - robotPosition);
				}
				else
				{
					// The line to track is defined by current goal point and previoud goal point.
					task.toTrack = Line(*(itr - 1), *itr - *(itr - 1));
				}
				
				// The line to curve to in defined by current goal and new goal
				task.toCurveTo = Line(*itr, *(itr + 1) - *itr);
				float toCurveToLength = diagonalDistance(*itr, *(itr+1));
								
				ControlResult simpleResult = curvedSubtaskExecutor.execute(task, toCurveToLength, additionalCondition); 

				if(simpleResult != ReachedGoal)
				{
					LOG_CONTROL("subtask failed");
					result.lowLevelResult = simpleResult;
					return result;
				}
				else
				{
					result.lastPointReached = itr;
				}
			}
		}
	}


}

