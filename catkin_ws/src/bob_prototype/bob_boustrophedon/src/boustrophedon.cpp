#include <bob_boustrophedon/boustrophedon.h>

#include <bob_control_handle/control_handle.h>
#include <bob_sensor/isensor_handle.h>
#include <bob_coverage/discrete_area.h>
#include <bob_toolbox/logging.h>

#include <bob_system/system_utilities.h>

namespace bob
{

	void Boustrophedon::coverArea(const DiscreteArea& area)
	{
		// Initialize the member variables used for boustrophedon coverage
		WorldPoint robotPosition = sensorHandle.getTransformHandle().getLocalizedPose();	
		taskParameters = BoustrophedonTaskParameters(area, robotPosition);

		// Actually executes the boustrophedon action
		execute();	
	}

	void Boustrophedon::execute()
	{
		Subtask action = taskParameters.firstSubtask;

		// Navigate to starting point
		if (controlHandle.navigationManager.navigationGoal(action.toTrack.origin) != NavigationManager::ReachedGoal)
		{
			LOG_ERROR("Unreachable boustrophedon start");
			return;
		}

		// Determines if we should skip the normal travelling because we wall-followed across to the next major line
		bool skipNormal = false;
	
		// Tracks the distance travelled along the normal axis, so we can stop when we've travelled the entire rectangle
		float normalDistanceTravelled = 0;

		BoustrophedonSubtaskExecutor::DriveCurveResult result;
		while (systemUtilities->ok())
		{

			// Go along the major axis
			LOG_BOUSTROPHEDON("going along major");
			result = curveExecutor.driveAndCurve(action, lineSpacing);
			if (result == BoustrophedonSubtaskExecutor::WallFollowFailed)
			{
				// If we return to start when going along the major length, we cannot proceed further.
				// We have reached an insurmountable wall, so just return
				return;
			}
			
			// If we have passed over to the next line, we skip the normal task because	
			// it is not needed
			skipNormal = (result == BoustrophedonSubtaskExecutor::WallFollowedToNext);

			// Condition to finish if we have travelled far enough
			if (normalDistanceTravelled > taskParameters.normalLength - Config::ROBOT_RADIUS)
				return;

			// Now committed to moving another lineSpacing distance
			normalDistanceTravelled += lineSpacing;

			// Update to normal line action 
			updateSubtask(action, lineSpacing, action.toTrack.getAngle() + M_PI);

			if (!skipNormal)
			{
				// Go along normal axis
				LOG_BOUSTROPHEDON("Going along normal");
				result = curveExecutor.driveAndCurve(action, taskParameters.majorLength);
				if (result == BoustrophedonSubtaskExecutor::WallFollowedToNext || 
				    result == BoustrophedonSubtaskExecutor::WallFollowFailed)
				{
					// If normal axis wall follows along to next normal, we cannot proceed further
					// We have reached an insurmountable wall, so just return
					return;
				}
			}
			
			// Reset the flag
			skipNormal = false;

			// Update to major line action 
			updateSubtask(action, taskParameters.majorLength, action.toTrack.getAngle());
		}
	}

	void Boustrophedon::updateSubtask(Subtask& toUpdate, float spacing, float newCurveToAngle)
	{
		Subtask newTask;

		// We are now tracking the line we've curved onto
		newTask.toTrack = toUpdate.toCurveTo;

		// Determine the origin of the next curve line
		WorldPoint curveToStart = newTask.toTrack.origin + spacing * unitVector(newTask.toTrack.getAngle());

		newTask.toCurveTo = Line(curveToStart, newCurveToAngle);			

		toUpdate = newTask;
	}

}

