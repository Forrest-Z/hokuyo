#include <bob_control/conditions/pointing_to_goal_condition.h>

#include <bob_toolbox/geometry.h>
#include <bob_toolbox/angles.h>

#include <bob_toolbox/pose2d.h>

#include <bob_sensor/itransform_handle.h>

namespace bob
{
	bool PointingToGoalCondition::condition(const ISensorHandle& sensorHandle)
	{
		Pose2D pose = sensorHandle.getTransformHandle().getLocalizedPose();

		if (!initialized)
		{
			float initialHeading = pose.theta;
			float initialRobotGoalAngle = normalizeAngle(rayAngle<WorldPoint>(pose, goal));
			initialAngleDiff = normalizeAngle(initialRobotGoalAngle - pose.theta);
			initialized = true;
		}

		float robotGoalAngle = normalizeAngle(rayAngle<WorldPoint>(pose, goal));
		
		float angleDiff = normalizeAngle(robotGoalAngle - pose.theta);
		
		if(fabs(angleDiff) < 0.1 || initialAngleDiff * angleDiff < 0)
		{
			return true;
		}
		return false;
		
	}


}

