#include <bob_control/conditions/distance_condition.h>

#include <bob_toolbox/geometry.h>

#include <bob_sensor/itransform_handle.h>

namespace bob
{

	DistanceCondition::DistanceCondition(const ISensorHandle& sensorHandle, float distance, bool useLocalization) :
		useLocalization(useLocalization),
		startPoint(desiredRobotPose(sensorHandle)),
		distance(distance)
	{}

	bool DistanceCondition::condition(const ISensorHandle& sensorHandle)
	{
		Pose2D pose = desiredRobotPose(sensorHandle);

		if (distance - diagonalDistance<WorldPoint>(pose, startPoint) <= 0.01)
		{
			// The distance between robotPoint and startPoint larger than distance
			return true;
		}

		return false;
	}

	Pose2D DistanceCondition::desiredRobotPose(const ISensorHandle& sensorHandle)
	{
		if (useLocalization)
			return sensorHandle.getTransformHandle().getLocalizedPose();
		else	
			return sensorHandle.getTransformHandle().getOdomPose();

	}

}

