#include <bob_control/conditions/proximity_condition.h>

#include <bob_toolbox/geometry.h>

#include <bob_sensor/itransform_handle.h>

namespace bob
{

	bool ProximityCondition::condition(const ISensorHandle& sensorHandle)
	{
		Pose2D pose = sensorHandle.getTransformHandle().getLocalizedPose();
		return (diagonalDistance<WorldPoint>(pose, point) < distance);
	}

	

	
}
