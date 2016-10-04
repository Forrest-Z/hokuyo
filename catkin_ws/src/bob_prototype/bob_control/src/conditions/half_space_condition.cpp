#include <bob_control/conditions/half_space_condition.h>

#include <bob_toolbox/pose2d.h>
#include <bob_sensor/itransform_handle.h>

namespace bob
{

	bool HalfSpaceCondition::condition(const ISensorHandle& sensorHandle)
	{
		Pose2D pose = sensorHandle.getTransformHandle().getLocalizedPose();
		return space.contains(pose);
	}

}

