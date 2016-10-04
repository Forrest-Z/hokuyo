#include <bob_control/conditions/heading_condition.h>

#include <bob_toolbox/geometry.h>
#include <bob_toolbox/angles.h>
#include <bob_toolbox/sign.h>
#include <bob_toolbox/pose2d.h>

#include <bob_sensor/itransform_handle.h>

namespace bob
{

	bool HeadingCondition::condition(const ISensorHandle& sensorHandle)
	{
		Pose2D pose = sensorHandle.getTransformHandle().getLocalizedPose();
		
		float difference = shortestAngularDistance(desiredHeading, pose.theta);

		if (!initialized)
		{
			initialDifference = difference;
			initialized = true;			
		}

		//TODO: need a better way to detected reach goal angle
		if (fabs(difference) < tolerance )//|| sign(initialDifference) * difference < 0)
		{
			return true;
		}

		
		return false;

	}

}

