#include <bob_wall_following/wall_follow_back_condition.h>

#include <bob_toolbox/geometry.h>

#include <bob_toolbox/pose2d.h>

#include <bob_sensor/itransform_handle.h>

namespace bob
{
	bool WallFollowBackCondition::condition(const ISensorHandle& sensorHandle)
	{
		Pose2D pose = sensorHandle.getTransformHandle().getLocalizedPose();
		float distanceToStart = diagonalDistance<WorldPoint>(pose, startPosition);
		if(!moveOut)
		{
			if(distanceToStart >= moveOutRadius)
				moveOut = true;
		}
		else
		{
			if(distanceToStart < moveBackRadius)
				return true;	
		}

		return false;
	}


}

