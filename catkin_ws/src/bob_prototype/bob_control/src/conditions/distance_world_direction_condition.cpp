#include <bob_control/conditions/distance_world_direction_condition.h>

#include <bob_toolbox/pose2d.h>

#include <bob_sensor/iproximity_sensor.h>
#include <bob_sensor/itransform_handle.h>

namespace bob
{

	bool DistanceWorldDirectionCondition::condition(const ISensorHandle& sensorHandle)
	{
		Pose2D pose = sensorHandle.getTransformHandle().getLocalizedPose();	
		float distance = sensorHandle.getProximitySensor().distanceFromRobot(angle - pose.theta);	
		bool result = (distance > minDistance);
		return result;	
	}

}

