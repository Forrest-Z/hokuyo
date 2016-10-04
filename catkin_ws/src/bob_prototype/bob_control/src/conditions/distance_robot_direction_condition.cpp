#include <bob_control/conditions/distance_robot_direction_condition.h>

#include <bob_sensor/iproximity_sensor.h>

namespace bob
{

	bool DistanceRobotDirectionCondition::condition(const ISensorHandle& sensorHandle)
	{
		float distance = sensorHandle.getProximitySensor().distanceFromRobot(angle);	
		return (distance > minDistance);		
	}

}

