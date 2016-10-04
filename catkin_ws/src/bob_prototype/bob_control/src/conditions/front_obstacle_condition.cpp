#include <bob_control/conditions/front_obstacle_condition.h>

#include <bob_sensor/iproximity_sensor.h>

namespace bob
{

	bool FrontObstacleCondition::condition(const ISensorHandle& sensorHandle)
	{
		float frontDistance = sensorHandle.getProximitySensor().distanceFromRobot(0);
		return (frontDistance < minDistance);	
  
	}

}

