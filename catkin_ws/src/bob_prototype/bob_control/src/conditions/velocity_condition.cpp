#include <bob_control/conditions/velocity_condition.h>

#include <bob_sensor/iodometry_handle.h>

namespace bob
{
	bool VelocityCondition::condition(const ISensorHandle& sensorHandle)
	{
		Velocity2D robotVelocity = sensorHandle.getOdometryHandle().getRobotVel();
		if (fabs(targetVel.x - robotVelocity.x) < 0.01 && fabs(targetVel.w - robotVelocity.w) < 0.05)
		{
			return true;
		}	
	
		return false;
	}

}

