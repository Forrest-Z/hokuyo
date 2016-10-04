#include <bob_wall_following/wall_follow_stuck_condition.h>

#include <bob_config/config.h>
#include <cmath>

#include <bob_toolbox/angular_range.h>
#include <bob_sensor/iproximity_sensor.h>

namespace bob
{

	bool WallFollowStuckCondition::condition(const ISensorHandle& sensorHandle)
	{
		float radius = Config::ROBOT_RADIUS;
		float minDist = 0.07;
		float searchRange = asin(radius / (radius + minDist)) + 0.1;
		float innerLimit = 0.5;

		float lowerBound;
		float upperBound;
		CircularDirection direction;
		if (side == RightSide)
		{
			lowerBound = searchRange;
			upperBound = -innerLimit;
			direction = Clockwise;
		}
		else
		{
			lowerBound = -searchRange;
			upperBound = innerLimit;
			direction = CounterClockwise;
		}
		SimpleAngularRange range(lowerBound, upperBound);
		return sensorHandle.getProximitySensor().firstBeamShorterThan(range, direction, minDist, beam);

	}

}

