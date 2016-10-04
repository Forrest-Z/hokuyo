#include <bob_lidar/util_functions.h>

#include <limits>
#include <cmath>

namespace bob
{

	float distanceForBeam(float angle, float beamLength, float robotRadius)
	{
			// This is just pythagorean theorem: x^2 = r^2 - y^2    (x-axis lies along "angle" variable)
			// Keep it squared because that is slightly more efficient (we won't always calculate sqrt())
			float squaredRobotSliceWidth = pow(robotRadius, 2.0) - pow(beamLength * sin(angle), 2.0);

			float result = std::numeric_limits<float>::infinity();

			// Check to make sure that the beam is even inside the robot radius
			if (squaredRobotSliceWidth > 0)
			{
				// Not sure if fabs() is necessary, here
				float totalDistance = beamLength * cos(angle);

				// Find the actual beamLength between the object and the robot
				result = totalDistance - sqrt(squaredRobotSliceWidth);
			}
			
			return result;
	}

}


