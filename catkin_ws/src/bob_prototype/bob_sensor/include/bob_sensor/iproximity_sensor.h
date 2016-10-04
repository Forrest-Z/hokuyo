#ifndef _BOB_SENSOR_IPROXIMITY_SENSOR_H_
#define _BOB_SENSOR_IPROXIMITY_SENSOR_H_

#include <bob_toolbox/circular_direction.h>

namespace bob
{

	class LidarBeam;
	class SimpleAngularRange;
	class AngularRange;

	//! \brief A handle to sensor data which provides information about the state of the environment surrounding the robot.
	//! This class may be implemented using a combination of sensors, including Lidar, Infrared, SONAR, etc.
	//! Many (if not all) of the methods in this interface are in the robot frame where 0 degrees refers to directly
	//! in front of the robot. You need to be aware of this when using the class, because you may need to adjust for the
	//! orientation of the robot.
	class IProximitySensor
	{

		public:

			//! \brief How far the robot may travel in a given direction before hitting any obstacle.
			//! \param angle The angle in robot frame representing the direction the robot intends to go
			//! \return The distance the robot can go in the given angle before it hits an obstacle
			virtual float distanceFromRobot(float angle) const = 0;

			//! \brief Find a beam pointing to the nearest obstacle
			//! \return A beam pointing to the nearest obstacle (in robot frame)
			virtual LidarBeam shortestBeam() const = 0;

			//! \brief Find a beam pointing to the nearest obstacle, while only considering angles within a specific range
			//! \param angularRange The range in which to search for nearby obstacles
			//! \return A beam pointing to the nearest obstacle in the range
			virtual LidarBeam shortestBeam(const SimpleAngularRange& angularRange) const = 0;

			//! \brief Finds the first obstacle in a range that is nearer than a given distance
			//! \param angularRange The range in which to search for obstacles
			//! \param direction The direction to go when searching for obstacles inside. This will also determine the starting point.
			//! \param minDist The minimum distance for the found obstacle
			//! \param[out] beam A beam to the obstacle that was found
			//! \return True if an obstacle was found, false if no obstacles were found that are close enough to the robot
			virtual bool firstBeamShorterThan(const SimpleAngularRange& angularRange, const CircularDirection& direction, float minDist, LidarBeam& beam) const = 0;

			//! \brief Obtain an angular range whereby the robot may rotate and travel without hitting obstacles.
			//! \param travelDistance The distance that the robot must be able to move while maintaining margin with
			//! obstacles.
			//! \param minMargin A direction is only considered safe if the robot can travel in that direction
			//! while maintaining this margin between itself and obstacles.
			//! \return The "safe range" in the robot co-ordintae frame
			virtual AngularRange getPassableRange(float travelDistance, float minMargin) const = 0;

	};

}

#endif
