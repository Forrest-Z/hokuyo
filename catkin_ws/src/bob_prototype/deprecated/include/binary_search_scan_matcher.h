#ifndef SCANMATCHER_H
#define SCANMATCHER_H

#include <iostream>
#include <bob_toolbox/pose2d.h>
#include <bob_toolbox/eight_direction.h>
#include <bob_grid_map/concrete_map.h>
#include <bob_lidar/laser_reading.h>
#include <vector>

#include <bob_visualization/visualization.h>

#include <bob_lidar/lidar_properties.h>

namespace bob 
{

	//! THIS CLASS IS DEPRECATED
	//! The hector slam algorithm is better!
	class BinarySearchScanMatcher
	{
		public:

			Pose2D scanMatchPose(const ConcreteMap& map, const Pose2D& lidarPose, const LaserReading& readings, const std::vector<float>& laserAngles);

	};
}

#endif
