#include <bob_slam/map_filler.h>

#include <bob_toolbox/raytrace.h>
#include <bob_toolbox/bounds2d.h>

namespace bob
{

	void MapFiller::updateMap(IProbabilityMap& map, const Pose2D& lidarPose, const LidarScan& readings, bool fillFree)
	{
		resizeMap(map, lidarPose, readings);
		registerScan(map, lidarPose, readings, fillFree);
	}

	void MapFiller::resizeMap(IProbabilityMap& map, const Pose2D& lidarPose, const LidarScan& readings)
	{
		Bounds2D<float> laserBounds(lidarPose);

		// Determine the size of the area 
		for (LidarScan::iterator readingItr = readings.begin(); readingItr != readings.end(); ++readingItr)
		{
			float distance = readingItr->range;		
			float angle = readingItr->angle;

			// Throw out invalid scans
			if (distance > maxRange || distance == 0.0 || isnan(distance)) 
				continue;

			// Limit maximum to maxURange 
			distance = std::min(distance, maxURange);

			// Calculate endpoint of beam
			WorldPoint endpoint = lidarPose + distance * unitVector(lidarPose.theta + angle);

			laserBounds.expandToCover(endpoint);
		}

		// pad by 10cm (arbitrary)
		laserBounds.pad(0.1);
		map.expandToIncludeBounds(laserBounds);

	}

	void MapFiller::registerScan(IProbabilityMap& map, const Pose2D& lidarPose, const LidarScan& readings, bool fillFree)
	{
		IntPoint originLocation = map.worldToMap(lidarPose);

		for (LidarScan::iterator readingItr = readings.begin(); readingItr != readings.end(); ++readingItr)
		{
			float distance = readingItr->range;
			float angle = readingItr->angle;

			if (distance > maxRange || distance == 0.0 || isnan(distance))
				continue;

			// Fill beam endpoint
			WorldPoint endpoint = lidarPose + distance * unitVector(lidarPose.theta + angle);
			IntPoint endpointLocation = map.worldToMap(endpoint);
			map.setObstacle(endpointLocation);

			if (fillFree)
			{
				std::vector<IntPoint> line = raytraceLine(originLocation, endpointLocation);
				for (int i = 0; i < line.size() - 1; i ++)
				{
					map.setFree(line[i]);
				}
			} 

		}
	}
}

