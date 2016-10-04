#include <bob_map_algorithms/closest_clear_point.h>
#include <bob_config/config.h>

namespace bob
{
	WorldPoint ClosestClearPoint::getClosestClearPoint(const WorldPoint& basePoint)
	{
		float checkingRadius = 0.05;
		float maxClearDistance;
		WorldPoint clearPoint = basePoint;

		// Check points around the robot point, to find a most clear one (far from obstacles)
		// keep increasing the checking radius until find a clear one or checking radius larger than 0.3
		WorldPoint tempClearPoint = basePoint;

		do
		{
			maxClearDistance = -std::numeric_limits<float>::max();
			for(float angle = 0; angle < 2 * M_PI; angle += M_PI / 180)
			{
				WorldPoint worldPoint = basePoint + checkingRadius * unitVector(angle);	
				MapLocation location = costmap.worldToMap(worldPoint);
				if(costmap.getObstacleMap().pointFree(location) == Free && 
				costmap.getObstacleDistanceMap().obstacleDistance(location) > maxClearDistance)
				{
					maxClearDistance = costmap.getObstacleDistanceMap().obstacleDistance(location);
					tempClearPoint = worldPoint;
				}
			}
			checkingRadius += 0.02;
		}
		while(maxClearDistance < Config::ROBOT_RADIUS + 0.04 && checkingRadius < 0.25);

		
		if(maxClearDistance >= Config::ROBOT_RADIUS + 0.04)
		{
			clearPoint = tempClearPoint;
		}
		
		return clearPoint;
	}


	MapLocation ClosestClearPoint::getClosestClearPoint(const MapLocation& basePoint)
	{

		return costmap.worldToMap(getClosestClearPoint(costmap.mapToWorld(basePoint)));
	}
}

