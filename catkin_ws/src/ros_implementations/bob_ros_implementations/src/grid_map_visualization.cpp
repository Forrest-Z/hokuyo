#include <bob_ros_implementations/grid_map_visualization.h>

#include <bob_grid_map/iobstacle_map.h>
#include <bob_grid_map/iobstacle_distance_map.h>
#include <bob_grid_map/iprobability_map.h>

namespace bob
{

	unsigned char getROSMapValue(const IObstacleMap& obstacleMap, MapLocation point)
	{
		signed char value;	
		FreeState state = obstacleMap.pointFree(point);	
		switch (state)
		{
			case Unknown : 
				{
					value = 255;
					break;
				}
			case SensedObstacle : 
				{
					value = 100;
					break;
				}
			case HiddenObstacle :
				{
					value = 50;	
					break;
				}
			case Free : 
				{
					value = 0;
					break;
				}
		}
		return value;
	}

	unsigned char getROSMapValue(const IObstacleDistanceMap& obstacleDistanceMap, MapLocation point)
	{
		static int maxValue = 100;
		static int minValue = 0;
		static float scaling = ((float)maxValue / Config::OBSTACLE_INFLATION_DISTANCE);

		int value = (int)(maxValue - (obstacleDistanceMap.obstacleDistance(point) * scaling));
		value = std::max(minValue, std::min(maxValue, value));
		return (signed char)value;
	}

	unsigned char getROSMapValue(const IProbabilityMap& probabilityMap, MapLocation point)
	{
		float probability = probabilityMap.getProbability(point);
		if (probability == -1)
			return 255;
		else
			return (unsigned char)(probability * 100.0);
	}

}

