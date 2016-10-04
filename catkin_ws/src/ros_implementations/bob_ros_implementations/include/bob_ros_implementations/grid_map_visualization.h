#ifndef _BOB_GRID_MAP_VISUALIZATION_H_
#define _BOB_GRID_MAP_VISUALIZATION_H_

#include <vector>
#include <bob_grid_map/map_metadata.h>

namespace bob
{

	class IObstacleDistanceMap;
	class IObstacleMap;
	class IProbabilityMap;

	template <typename MapType>
	std::vector<signed char> ROSMapData(const MapType& map)
	{
		MapMetadata metadata = map.getMapMetadata();
		std::vector<signed char> toReturn(metadata.bounds.width * metadata.bounds.height);
		int i = 0;
		MapLocation upperRight(metadata.upperRightCorner());
		for (int y = metadata.bottomLeftCorner.y; y <= upperRight.y; ++y)
		{
			for (int x = metadata.bottomLeftCorner.x; x <= upperRight.x; ++x)
			{
				MapLocation point(x, y);
				toReturn[i] = getROSMapValue(map, point);
				++i;
			}
		}
		return toReturn;
	}

	unsigned char getROSMapValue(const IObstacleDistanceMap& obstacleDistanceMap, MapLocation point);

	unsigned char getROSMapValue(const IObstacleMap& obstacleMap, MapLocation point);

	unsigned char getROSMapValue(const IProbabilityMap& obstacleMap, MapLocation point);

}

#endif
