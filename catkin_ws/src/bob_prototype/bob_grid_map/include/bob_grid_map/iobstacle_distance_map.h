#ifndef _BOB_GRID_MAP_IOBSTACLE_DISTANCE_MAP_H_
#define _BOB_GRID_MAP_IOBSTACLE_DISTANCE_MAP_H_

#include <bob_grid_map/map_location.h>
#include <bob_grid_map/location_mapper.h>
#include <bob_grid_map/idimensioned_map.h>

namespace bob
{

	class IObstacleMap;

	//! \brief Represents a map of cached distance values. These distance values represent the distance of each point
	//! to the nearest obstacle. The map is produced from 
	class IObstacleDistanceMap : public virtual LocationMapper, public virtual IDimensionedMap
	{

		public:

			//! \brief Generates the cached distance data
			//! \param map The map from which to generate the obstacle distances
			virtual void setInflationData(const IObstacleMap& data) = 0;

			//! \brief Gets an estimate of the distance to the nearest obstacle
			//! It is an estimate due to the discrete nature of the map.
			//! \param location The location in the map to query
			//! \return The distance to nearest obstacle
			virtual float obstacleDistance(MapLocation location) const = 0;

	};

}

#endif
