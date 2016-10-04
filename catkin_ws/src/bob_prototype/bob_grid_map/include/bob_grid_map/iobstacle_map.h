#ifndef _BOB_GRID_MAP_IOBSTACLE_MAP_H_
#define _BOB_GRID_MAP_IOBSTACLE_MAP_H_

#include <bob_grid_map/free_state.h>
#include <bob_grid_map/map_location.h>
#include <bob_grid_map/location_mapper.h>
#include <bob_grid_map/idimensioned_map.h>

namespace bob
{

	//! \brief An interface for a map which contains obstacle data. This interface to the map is used
	//! by non-SLAM parts of the system. Clients can query points to determine whether or not they
	//! contain an obstacle.
	class IObstacleMap : public virtual LocationMapper, public virtual IDimensionedMap
	{

		public:

			//! \brief Provides information about whether the point is occupied, free or unknown
			//! \param point The point to query
			virtual FreeState pointFree(MapLocation point) const = 0;		

	};

}

#endif
