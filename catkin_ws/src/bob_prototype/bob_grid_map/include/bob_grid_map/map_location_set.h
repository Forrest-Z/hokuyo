#ifndef _BOB_GRID_MAP_MAP_LOCATION_SET_H_
#define _BOB_GRID_MAP_MAP_LOCATION_SET_H_

#include <bob_grid_map/map_location.h>
#include <unordered_set>

namespace bob
{

	class MapLocationHash
	{

		public:

			std::size_t operator()(const MapLocation& point) const;

	};

	typedef std::unordered_set<MapLocation, MapLocationHash> MapLocationSet;

}

#endif
