#ifndef _BOB_GRID_MAP_MAP_METADATA_H_
#define _BOB_GRID_MAP_MAP_METADATA_H_

#include <bob_toolbox/dimension2d.h>
#include <bob_grid_map/map_location.h>

namespace bob
{

	//! \brief Contains the basic information about map size and location in world.
	struct MapMetadata
	{

		//! Height and width of map
		Dimension2D<int> bounds;

		//! The bottom left corner of the map
		MapLocation bottomLeftCorner;

		//! \brief Calculate the upper right corner of the map
		//! \return The MapLocation representing the upper right corner of the map
		inline MapLocation upperRightCorner() const
		{
			return MapLocation(bottomLeftCorner.x + bounds.width - 1, 
					   bottomLeftCorner.y + bounds.height - 1);
		}

		//! Equality operator
		inline bool operator==(const MapMetadata& other) const
		{
			return	(bounds == other.bounds) &&
				(bottomLeftCorner == other.bottomLeftCorner);	
		}
	
		//! Inequality operator
		inline bool operator!=(const MapMetadata& other) const
		{
			return !operator==(other);
		}
	};

}

#endif
