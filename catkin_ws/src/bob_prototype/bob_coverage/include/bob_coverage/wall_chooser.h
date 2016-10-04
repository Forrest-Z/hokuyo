#ifndef _BOB_COVERAGE_WALL_CHOOSER_H_
#define _BOB_COVERAGE_WALL_CHOOSER_H_

#include <bob_grid_map/map_location_set.h>

#include <bob_toolbox/world_point.h>
#include <bob_grid_map/map_location.h>

#include <vector>

namespace bob
{

	class LockableMap;
	class ITransformHandle;
	
	//! Locate the walls (obstacles). for each wall, choose a closest point to start wall follow. Iterate through all the wall sets.
	class WallChooser
	{

		public:

			WallChooser(const LockableMap& map, const ITransformHandle& transformHandle);
	
			bool done() const;		
	
			WorldPoint nextWallPoint();

		private:

			MapLocation closestWallPoint(const MapLocationSet& set) const;

			const ITransformHandle& transformHandle;

			float resolution;

			std::vector<MapLocationSet> obstacles;
			
	};

}

#endif
