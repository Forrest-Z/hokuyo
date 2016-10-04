#ifndef _BOB_COVERAGE_BINARY_MAP_AREA_H_
#define _BOB_COVERAGE_BINARY_MAP_AREA_H_

#include <bob_grid_map/raw_map.h>
#include <bob_grid_map/map_location.h>

namespace bob
{

	class BinaryMapArea : public RawMap<bool>
	{

		public:

			BinaryMapArea(float resolution) : 
			LocationMapper(resolution)
			{}

			bool contains(MapLocation mapLocation) const;
		
			void insert(const MapLocation& toInsert);

			virtual bool defaultCellValue() const;

	};

}

#endif
