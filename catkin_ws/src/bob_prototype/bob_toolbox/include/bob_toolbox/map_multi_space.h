#ifndef _BOB_TOOLBOX_MAP_MULTI_SPACE_H_
#define _BOB_TOOLBOX_MAP_MULTI_SPACE_H_

#include <bob_toolbox/map_half_space.h>
#include <vector>

namespace bob
{

	//! Defines a space in the map using a vector of MapHalfSpaces
	//! These half spaces may either be OR'd or AND'd together to create
	//! this composite space.
	class MapMultiSpace
	{

		public:

			//! The type of multispace.
	
			//! Determines whether all contents are AND'd or OR'd
			//! to determine if a point is contained
			enum Type { AND, OR, EMPTY };

			//! Empty constructor to create empty multi space
			MapMultiSpace() : 
				spaceType(EMPTY)
		{};

			MapMultiSpace(std::vector<MapHalfSpace> halfSpaces, Type type) :
				spaces(halfSpaces),
				spaceType(type)
		{};

			//! Test if a point is contained in the multispace
			bool contains(WorldPoint point) const;

		private:

			std::vector<MapHalfSpace> spaces;
			Type spaceType;
	};

}

#endif
