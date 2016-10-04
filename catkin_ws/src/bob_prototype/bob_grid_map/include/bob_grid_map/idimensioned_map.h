#ifndef _BOB_GRID_MAP_IDIMENSIONED_MAP_H_
#define _BOB_GRID_MAP_IDIMENSIONED_MAP_H_

namespace bob
{

	class MapMetadata;

	//! \brief Interface for a map that contains a size, and can be resized
	class IDimensionedMap
	{
	
		public:

			//! \brief Get the size and location of the map
			//! \return The size info
			virtual MapMetadata getMapMetadata() const = 0;

			//! \brief Resize the map and clear all the data. This is potentially
			//! more efficient because the data does not need to be copied.
			//! \param metadata The new size and dimensions of the map
			virtual void resizeClear(MapMetadata metadata) = 0;

			//! \brief Resize the map, but copy the data
			//! \param metadata The new size and dimensions of the map	
			virtual void resize(MapMetadata metadata) = 0;

	};

}

#endif
