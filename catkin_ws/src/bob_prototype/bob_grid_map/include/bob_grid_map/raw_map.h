#ifndef _BOB_GRID_MAP_RAW_MAP_H_
#define _BOB_GRID_MAP_RAW_MAP_H_

#include <vector>
#include <bob_toolbox/world_point.h>
#include <bob_toolbox/int_point.h>
#include <bob_toolbox/array2d.h>
#include <bob_grid_map/map_metadata.h>
#include <bob_toolbox/geometry.h>

#include <bob_grid_map/map_location.h>
#include <bob_grid_map/location_mapper.h>
#include <bob_grid_map/idimensioned_map.h>

#include <cmath>

#include <assert.h>

namespace bob
{

	//! \brief A base class representing a discretized map in 2D space. Values in the map
	//! can be modified through references obtaind via the [] operator. The class inherits
	//! from abstract class LocationMapper, which gives it a co-ordinate mapping from world
	//! to map. It also implements IDimensionedMap, which contains function to query about
	//! map dimensions and resize the map.
	template <typename DataType>
		class RawMap : public virtual LocationMapper, public virtual IDimensionedMap
	{

		public:

			//! Reference to data in a cell
			typedef typename Array2D<DataType>::reference reference;

			//! const Reference to data in a cell
			typedef typename Array2D<DataType>::const_reference const_reference;	

			//! \brief Get a reference to a cell in the map
			//! \param point The location referring to the cell
			//! \return A reference to the cell
			reference operator[](MapLocation point)
			{
				assert(isInside(point));
				return data[getIndex(point)];
			}

			//! \brief Get a reference to a cell in the map
			//! \param point The location referring to the cell
			//! \return A reference to the cell
			const_reference operator[](MapLocation point) const
			{
				assert(isInside(point));
				return data[getIndex(point)];
			}

			//! \brief Check if a point lies within the map bounds
			//! \param point The point to check
			//! \return True if the point lies within map bounds
			bool isInside(MapLocation point) const
			{
				return data.isInside(getIndex(point));
			}

			//! \brief Gets the value stored at a given point.
			//! Note: This function will return defaultCellValue() if the point is
			//! not found in the map bounds, so if you are using this function you
			//! don't need to check isInside before you query a point, but you need
			//! to make sure your algorithm is aware that it should not continue forever
			//! if it keeps getting defaultCellValue() results.
			//! An alternative to this function is to use the square bracket operators []
			//! \param A point in world co-ordinates
			//! \return The value stored in the given cell, or defaultCellValue() if out of bounds
			DataType getValue(WorldPoint point) const
			{
				return getValue(worldToMap(point));
			}

			//! \brief Gets the value stored at a given point.
			//! Note: This function will return defaultCellValue() if the point is
			//! not found in the map bounds, so if you are using this function you
			//! don't need to check isInside before you query a point, but you need
			//! to make sure your algorithm is aware that it should not continue forever
			//! if it keeps getting defaultCellValue() results.
			//! An alternative to this function is to use the square bracket operators []
			//! \param A point in map co-ordinates
			//! \return The value stored in the given cell, or defaultCellValue() if out of bounds
			DataType getValue(MapLocation point) const
			{
				if (!isInside(point))
					return defaultCellValue();
				else
					return data[getIndex(point)];
			}

			//! \brief Resizes the map and copies the data. Some data may still be "cut off" by the new size.
			//! Note: This is a slower operation than "resizeClear," so only use it when you want to 
			//! preserve the map data.
			//! \param newMetadata The dimensions of the new map
			virtual void resize(MapMetadata newMetadata)
			{
				MapMetadata oldMetadata = getMapMetadata();
	
				// No need to resize if new size is same as old
				if (oldMetadata == newMetadata)
					return;

				// Allocate new array
				Array2D<DataType> newData = Array2D<DataType>(newMetadata.bounds, defaultCellValue());

				// Get the upper right corners of the map
				MapLocation oldUpperRightCorner(oldMetadata.upperRightCorner());
				MapLocation newUpperRightCorner(newMetadata.upperRightCorner());

				// Get the bounds for the part of the map to copy
				MapLocation copiedUpperRight = elementalMin(oldUpperRightCorner, newUpperRightCorner);
				MapLocation copiedBottomLeft = elementalMax(oldMetadata.bottomLeftCorner, newMetadata.bottomLeftCorner);

				// Copy map data into new Array2D
				for (int x = copiedBottomLeft.x; x <= copiedUpperRight.x; ++x)
				{
					for (int y = copiedBottomLeft.y; y <= copiedUpperRight.y; ++y)
					{
						IntPoint point(x, y);

						// This way is inefficient - there is a faster (less intuitive) way using the raw Array2D data
						newData[point - newMetadata.bottomLeftCorner] = (*this)[point];
					}
				}	

				// Swap in the data
				data = newData;
				bottomLeftCorner = newMetadata.bottomLeftCorner;
			}

			//! \brief Resizes the map and clears all the data
			//! \param metadata The dimensions of the new map
			virtual void resizeClear(MapMetadata metadata)
			{
				if (getMapMetadata() != metadata)
				{
					// Array resize is necessary since new size is different
					data = Array2D<DataType>(metadata.bounds, defaultCellValue());
				}
				else
				{
					// Array resize not necessary, just reset array values
					data.fill(defaultCellValue());	
				}

				bottomLeftCorner = metadata.bottomLeftCorner;
			}

			//! \brief Gets the dimensions of the current map
			//! \return The MapMetadata of the current map
			MapMetadata getMapMetadata() const
			{
				MapMetadata toReturn;
				toReturn.bounds.width = data.getWidth();
				toReturn.bounds.height = data.getHeight();
				toReturn.bottomLeftCorner = bottomLeftCorner;
				return toReturn;
			}

			//! \brief The value considered to be "unknown" or outside of the map space
			//! This virtual function is intended to be overriden in derived classes.
			//! \return The default value for a cell in the RawMap
			virtual DataType defaultCellValue() const
			{
				return DataType();
			}		

		private:

			//! Underlying data storage
			Array2D<DataType> data;

			//! Bottom left corner cell of map
			MapLocation bottomLeftCorner;

			//! \brief Converts a point from map co-ordinates to the
			//! co-ordinates of the underlying Array2D representation
			//! \param point A point in map co-ordinates
			//! \return The corresponding point in Array2D co-ordinates
			IntPoint getIndex(MapLocation point) const
			{
				return point - bottomLeftCorner;
			}

	};
} 

#endif
