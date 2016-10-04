#ifndef _BOB_GRID_MAP_LOCATION_MAPPER_H_
#define _BOB_GRID_MAP_LOCATION_MAPPER_H_

#include <bob_toolbox/world_point.h>
#include <bob_grid_map/map_location.h>
#include <bob_config/config.h>
#include <algorithm>
#include <boost/bind.hpp>

namespace bob
{

	//! \brief Absract class which provides a mapping from the world co-ordinates and a "map" 
	//! co-ordinates. This mapping is defined entirely by the resolution of the map, with the
	//! resolution defining a scaling factor. (0, 0) in world co-ordinates always represents (0, 0) in
	//! map co-ordinates. Other points are mapped based on the scaling factor of the map resolution.
	//! The class is designed to be a virtual base class for other map classes, and used for it's
	//! public member functions, which provide the co-ordinate transform system used by the map.
	class LocationMapper
	{
		
		public:
	
			//! \brief Default constructor sets map resolution based on config,
			//! but another resolution may be specified.

			//! This default value is specified to remove the requirement
			//! for constructors in all virtual base classes of this class.
			//! ie. It improves code readability elsewhere
			//! Otherwise, due to a quirk of public virtual inheritance,	
			//! you need to initialize LocationMapper even in interfaces, which
			//! looks ugly and is confusing.
			//! \param resolution The resolution of the map
			LocationMapper(float resolution = Config::MAP_RESOLUTION);
		
			//! \brief Convert a point from map co-ordinates to world co-ordinates
			//! \param location The MapLocation to convert
			//! \return The corresponding point in world co-ordinates.
			WorldPoint mapToWorld(const MapLocation location) const;

			//! \brief Convert a point from world co-ordinates to map co-ordinates
			//! \param point The WorldPoint to conert
			//! \return The corresponding point in map co-ordinates 
			MapLocation worldToMap(const WorldPoint point) const;

			//! \brief A continuous version of worldToMap. The ouput is not
			//! rounded to the nearest cell. 
			//! \param The point to convert
			//! \return The resulting point in map co-ordinates
			GenericPoint<float> worldToMapContinuous(const WorldPoint& point) const;

			//! \brief A continuous version of mapToWorld.
			//! \param The point to convert.
			//! \return The resulting point in world co-ordinates
			WorldPoint mapToWorldContinuous(const GenericPoint<float>& point) const;

			//! \brief Gets the resolution of the map
			//! \return The map's resolution (width of a cell)
			float getResolution() const;

			//! \brief Special templated version of worldToMap which is used for bulk conversions
			//! \param data An iterable container containing world points
			//! \return A container containing the converted points in map co-ordinates
			template <typename OutputContainer, typename InputContainer>
				OutputContainer worldToMap(const InputContainer& data) const
				{
					OutputContainer toReturn;	
					MapLocation (LocationMapper::*discreteFunction) (WorldPoint) const = &LocationMapper::worldToMap;
					std::transform(data.begin(), data.end(), std::back_inserter(toReturn), boost::bind(discreteFunction, this, _1));
					return toReturn;
				}	

			//! \brief Special templated version of mapToWorld which is used for bulk conversions
			//! \param data An iterable container containing map points
			//! \return A container containing the converted points in world co-ordinates
			template <typename OutputContainer, typename InputContainer>
				OutputContainer mapToWorld(const InputContainer& data) const
				{
					OutputContainer toReturn;	
					WorldPoint (LocationMapper::*continuousFunction) (MapLocation) const = &LocationMapper::mapToWorld;
					std::transform(data.begin(), data.end(), std::back_inserter(toReturn), boost::bind(continuousFunction, this, _1));
					return toReturn;
				}	

		private: 
			
			//! Map resolution
			float resolution;

	};

}

#endif
