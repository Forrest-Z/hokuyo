#ifndef _BOB_GRID_MAP_COSTMAP_H_
#define _BOB_GRID_MAP_COSTMAP_H_

#include <bob_grid_map/concrete_map.h>
#include <bob_grid_map/concrete_inflated_map.h>
#include <bob_grid_map/counting_probability_cell.h>
#include <bob_grid_map/decay_probability_cell.h>

#include <bob_toolbox/world_point.h>

namespace bob
{

	//! \brief This class stores a map representation of the world. There are two components 
	//! to this map, the obstacle map and the inflated map. The obstacle map is simply a map
	//! of obstacles present in the world. The inflated map is a map of cached distance values,
	//! where the distance stored represents the distance to the nearest obstacle. Clients
	//! access the costmap using on of the 3 interfaces it provides (IProbabilityMap,
	//! IObstacleMap and IObstacleDistanceMap). This costmap is not made to be thread safe. LockableMap
	//! is the thread-safe version of the map.

	//! SLAM algorithms can modify this map using access to the IProbabilityMap,
	//! then calling inflateObstacles() to calculate the IObstacleDistanceMap.
	//
	//! The coverage, control, etc. algorithms can then use the IObstacleMap and
	//! the IObstacleDistanceMap to get information about the world, as represented by the map.
	//
	//! Note: All the maps in the system have the same resolution. Costmap can be
	//! used as a LocationMapper to map points to and from map co-ordinates.
	//! The conversions are valid for any of the maps, since they have the same resolutions.
	class Costmap : public LocationMapper
	{

		public:

			//! \brief Basic constructor.
			//! \param resolution The resolution of the new map.
			Costmap(float resolution) :
			staticCostmap(resolution),
			inflatedCostmap(resolution),
			LocationMapper(resolution),
			mapAvailable(false)
			{}

			//! \brief Get interface to obstacle distance data
			const IObstacleDistanceMap& getObstacleDistanceMap() const
			{
				return inflatedCostmap;
			}

			//! \brief Get interface to obstacle data
			const IObstacleMap& getObstacleMap() const
			{
				return staticCostmap;
			}	

			//! \brief Get interface to probability map. Modifying this map
			//! is the main interface for modifying the overall costmap
			IProbabilityMap& getProbabilityMap()
			{
				return staticCostmap;
			}	

			IHiddenObstacleMap& getHiddenObstacleMap()
			{
				return staticCostmap;
			}

			//! \brief Recalculate the IObstacleDistanceMap based on the IObstacleMap (ie. probabilitymap)
			//! First, update the probability map, then call this function to inflate the obstacles
			void inflateObstacles();

			//! \brief Returns true if a map is available
			bool isMapAvailable() const
			{
				return mapAvailable;
			}
	
		private:

			//! Concrete IObstacleMap and IProbabilityMap representation
			ConcreteMap<DecayProbabilityCell> staticCostmap;

			//! Concrete IObstacleDistanceMap representation
			ConcreteInflatedMap inflatedCostmap;

			//! If true, the object has been initialized with an inflated map
			bool mapAvailable;

			//! Copy constructor private to prevent accidental copying
			Costmap(const Costmap& other);

			//! Assignment operator private to prevent accidental copying
			void operator=(const Costmap& other);

	};

}

#endif
