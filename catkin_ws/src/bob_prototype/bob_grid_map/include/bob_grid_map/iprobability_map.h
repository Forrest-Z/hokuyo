#ifndef _BOB_GRID_MAP_IPROBABILITY_MAP_H_
#define _BOB_GRID_MAP_IPROBABILITY_MAP_H_

#include <bob_grid_map/location_mapper.h>

#include <bob_toolbox/bounds2d.h>
#include <bob_grid_map/map_location.h>
#include <bob_grid_map/idimensioned_map.h>

namespace bob 
{

	//! \brief Interface for a map used within the SLAM system. Any map which
	//! implements this interface can be used within the SLAM packages. Such
	//! a map will likely also impelement IObstacleMap, if it will be used simultaneously
	//! within the rest of the system for obstacle data.
	//! The interface defines a map which associates points with the probability
	//! that the point is an obstacle.
	class IProbabilityMap : public virtual LocationMapper, public virtual IDimensionedMap
	{

		public:

			//! The data type used to represent obstacle probability
			typedef float data_type;

			//! \brief Updates a point in the probability map after evidence shows it 
			//! is an obstacle. A weighting factor may be added in the future to account
			//! for changes in the accuracy of the sensor.
			//! \param point The point to update
			virtual void setObstacle(MapLocation point) = 0;
	
			//! \brief Updates a point in the probability map after evidence shows it 
			//! is free. This is the counterpoint to "setObstacle" function.
			//! \param point The point to update
			virtual void setFree(MapLocation point) = 0;
		
			//! Expands the size of the probability map up to a certain bounds
			virtual void expandToIncludeBounds(Bounds2D<float> bounds) = 0;
	
			//! Gets the probability that a point in the map is occupied
			//! \param point The point to query
			virtual data_type getProbability(MapLocation point) const = 0;

			virtual void setValue(MapLocation point, float probability) = 0;

	};

}

#endif

