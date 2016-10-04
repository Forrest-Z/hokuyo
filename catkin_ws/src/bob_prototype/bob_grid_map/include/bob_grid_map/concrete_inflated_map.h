#ifndef _BOB_GRID_MAP_CONCRETE_INFLATED_MAP_H_
#define _BOB_GRID_MAP_CONCRETE_INFLATED_MAP_H_

#include <queue>

#include <bob_grid_map/iobstacle_distance_map.h>
#include <bob_grid_map/raw_map.h>
#include <bob_grid_map/distance_cache_matrix.h>

namespace bob
{
	
	class IObstacleMap;

	class CellData
	{
		public:

			CellData(float distance, IntPoint point, IntPoint closestObstacle) :
				distance(distance), 
				point(point),
				closestObstacle(closestObstacle)
		{}


			//! Distance to obstacle
			float distance;

			//! The point
			IntPoint point;
	
			//! Candidate for closest obstacle to the point
			IntPoint closestObstacle;
	};

	inline bool operator<(const CellData &first, const CellData &second)
	{
		return first.distance > second.distance;
	}

	//! \brief Implementation of IObstacleDistanceMap. Caches a map of obstacle distances
	//! and provides access to them. 
	class ConcreteInflatedMap : public RawMap<float>, public IObstacleDistanceMap
	{
		public:

			//! \brief Construct a new ConcreteInflatedMap object
			//! \param resolution The resolution of the new map
			ConcreteInflatedMap(float resolution);

			//! \brief Cache new distance values based on an IObstacleMap
			//! \param map The map of obstacles, from which the cached distances will be calculated
			virtual void setInflationData(const IObstacleMap& map);

			//! \brief Overriding virtual function referring to default cell value
			//! \return max or infinity
			virtual float defaultCellValue() const;		

			//! \brief Get the distance to obstacles
			//! \param location The point to query
			//! \return The distance from the point to the nearest obstacle	
			virtual float obstacleDistance(MapLocation location) const;

		private:

			//! \brief Get IntVector objects pointing in all directions
			//! \return A collection of IntVector objects pointing in all directions
			std::vector<IntVector> vectorsInAllDirections();

			//! \brief Adds all the obstacles from the map into the queue
			//! \param sourceMap The input map containing obstacle information
			void enqueueObstacles(const IObstacleMap& sourceMap);

			//! \brief Keep expanding the queue until it is empty
			void expandQueueUntilEmpty();

			//! \brief Adds a point to the queue
			//! \param point The point to add
			//! \param closestObstacle Candidate for closest obstacle to the point
			void enqueue(IntPoint point, IntPoint closestObstacle);

			//! \brief Expands the next element in the queue
			void expandNextElement();

			//! Queue for points used when calculating distances
			std::priority_queue<CellData> inflationQueue;

			//! The points which have already been visited
			RawMap<bool> closedSet;
			
			//! Caches a small matrix of distance values to avoid calling hypot()
			DistanceCacheMatrix distanceCacheMatrix;

	};

} 

#endif  
