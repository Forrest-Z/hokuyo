#ifndef _BOB_NAVIGATION_DIJKSTRA_PLANNER_H_
#define _BOB_NAVIGATION_DIJKSTRA_PLANNER_H_

#include <bob_grid_map/raw_map.h>
#include <bob_grid_map/map_location_set.h>
#include <bob_toolbox/eight_direction.h>

#include <limits> 
#include <queue>

namespace bob
{

	class Costmap;

	//! A point used in the potential map
	struct DijkstraPoint
	{
		DijkstraPoint(MapLocation previous = MapLocation(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()), float potential = std::numeric_limits<float>::infinity()) :
			previous(previous),
			potential(potential)
		{}

		float potential;

		MapLocation previous;
	};

	//! Stores an array of potentials as well as a reverse path from any point to goal
	class PotentialMap : public RawMap<DijkstraPoint>
	{

		private:

			virtual DijkstraPoint defaultCellValue() const
			{
				return DijkstraPoint();
			}
	};

	//! Stores a candidate potential and it's location in the map
	//! This is used for the priority queue representing the open set
	struct PotentialPoint
	{
		PotentialPoint(MapLocation location, float potential) :
			location(location),
			potential(potential)
		{}

		float potential;

		MapLocation location;

		bool operator<(const PotentialPoint& other) const
		{
			//! Smallest potential is the best
			return potential > other.potential;
		}
	};

	//! Implements dijkstra's algorithm in order to generate a path between two points in the world.
	class DijkstraPlanner
	{

		public:

			DijkstraPlanner(const Costmap& map) :
				map(map)
		{}

			//! Makes a plan between two points in the world
	
			//! @param start The starting point of the plan
			//! @param goal The goal point of the plan
			//! @param[out] plan The resulting plan
			//! @return True if a plan was successfully made, false otherwise 
			bool makePlan(WorldPoint start, WorldPoint goal, std::vector<WorldPoint>& plan);

		private:

			static const float crossFactor;

			//! Clears the datatypes so that the same planner can be used repeatedly without re-initialization
			void reset();

			//! Computes the potential map based on a costmap, start point and endpoint
			bool computePotential(MapLocation start, MapLocation goal);

			//! Computes a plan based on the potential map
			std::vector<WorldPoint> computePlanFromPotential(MapLocation start, MapLocation goal);

			//! Calculate potential of any given point. This is used to adjust the "potential field"
			//! and directly affects the qualitative shape of the result
			float calculatePotential(MapLocation point);

			//! Expand all the points in a given direction and multiply cost by factor (diagonal directions travel
			//! further and thus require greater cost
			void expandPointsInDirections(std::vector<EightDirection> directionsToCheck, PotentialPoint source, float factor);

			//! Determines how far the potential chain has travelled into unknown space
			//! Uses the PotentialMap to determine this
			int distanceTravelledInUnknown(MapLocation point);

			//! The points which have been expanded (main contain duplicates with diff potentials)
			//! Used as an intermediate when calculating the potential map
			std::priority_queue<PotentialPoint> openQueue; 

			//! The map of all potentials
			PotentialMap potential;	

			//! The nodes which have already been visited (AFTER being expanded one or more times)
			MapLocationSet closedSet;

			const Costmap& map;

	};

}

#endif
