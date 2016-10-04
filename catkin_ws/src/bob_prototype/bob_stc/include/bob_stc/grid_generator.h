#ifndef _BOB_STC_GRID_GENERATOR_H_
#define _BOB_STC_GRID_GENERATOR_H_

#include <bob_grid_map/iobstacle_map.h>
#include <bob_stc/grid_set.h>
#include <bob_toolbox/world_point.h>
#include <bob_grid_map/map_location_set.h>

#include <bob_config/config.h>

#include <bob_coverage/discrete_area.h>

namespace bob
{

	class GridGenerator
	{

		public:

			GridGenerator(const IObstacleMap& obstacleMap) :
				obstacleMap(obstacleMap),
				gridWidth(Config::STC_GRID_RADIUS_MULTIPLE * Config::STC_ROBOT_COVERAGE_RADIUS),
				coverageWidth(std::max(gridWidth, gridWidth / 2 + Config::STC_ROBOT_COVERAGE_RADIUS * 2)) {}

			GridSet optimalSet(const DiscreteArea& areaToCover, DiscreteArea& coveredArea) const;

			GridSet generateSet(DiscreteArea areaToCover, WorldPoint origin, float rotation, DiscreteArea& coveredArea) const;

		private:

			std::vector<WorldPoint> gridOriginsToTry() const;

			std::vector<float> anglesToTry() const;

			//! Store a local reference to an inflated costmap
			const IObstacleMap& obstacleMap;

			//! Width of the grid for STC coverage
			float gridWidth;

			//! The actual width that the robot will pass over (is only equal to gridWidth when gridWidth = 4 * robotRadius)
			float coverageWidth;

	};

}

#endif
