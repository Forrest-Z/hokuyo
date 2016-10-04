#ifndef _BOB_STC_OPTIMAL_STC_GENERATOR_H_
#define _BOB_STC_OPTIMAL_STC_GENERATOR_H_

#include <bob_grid_map/iobstacle_map.h>

#include <vector>

#include <bob_coverage/discrete_area.h>
#include <bob_stc/grid_set.h>
#include <bob_stc/stc_plan.h>
#include <bob_stc/grid_generator.h>

namespace bob
{

	class OptimalSTCGenerator
	{

		public:

			OptimalSTCGenerator(const IObstacleMap& obstacleMap) :
				gridGenerator(obstacleMap) 
		{}

			//! Tries to generate STCPaths to cover a discrete area
			//
			//! Input:
			//! areaToCover - The area we are trying to cover
			//! areaCovered - The total area covered
			//
			//! Returns: A vector of the paths
			std::vector<STCPath> tryToCoverArea(const DiscreteArea& areaToCover, DiscreteArea& areaCovered) const;

		private:

			//! Generates an STCPath from a GridSet using the GridGenerator
			STCPath pathForGrid(const GridSet& set) const;

			//! Separates a discrete area into several smaller areas that are not connected together
			//! If areas are close enough together they will be combined even if they are not touching
			//! This is done to prevent multiple tiny areas each getting their own plans
			std::vector<DiscreteArea> separateAreas(const DiscreteArea& area) const;

			GridGenerator gridGenerator;

	};

}

#endif
