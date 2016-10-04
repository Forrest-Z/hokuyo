#ifndef _BOB_STC_STC_PLANNER_H_
#define _BOB_STC_STC_PLANNER_H_

#include <bob_stc/stc_plan.h>
#include <bob_coverage/discrete_area.h>
#include <bob_stc/optimal_stc_generator.h>

#include <bob_grid_map/iobstacle_map.h>

namespace bob
{

	//! This class creates an STCPlan used to cover a given area
	//! An STCPlan is a serious of STCPaths connected by LineSegments
	//! The LineSegment connections are generated as the shortest
	//! segments connecting the adjacent paths
	class STCPlanner
	{

		public:

			STCPlanner(const IObstacleMap& obstacleMap) :
				optimalSTCGenerator(obstacleMap)
		{}

			//! Creates a "good" STCPlan from a DiscreteArea
			//
			//! Input:
			//! toCover - The area that we want to be covered using the STC plan
			//
			//! Returns: An STC plan that suitably covers the area
			STCPlan createPlan(const DiscreteArea& toCover) const;

		private:

			//! Used to generate paths for given areas (which are later combined into STCPlan)
			OptimalSTCGenerator optimalSTCGenerator;

	};

}

#endif
