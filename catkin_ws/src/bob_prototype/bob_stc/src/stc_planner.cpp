#include <bob_stc/stc_planner.h>

#include <bob_stc/stc_path_combiner.h>

#include <list>

namespace bob
{

	STCPlan STCPlanner::createPlan(const DiscreteArea& toCover) const
	{
		DiscreteArea remainingArea = toCover;

		DiscreteArea covered(toCover.getResolution());

		// Try to cover area with plans
		std::vector<STCPath> firstPaths = optimalSTCGenerator.tryToCoverArea(toCover, covered);

		// Update the remaining areas
		remainingArea.erase(covered);

		DiscreteArea finalCovered(toCover.getResolution());

		// Try to cover a second time to get as many remaining area as possible
		std::vector<STCPath> remainingPaths = optimalSTCGenerator.tryToCoverArea(remainingArea, finalCovered);

		// Combine the plans
		std::list<STCPath> stcPaths;
		stcPaths.insert(stcPaths.end(), firstPaths.begin(), firstPaths.end());
		stcPaths.insert(stcPaths.end(), remainingPaths.begin(), remainingPaths.end());


		STCPathCombiner combiner;
		STCPlan toReturn = combiner.combinePaths(stcPaths);

		return toReturn;
	}

}
