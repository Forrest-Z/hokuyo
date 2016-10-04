#include <bob_core/area_coverer.h>

#include <bob_visualization/visualization.h>
#include <bob_coverage/area_tools.h>
#include <bob_coverage/visualization.h>
#include <bob_toolbox/pose2d.h>

namespace bob
{

	void AreaCoverer::cover(DiscreteArea area)
	{
		visualizer->visualize("area_covering", visualization(area));

		// Group areas into connected sets
		auto areas = area.eightConnectedSets<std::list<DiscreteArea> >();
		
		// Cover each sub-area, and erase coverd sub-area from total area.
		while (areas.size() > 0)
		{
			// Get closest area
			WorldPoint robotPosition = sensorHandle.getTransformHandle().getLocalizedPose();
			auto itrToNext = closestArea(areas, robotPosition);
		
			if (itrToNext->size() > 10)
			{	
				// Cover colsest area
				coverSubsection(*itrToNext);
			}

			// Erase covered area from total area
			areas.erase(itrToNext);
		}
	}

	void AreaCoverer::coverSubsection(DiscreteArea area)
	{
		visualizer->visualize("area_portion", visualization(area));

		// Try to cover the whole area
		coverageAlgorithm.coverArea(area);

		// Erase covered area
		area.erase(areaProcessor.getCoveredArea());

		// Group into connected set
		auto areas = area.eightConnectedSets<std::vector<DiscreteArea> >();

		// Cover each sub-area
		for (auto areaItr = areas.begin(); areaItr != areas.end(); ++areaItr)
		{
			if (areaItr->size() > 10)
			{
				visualizer->visualize("area_portion", visualization(*areaItr));
				coverageAlgorithm.coverArea(*areaItr);
			}
		}

	}

}

