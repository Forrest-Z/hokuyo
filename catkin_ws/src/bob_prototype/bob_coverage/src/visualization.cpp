#include <bob_coverage/visualization.h>

#include <bob_coverage/discrete_area.h>

namespace bob
{

	MarkerSquares visualization(const DiscreteArea& area)
	{
		MarkerSquares markerSquares;
		markerSquares.width = area.getResolution();

		for (auto areaItr = area.begin(); areaItr != area.end(); ++areaItr)
		{
			WorldPoint point = area.mapToWorld(*areaItr);
			markerSquares.data.push_back(point);
		}

		return markerSquares;
	}

}

