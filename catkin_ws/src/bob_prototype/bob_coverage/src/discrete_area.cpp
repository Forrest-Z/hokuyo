#include <bob_coverage/discrete_area.h>
#include <cmath>
#include <bob_toolbox/grid_algorithms.h>
#include <bob_toolbox/line_geometry.h>

namespace bob
{

	std::vector<DiscreteArea> DiscreteArea::fourConnectedSets() const
	{
		std::vector<DiscreteArea> result;
		std::vector<MapLocationSet> connectedList = fourConnected(mask);
		for (std::vector<MapLocationSet>::iterator listItr = connectedList.begin(); listItr != connectedList.end(); ++listItr)
		{
			DiscreteArea newSet(getResolution());
			newSet.mask = *listItr;
			result.push_back(newSet);
		}
		return result;
	}

	
}

