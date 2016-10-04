#include <bob_boustrophedon/boustrophedon_executor.h>

#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_coverage/visualization.h>
#include <bob_visualization/visualization.h>

#include <bob_toolbox/logging.h>

namespace bob
{

	void BoustrophedonExecutor::execute()
	{
		DiscreteArea areaToCover = areaChooser.nextArea();
		while(!areaToCover.empty())
		{
			LOG_BOUSTROPHEDON("Next area suggested");
			visualizer->visualize("nextArea", visualization(areaToCover));

			std::vector<DiscreteArea> areaSet = areaToCover.eightConnectedSets< std::vector<DiscreteArea> >();
			if(!areaSet.empty())
			{
				boustrophedon.coverArea(areaSet[0]);
			}

			areaToCover = areaChooser.nextArea();
		}
		visualizer->visualize("nextArea", visualization(areaToCover));
	}
}

