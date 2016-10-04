#include <bob_grid_map/costmap.h>

namespace bob
{

	void Costmap::inflateObstacles()
	{
		inflatedCostmap.setInflationData(staticCostmap);
		
		// Set the flag indicating a map is available
		mapAvailable = true;
	}
}

