#include <bob_stc/stc_plan_tools.h>

#include <bob_toolbox/geometry.h>

#include <boost/utility.hpp>

#include <limits>

namespace bob
{

	STCPlan reorganizeAboutPoint(const STCPlan& plan, WorldPoint point)
	{
		float shortestDistance = std::numeric_limits<float>::max();
		STCPlan::const_iterator splitPath;
		STCPath::const_iterator splitPoint;
		for (STCPlan::const_iterator planItr = plan.begin(); planItr != plan.end(); ++planItr)
		{
			for (STCPath::const_iterator pathItr = planItr->begin(); pathItr != planItr->end(); ++pathItr)
			{
				float distance = diagonalDistance(*pathItr, point);		
				if (distance < shortestDistance)
				{			
					shortestDistance = distance;
					splitPath = planItr;
					splitPoint = pathItr;
				}
			}
		}
		

		STCPlan result;
		if (splitPoint == splitPath->begin())
		{
			result.insert(result.end(), splitPath, plan.end());
			result.insert(result.end(), plan.begin(), splitPath);
		}
		else
		{
			if (boost::next(splitPoint) != splitPath->end())
				result.insert(result.end(), STCPath(splitPoint, splitPath->end()));

			result.insert(result.end(), boost::next(splitPath), plan.end());
			result.insert(result.end(), plan.begin(), splitPath);

			if (splitPoint != splitPath->begin())
				result.insert(result.end(), STCPath(splitPath->begin(), boost::next(splitPoint)));
		}
		return result;
	}

}

