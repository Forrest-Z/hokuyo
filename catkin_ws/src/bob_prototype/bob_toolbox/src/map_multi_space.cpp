#include <bob_toolbox/map_multi_space.h>

namespace bob
{

bool MapMultiSpace::contains(WorldPoint point) const
{
	bool result = false;
	if (spaceType == AND)
	{
		result = true;
		for (std::vector<MapHalfSpace>::const_iterator itr = spaces.begin(); itr != spaces.end(); ++itr)
		{
			if (!itr->contains(point))
			{
				result = false;
				break;
			}
		}	
	}
	else if (spaceType == OR)
	{
		result = false;
		for (std::vector<MapHalfSpace>::const_iterator itr = spaces.begin(); itr != spaces.end(); ++itr)
		{
			if (itr->contains(point))
			{
				result = true;
				break;
			}
		}	
	}
	return result;
}
 
}
