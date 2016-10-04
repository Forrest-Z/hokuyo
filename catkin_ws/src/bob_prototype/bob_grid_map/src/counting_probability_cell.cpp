#include <bob_grid_map/counting_probability_cell.h>

namespace bob 
{

	void CountingProbabilityCell::update(bool occupied)
	{
		if (occupied) 
		{
			occupiedHits++; 
		}
		totalHits++;
	}

	FreeState CountingProbabilityCell::getFreeState() const
	{
		float currentProbability = getProbability();
		if (currentProbability == -1)
			return Unknown;
		else if (currentProbability < 0.25)
			return Free;
		else
			return SensedObstacle;

	}

	float CountingProbabilityCell::getProbability() const
	{
		return totalHits ? (float)occupiedHits / (float)totalHits : -1; 
	}

};


