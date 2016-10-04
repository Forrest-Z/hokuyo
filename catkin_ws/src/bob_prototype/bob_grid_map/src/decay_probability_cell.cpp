#include <bob_grid_map/decay_probability_cell.h>

#include <bob_config/config.h>

namespace bob 
{

	void DecayProbabilityCell::update(bool occupied)
	{
		// First time initialization
		if (probability == 255)
		{
			probability = probabilityConversion(Config::OBSTACLE_THRESHOLD_PROBABILITY);

			// Update recursively (should only go down one level)
			update(occupied);
		}
		else
		{
			// Get probability representation of bool
			float adjustVal = occupied ? 1.0 : 0.0;

			float oldProbability = getProbability();

			// This is a low-pass filter equation
			float newProbability = oldProbability + Config::SLAM_PROBABILITY_DECAY_FACTOR * (adjustVal - oldProbability);
	
			probability = probabilityConversion(newProbability);
		}
	}

	float DecayProbabilityCell::getProbability() const 
	{ 
		if (probability == 255)
		{
			return -1;
		}
		else
		{
			return (float)probability / 254.0;
		}
		
		return probability;
	}

	FreeState DecayProbabilityCell::getFreeState() const
	{
		float currentProbability = getProbability();
		if (currentProbability < 0)
			return Unknown;	
		else if (currentProbability < 0.25)
			return Free;
		else
			return SensedObstacle;
	}

	unsigned char DecayProbabilityCell::probabilityConversion(float probability)
	{
		return (unsigned char)(254 * probability);
	}
};


