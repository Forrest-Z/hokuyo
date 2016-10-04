#ifndef _BOB_GRID_MAP_COUNTING_PROBABILITY_CELL_H_
#define _BOB_GRID_MAP_COUNTING_PROBABILITY_CELL_H_

#include <bob_grid_map/free_state.h>

namespace bob 
{

	//! This is the old probability cell that was originally used in hector slam
	//! There is a new version: DecayProbabilityCell. 
	class CountingProbabilityCell
	{
		public:

			CountingProbabilityCell() : 
			occupiedHits(0), 
			totalHits(0) 
			{}
	
			CountingProbabilityCell(float initialProbability) : 
			occupiedHits(initialProbability * 100), 
			totalHits(100) 
			{}

			void update(bool occupied);

			FreeState getFreeState() const;

			float getProbability() const;

		private:

			int occupiedHits, totalHits;
	};


};

#endif 
