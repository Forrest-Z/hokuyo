#ifndef _BOB_GRID_MAP_DECAY_PROBABILITY_CELL_H_
#define _BOB_GRID_MAP_DECAY_PROBABILITY_CELL_H_

#include <bob_grid_map/free_state.h>

namespace bob 
{

	//! \brief Represents a cell in a probability map. The cell is updated,
	//! using a reading, as either occupied or free. This changes its probability
	//! value.
	class DecayProbabilityCell
	{
		public:

			//! \brief Constructs a cell starting with unknown probability
			DecayProbabilityCell() :
			probability(255)
	 		{}

			//! \brief Constructs a cell starting with an initial probability
			DecayProbabilityCell(float probability) :
			probability(probability)
	 		{}

			//! \brief Updates the cell with a reading as free or occupied
			//! \param occupied If true, updating cell as occupied, if false, updating cell as free
			void update(bool occupied);

			//! \brief Gets the usable probability value of the cell
			float getProbability() const;

			FreeState getFreeState() const;

		private:

			unsigned char probabilityConversion(float probability);

			//! The probability value 0 to 254 where 255 is unknown
			unsigned char probability;
			
	};

};

#endif 
