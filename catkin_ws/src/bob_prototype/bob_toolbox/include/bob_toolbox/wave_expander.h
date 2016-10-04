#ifndef _BOB_TOOLBOX_WAVE_EXPANDER_H_
#define _BOB_TOOLBOX_WAVE_EXPANDER_H_

#include <bob_toolbox/grid_wavefront_iterator.h>
#include <limits>

namespace bob
{
	
	//! Expands a GridWavefrontIterator object to its edge points. Optional maxCellsToExpand arg 
	//! limits the loop to a max number of cells, preventing alloc problems. 
	template <typename SetType>
	void waveExpand(GridWavefrontIterator<SetType>& wavefrontIterator, int maxCellsToExpand = std::numeric_limits<int>::max())
	{
		while (!wavefrontIterator.done() && 
			wavefrontIterator.getNumberOfCellsExpanded() < maxCellsToExpand)
                {
                       	wavefrontIterator.next();
                }
	}

}

#endif
