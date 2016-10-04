#ifndef _BOB_TOOLBOX_GRID_WAVEFRONT_ITERATOR_H_
#define _BOB_TOOLBOX_GRID_WAVEFRONT_ITERATOR_H_

#include <unordered_set>

namespace bob
{

	//! Abstract class for grid wavefront iterators. Seed point(s) are expanded until "edge points" are reached. These terminus "edge points" can then be retrieved as needed.

	//! These seed points are "expanded" outwardly to explore the area.
	template <typename CellSet>
	class GridWavefrontIterator
	{

		public:		

			GridWavefrontIterator<CellSet>() :
			numberOfCellsExpanded(0)
			{}

			//! No more cells to expand
			virtual bool done() const = 0;

			//! Gets the current "edge" (the points where the expansion terminates)
			virtual CellSet getEdge() const = 0;

			//! Moves the iteration forward by one
			virtual void next() = 0;

			int getNumberOfCellsExpanded() const
			{
				return numberOfCellsExpanded;
			}

		protected:

			//! Used by derived classes to update the number of cells expanded
			//! This is not a very good design, and needs to be updated.
			void incrementNumberOfCellsExpanded(int increment)
			{
				numberOfCellsExpanded += increment;
			}

		private:

			int numberOfCellsExpanded;
			
	};

}

#endif
