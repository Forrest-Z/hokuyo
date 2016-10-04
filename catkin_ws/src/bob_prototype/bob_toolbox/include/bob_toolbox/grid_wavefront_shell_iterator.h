#ifndef _BOB_TOOLBOX_GRID_WAVEFRONT_SHELL_ITERATOR_H_
#define _BOB_TOOLBOX_GRID_WAVEFRONT_SHELL_ITERATOR_H_

#include <bob_toolbox/eight_direction.h>
#include <bob_toolbox/grid_wavefront_iterator.h>

#include <queue>
#include <vector>

namespace bob
{

	//! This is a templated iterator that produces a "wavefront" out from a seed cell (or collection of seed cells)
	//! Start by providing a single seed cell or a collection of seed cells.
	//! With each call of next(), a new shell of the wavefront is obtained by expanding the neighbors of the current cells in
	//! an eight-direction manner.
	//
	//! A functor that accepts a cell as an argument and returns a boolean is provided into the algorithm.
	//! If the functor evaluates a cell to be true it will not be added to next openlist and will instead be added to 
	//! an "edge" collection.
	//! The cells in the "edge" collection can be obtained at any time by calling getEdge() 
	//
	//! Before calling next(), be sure to test whether done() is true. done() returns true when there are
	//! no more cells to expand because the wave has been expanded into the limit of the edges.
	//
	//! Note: This algorithm does NOT expand in a perfectly circular manner. It expands in an octogon shape
	//! due to the principle by which it operates.
	template <class CellSet, class EdgeConditionFunctor>
		class GridWavefrontShellIterator : public GridWavefrontIterator<CellSet>
		{

			private:

				typedef typename CellSet::key_type Cell;

			public: 

				//! \brief Constructor that allows for a single seed cell
				//! \param closedSet The shell of already explored Cells
				//! \param seedPoint Seed cell from which wavefront start to expand
				//! \param functor Functor to check each Cell. If return true, the checked Cell 
				//! will be added to "edge" collection
				//! \param permanentEdge Permanent edge, Cells in this collection will never be 
				//! added to open collection
				GridWavefrontShellIterator(CellSet& closedSet, Cell seedPoint, const EdgeConditionFunctor& functor, const CellSet& permanentEdge) : 
					closedSet(closedSet),
					permanentEdge(permanentEdge),
					functor(functor)
				{
					examineAndAddToDst(seedPoint, currOpenSet);
				}

				//! \brief Constructor that allows for a collection of seeds from any type (due to templated designe)
				//! \param closedSet The shell of already explored Cells
				//! \param seedPoints Seed cells from which wavefront start to expand
				//! \param functor Functor to check each Cell. If return true, the checked Cell 
				//! will be added to "edge" collection
				//! \param permanentEdge Permanent edge, Cells in this collection will never be 
				//! added to open collection
				template <class OpenListType>
				GridWavefrontShellIterator(CellSet& closedSet, OpenListType seedPoints, const EdgeConditionFunctor& functor, const CellSet& permanentEdge) : 
					closedSet(closedSet),
					permanentEdge(permanentEdge),
					functor(functor)
				{
					for(typename OpenListType::const_iterator itr = seedPoints.begin(); itr != seedPoints.end(); ++itr)
					{
						examineAndAddToDst(*itr, currOpenSet);
					}
				}

				//! \brief Check if there are more cells to expand
				//! \return Return true if there are no more cells to expand
				virtual bool done() const
				{
					return currOpenSet.empty();
				}

				//! \brief Get the "edge" point of the wavefront
				//! This does NOT return the cells that have yet to be expanded
				//! It only returns the cells that have been examined and NOT opened
				//! because they satisfy the edge condition functor
				//! \return the "edge" of the wavefront.
				virtual CellSet getEdge() const
				{
					return edge;	
				}

				//! Expands a cell from the open set and adds the expanded cells
				//! to either the open set or edge set depending on result of functor.
				virtual void next()
				{	
					//! Save current open set so it can be returned by getCurrent
					current = currOpenSet;

					//! Expand each cell in currOpenSet
					CellSet neighborsToCheck = expandOpenShell();	

					//! Remove cells from closedSet to maintain a thin shell of closed
					removeInvalidClosedPoints(neighborsToCheck);
	
					//! All the points in currOpenSet were expanded
					GridWavefrontIterator<CellSet>::incrementNumberOfCellsExpanded(current.size());
				}

				//! \brief Get the current wavefront shell
				//! \return Current wavefront shell
				CellSet getCurrent()
				{
					return current;
				}

			private:

				//! Expands current open set
				CellSet expandOpenShell()
				{
					CellSet nextOpenSet;
					CellSet neighborsToCheck;
					for (typename CellSet::const_iterator itr = currOpenSet.begin(); itr != currOpenSet.end(); ++itr)
					{
						examineAndExpandNeighbors(*itr, nextOpenSet, neighborsToCheck);
						closedSet.insert(*itr);
					}
					currOpenSet = nextOpenSet;
					return neighborsToCheck;
				}

				//! Removes the given cells from the closed set that should no longer be in that shell
				void removeInvalidClosedPoints(const CellSet& set)
				{
					for (typename CellSet::const_iterator itr = set.begin(); itr != set.end(); ++itr)
					{
						examineAndRemoveFromClose(*itr);
					}					
				}

				//! Examine and add cell to dstSet 
				void examineAndAddToDst(const Cell& cell, CellSet& dstSet)
				{
					if (dstSet.count(cell) == 0 && closedSet.count(cell) == 0)
					{
						if(functor(cell))
						{
							edge.insert(cell);
						}
						else
						{
							//! Add cell to dst set if it is clear and free, and not in closed set
							dstSet.insert(cell);	
						}
					}	
				}

				//! Expand the neighbors of the cell, add closed neighbors to closedNeighbors set
				void examineAndExpandNeighbors(const Cell& cell, CellSet& dstSet, CellSet& closedNeighbors)
				{
					closedNeighbors.insert(cell);

					//! Get neighbor cells
					std::vector<Cell> neighbors = cellsInDirections(cell, allEightDirections());

					//! Check every neighbors
					for (typename std::vector<Cell>::const_iterator itr = neighbors.begin(); itr != neighbors.end(); ++itr)
					{
						if (closedSet.count(*itr) > 0)
						{
							if (currOpenSet.count(*itr) == 0 && closedNeighbors.count(*itr) == 0)	
							{
								//! Add cell to closedNeighbors if this neighbor is closed
								closedNeighbors.insert(*itr);
							}
						}
						else if (currOpenSet.count(*itr) == 0)
						{
							examineAndAddToDst(*itr, dstSet);
						}
					}
				}

				//! Examine the closed cell, if it's not on the shell, erase it from closeSet
				void examineAndRemoveFromClose(const Cell& cell)
				{
					//! Get neighbor cells
					std::vector<Cell> neighbors = cellsInDirections(cell, allEightDirections());
					
					for (typename std::vector<Cell>::const_iterator itr = neighbors.begin(); itr != neighbors.end(); ++itr)
					{
						if (closedPointValid(*itr))
						{
							//! Point is valid, so don't remove it
							return;
						}
					}
					
					//! Checked all neighbors, cells is invalid, so remove it	
					closedSet.erase(cell);
				}

				bool closedPointValid(const Cell& cell)
				{
					return (permanentEdge.count(cell) > 0 || functor(cell) || currOpenSet.count(cell) > 0);
				}

				//! The "queue" for the cells to expand
				//! Needed to ensure we don't add things to the queue twice
				CellSet currOpenSet;

				//! Wavefronts are put into closedSet after having been opoened
				CellSet& closedSet;

				const CellSet& permanentEdge;
				
				CellSet edge;

				const EdgeConditionFunctor& functor;

				CellSet current;

		};
}

#endif
