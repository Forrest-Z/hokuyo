#ifndef _BOB_TOOLBOX_GRID_WAVEFRONT_FULL_ITERATOR_H_
#define _BOB_TOOLBOX_GRID_WAVEFRONT_FULL_ITERATOR_H_

#include <bob_toolbox/eight_direction.h>
#include <bob_toolbox/grid_wavefront_iterator.h>

#include <queue>
#include <vector>

namespace bob
{

	//! This is a templated iterator that produces a "wavefront" out from a seed cell (or collection of seed cells)
	//! Start by providing a single seed cell or a collection of seed cells.
	//! With each call of next(), a new Cell is obtained by expanding the neighbors of the current cells in
	//! an eight-direction manner.
	//
	//! A functor that accepts a cell as an argument and returns a boolean is provided into the algorithm.
	//! If the functor evaluates a cell to be true it will not be returned in next() and will instead be added to 
	//! an "edge" collection.
	//! The points in the "edge" collection can be obtained at any time by calling getEdge() 
	//
	//! Before calling next(), be sure to test whether done() is true. done() returns true when there are
	//! no more points to expand because the wave has been expanded into the limit of the edges.
	//
	//! Note: This algorithm does NOT expand in a perfectly circular manner. It expands in an octogon shape
	//! due to the principle by which it operates.
	template <class CellSet, class EdgeConditionFunctor>
		class GridWavefrontFullIterator : public GridWavefrontIterator<CellSet>
		{

			private:

				typedef typename CellSet::key_type Cell;
	
			public: 

				//! Constructor that allows for a single seed point
				GridWavefrontFullIterator(CellSet& closedSet, Cell seedPoint, EdgeConditionFunctor& functor) : 
					closedSet(closedSet),
					functor(functor)
				{
					examineAndAdd(seedPoint);
				}

				//! Constructor that allows for a collection of seeds from any type (due to templated designe)
				template <class OpenListType>
				GridWavefrontFullIterator(CellSet& closedSet, OpenListType seedPoints, EdgeConditionFunctor& functor) : 
					closedSet(closedSet),
					functor(functor)
				{
					for(typename OpenListType::const_iterator itr = seedPoints.begin(); itr != seedPoints.end(); ++itr)
					{
						examineAndAdd(*itr);
					}
				}

				//! Returns true if there are no more points to expand
				virtual bool done() const
				{
					return openQueue.empty();
				}

				//! Returns the "edge" of the wavefront.
				//! This does NOT return the points that have yet to be expanded
				//! It only returns the points that have been examined and NOT opened
				//! because they satisfy the edge condition functor
				virtual CellSet getEdge() const
				{
					return edge;	
				}

				//! Expands a point from the open set and adds the expanded points
				//! to either the open set or edge set depending on result of functor.
				//! Returns the expanded point.
				virtual void next()
				{	
					//! Get the next point in the queue
					current = openQueue.front();
					openQueue.pop();	
					openSet.erase(current);

					//! Get neighbor points
					std::vector<Cell> nonDiagonalNeighbors = cellsInDirections(current, nondiagonalEightDirections());
					std::vector<Cell> diagonalNeighbors = cellsInDirections(current, diagonalEightDirections());

					//! Add appropriate neighbor points
					examineAndAdd(nonDiagonalNeighbors);
					examineAndAdd(diagonalNeighbors);

					//! Close the next point so we do not open it again
					closedSet.insert(current);

					//! We have expanded the openQueue.front
					GridWavefrontIterator<CellSet>::incrementNumberOfCellsExpanded(1);
				}

				Cell getCurrent()
				{
					return current;
				}

			private:

				//! Could replace this with for_each
				void examineAndAdd(std::vector<Cell>& points)
				{
					for (typename std::vector<Cell>::iterator itr = points.begin();
							itr != points.end();
							++itr)
					{
						examineAndAdd(*itr);
					}
				}

				void examineAndAdd(Cell point)
				{
					if (closedSet.count(point) == 0 && openSet.count(point) == 0)
					{
						if(functor(point))
						{
							edge.insert(point);
						}
						else
						{
							//! Add point to openQueue if it is clear and free, and not in closed set
							openQueue.push(point);
							openSet.insert(point);	
						}
					}
				}

				//! The queue for the cells to expand
				std::queue<Cell> openQueue;

				//! Wavefronts are put into closedSet after having been opoened
				CellSet& closedSet;

				//! Needed to ensure we don't add things to the queue twice
				CellSet openSet;

				CellSet edge;

				EdgeConditionFunctor& functor;

				Cell current;

		};

}

#endif
