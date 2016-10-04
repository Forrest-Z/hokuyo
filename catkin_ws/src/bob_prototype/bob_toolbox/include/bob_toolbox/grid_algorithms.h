#ifndef _BOB_TOOLBOX_GRID_ALGORITHMS_H_
#define _BOB_TOOLBOX_GRID_ALGORITHMS_H_

#include <vector>
#include <queue>
#include <bob_toolbox/eight_direction.h>
#include <bob_toolbox/geometry.h>
#include <bob_toolbox/raytrace.h>

namespace bob
{

	//! \brief Gets the points in neighboring cells in the given directions.
	//! \param point The point to look around.
	//! \param directions The directions to look and get points.
	//! \return The co-ordinates around the point, in the given directions.
	template <class GridType> 
	std::vector<GridType> neighbors(GridType point, std::vector<EightDirection> directions)
	{
		std::vector<GridType> neighbors;
		for (std::vector<EightDirection>::iterator directionItr = directions.begin(); directionItr != directions.end(); ++directionItr)
			neighbors.push_back(cellInDirection(point, *directionItr));
		return neighbors;
	}

	//! \brief Finds and returns the subsets with the data which are connected in the given directions.
	//! \param The data to examine.
	template <class Container>
	std::vector<Container> connectedSets(Container data, std::vector<EightDirection> directionsToCheck)
	{
		typedef typename Container::value_type dataType;
		std::vector<Container> result;
		while (!data.empty())
		{
			Container newSet;
			
			std::queue<dataType> openPoints;
			openPoints.push(*data.begin());
			data.erase(openPoints.front());		
	
			while (!openPoints.empty())
			{
				dataType poppedPoint = openPoints.front();
				openPoints.pop();
				newSet.insert(newSet.end(), poppedPoint);

				std::vector<dataType> pointNeighbors = neighbors(poppedPoint, directionsToCheck);
				for (typename std::vector<dataType>::iterator itr = pointNeighbors.begin();
					itr != pointNeighbors.end(); ++itr)
				{
					if (data.count(*itr) == 1)
					{
						data.erase(*itr);
						openPoints.push(*itr);
					}
				}
			}
			result.push_back(newSet);
		}

		return result;
	} 

	//! Finds all the subsets within data that are connected non-diagonally (up, down, left, right)
	template <class Container>
	std::vector<Container> fourConnected(Container data)
	{
		return connectedSets(data, nondiagonalEightDirections());
	}

	//! Finds all the subsets within data that are connected diagonally or non-diagonally
	template <class Container>
	std::vector<Container> eightConnected(Container data)
	{
		return connectedSets(data, allEightDirections());
	}


	//! Generates all the cells that outline a polygon

	//! This is accomplished using raytracing between the cells
	template <class Cell>
		std::vector<Cell> polygonOutlineCells(const std::vector<Cell>& polygon) 
		{
			std::vector<Cell> outline;
			for (typename std::vector<Cell>::const_iterator itr = polygon.begin(); itr != (polygon.end() - 1); ++itr)
			{
				std::vector<Cell> border = raytraceLine(*itr, *(itr + 1));
				outline.insert(outline.begin(), border.begin(), border.end());
			}
			if (!polygon.empty())
			{
				//! we also need to close the polygon by going from the last cell to the first
				std::vector<Cell> closingBorder = raytraceLine(polygon.back(), polygon.front());
				outline.insert(outline.begin(), closingBorder.begin(), closingBorder.end());
			}
			return outline;
		}

	//! Generates all the cells that are used to fill a convex polygon

	//! If a non-convex polygon is used this algorithm produce unusual (and invalid) results.
	//! This algorithm does not check if a polygon is convex before processing.
	//! First it generates a polygon, then it raytraces along the y axis
	//! to fill that polygon.
	template <class Cell>
		std::vector<Cell> convexFillCells(const std::vector<Cell>& polygon) 
		{
			//! We need a minimum polygon of a triangle
			if (polygon.size() < 3)
				return std::vector<Cell>();

			//! First get the cells that make up the outline of the polygon
			std::vector<Cell> polygon_cells = polygonOutlineCells(polygon);

			//! Quick bubble sort to sort cells by x
			Cell swap;
			unsigned int i = 0;
			while (i < polygon_cells.size() - 1)
			{
				if (polygon_cells[i].x > polygon_cells[i + 1].x)
				{
					swap = polygon_cells[i];
					polygon_cells[i] = polygon_cells[i + 1];
					polygon_cells[i + 1] = swap;

					if (i > 0)
						--i;
				}
				else
					++i;
			}

			i = 0;
			Cell min_pt;
			Cell max_pt;
			int min_x = polygon_cells[0].x;
			int max_x = polygon_cells[polygon_cells.size() - 1].x;

			//! Walk through each column and mark cells inside the polygon
			for (int x = min_x; x <= max_x; ++x)
			{
				if (i >= polygon_cells.size() - 1)
					break;

				if (polygon_cells[i].y < polygon_cells[i + 1].y)
				{
					min_pt = polygon_cells[i];
					max_pt = polygon_cells[i + 1];
				}
				else
				{
					min_pt = polygon_cells[i + 1];
					max_pt = polygon_cells[i];
				}

				i += 2;
				while (i < polygon_cells.size() && polygon_cells[i].x == x)
				{
					if (polygon_cells[i].y < min_pt.y)
						min_pt = polygon_cells[i];
					else if (polygon_cells[i].y > max_pt.y)
						max_pt = polygon_cells[i];
					++i;
				}


				Cell pt;
				//! Loop though cells in the column
				for (int y = min_pt.y; y < max_pt.y; ++y)
				{
					pt.x = x;
					pt.y = y;
					polygon_cells.push_back(pt);
				}
			}
			return polygon_cells;
		}

	//! Finds the smallest distance between two sets of grid data 
	template <typename GridType>
	float minDistanceBetween(const GridType& grid1, const GridType& grid2)
	{
		float minDistance = std::numeric_limits<float>::max();

		for (auto& gridPoint1 : grid1)
		{
			for (auto& gridPoint2 : grid2)
			{
				float newDistance = diagonalDistance(gridPoint1, gridPoint2);
				if (newDistance < minDistance)
				{
					minDistance = newDistance;
				}
			}	
		}	

		return minDistance;
	}
}

#endif
