#ifndef _BOB_TOOLBOX_EIGHT_DIRECTION_H_
#define _BOB_TOOLBOX_EIGHT_DIRECTION_H_

#include <vector>
#include <bob_toolbox/circular_direction.h>

namespace bob
{

	enum EightDirection 
	{
		Right,
		UpperRight,
		Up,
		UpperLeft,
		Left,
		DownLeft,
		Down,
		DownRight 
	};

	EightDirection rotatedDirection(EightDirection initial, CircularDirection direction=CounterClockwise, int steps=1);

	const std::vector<EightDirection>& nondiagonalEightDirections();

	const std::vector<EightDirection>& diagonalEightDirections();

	const std::vector<EightDirection>& allEightDirections();

	template <class Cell>
	Cell cellInDirection(const Cell& cell, EightDirection direction) 
	{
		Cell toReturn;
		switch (direction) 
		{
			case Right:
				toReturn.x = cell.x + 1;
				toReturn.y = cell.y;
				break;

			case UpperRight:
				toReturn.x = cell.x + 1;
				toReturn.y = cell.y + 1;
				break;

			case Up:
				toReturn.x = cell.x;
				toReturn.y = cell.y + 1;
				break;

			case UpperLeft:
				toReturn.x = cell.x - 1;
				toReturn.y = cell.y + 1;
				break;

			case Left:
				toReturn.x = cell.x - 1;
				toReturn.y = cell.y;
				break;

			case DownLeft:
				toReturn.x = cell.x - 1;
				toReturn.y = cell.y - 1;
				break;

			case Down:
				toReturn.x = cell.x;
				toReturn.y = cell.y - 1;
				break;

			case DownRight:
				toReturn.x = cell.x + 1;
				toReturn.y = cell.y - 1;
				break;
		}
		return toReturn;
	}

	template <class Cell>
	std::vector<Cell> cellsInDirections(const Cell& cell, std::vector<EightDirection> directions)
	{
		std::vector<Cell> neighbors;
		for (std::vector<EightDirection>::iterator directionItr = directions.begin(); directionItr != directions.end(); ++directionItr)
			neighbors.push_back(cellInDirection(cell, *directionItr));
		return neighbors;
	}

	template <class Cell>
	std::vector<Cell> fourNeighbors(Cell& cell)
	{
		return cellsInDirections(cell, nondiagonalEightDirections());
	}
}

#endif
