#include <bob_toolbox/eight_direction.h>

namespace bob
{

	EightDirection rotatedDirection(EightDirection initial, CircularDirection direction, int steps) 
	{
		int factor = (direction == CounterClockwise) ? 1 : -1;
		return (EightDirection) (((int) initial + steps * factor + 8) % 8);
	}

	const std::vector<EightDirection>& nondiagonalEightDirections()
	{
		static std::vector<EightDirection> nonDiagonals;
		if (nonDiagonals.size() == 0)
		{
			nonDiagonals.push_back(Up);
			nonDiagonals.push_back(Left);
			nonDiagonals.push_back(Down);
			nonDiagonals.push_back(Right);
		}
		return nonDiagonals;
	}

	const std::vector<EightDirection>& diagonalEightDirections()
	{
		static std::vector<EightDirection> diagonals;
		if (diagonals.size() == 0)
		{
			diagonals.push_back(UpperLeft);
			diagonals.push_back(UpperRight);
			diagonals.push_back(DownLeft);
			diagonals.push_back(DownRight);
		}
		return diagonals;
	}

	const std::vector<EightDirection>& allEightDirections()
	{
		static std::vector<EightDirection> all;
		if (all.size() == 0)
		{
			all.push_back(Up);
			all.push_back(Left);
			all.push_back(Down);
			all.push_back(Right);
			all.push_back(UpperLeft);
			all.push_back(UpperRight);
			all.push_back(DownLeft);
			all.push_back(DownRight);
		}
		return all;
	}

}

