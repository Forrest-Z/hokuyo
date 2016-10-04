#include <gtest/gtest.h>
#include <bob_grid_map/raw_map.h>

using namespace bob;

TEST (MapResize, consistent) 
{ 

	RawMap<int> map;
	MapMetadata initialShape;
	initialShape.bounds.width = 3;
	initialShape.bounds.height = 3;
	initialShape.bottomLeftCorner = MapLocation(0, 0);
	
	map.resize(initialShape);

	std::vector<std::pair<MapLocation, int> > input;

	input.push_back(std::make_pair(MapLocation(0, 0), 2));
	input.push_back(std::make_pair(MapLocation(2, 2), 3));
	input.push_back(std::make_pair(MapLocation(0, 2), 4));
	input.push_back(std::make_pair(MapLocation(1, 1), 5));

	for (auto inputItr = input.begin(); inputItr != input.end(); ++inputItr)
	{
		map[inputItr->first] = inputItr->second;
	}
	
	MapMetadata secondShape;
	secondShape.bounds.width = 5;
	secondShape.bounds.height = 5;
	secondShape.bottomLeftCorner = MapLocation(-1, -1);
	
	map.resize(secondShape);

	for (auto inputItr = input.begin(); inputItr != input.end(); ++inputItr)
	{
		ASSERT_EQ(map[inputItr->first], inputItr->second);
	}

	//ASSERT_EQ(val, actual);
	//ASSERT_EQ(WorldPoint(2, 2), WorldPoint(1, 1) + WorldVector(1, 1));
}

TEST (MapResize, consistent2) 
{ 

	RawMap<int> map;
	MapMetadata initialShape;
	initialShape.bounds.width = 3;
	initialShape.bounds.height = 3;
	initialShape.bottomLeftCorner = MapLocation(0, 0);
	
	map.resize(initialShape);

	std::vector<std::pair<MapLocation, int> > input;

	input.push_back(std::make_pair(MapLocation(0, 0), 2));
	input.push_back(std::make_pair(MapLocation(2, 2), 3));
	input.push_back(std::make_pair(MapLocation(0, 2), 4));
	input.push_back(std::make_pair(MapLocation(1, 1), 5));

	for (auto inputItr = input.begin(); inputItr != input.end(); ++inputItr)
	{
		map[inputItr->first] = inputItr->second;
	}
	
	MapMetadata secondShape;
	secondShape.bounds.width = 8;
	secondShape.bounds.height = 8;
	secondShape.bottomLeftCorner = MapLocation(-5, -5);
	
	map.resize(secondShape);

	for (auto inputItr = input.begin(); inputItr != input.end(); ++inputItr)
	{
		ASSERT_EQ(map[inputItr->first], inputItr->second);
	}

	//ASSERT_EQ(val, actual);
	//ASSERT_EQ(WorldPoint(2, 2), WorldPoint(1, 1) + WorldVector(1, 1));
}

