#include <gtest/gtest.h>

#include <bob_coverage/discrete_area.h>
#include <bob_coverage/binary_map_area.h>

#include <bob_grid_map/map_location.h>

#include <bob_toolbox/dimension2d.h>

using namespace bob;

TEST (AreaTest, DiscreteAreaTest)
{
	DiscreteArea area(0.05);
	
	MapLocation inside(3, 5);
	MapLocation outside(2, 6);
	
	area.insert(inside);

	ASSERT_EQ(area.contains(inside), true);
	ASSERT_EQ(area.contains(outside), false);
}

TEST (AreaTest, BasicBinaryMapAreaTest)
{
	BinaryMapArea area(0.05);

	MapMetadata size;
	size.bottomLeftCorner = MapLocation(0, 0);
	size.bounds = Dimension2D<int>(200, 200);

	area.resizeClear(size);
	
	MapLocation inside(3, 5);
	MapLocation outside(2, 6);

	MapMetadata setData = area.getMapMetadata();
	
	area.insert(inside);

	ASSERT_EQ(area.contains(inside), true);
	ASSERT_EQ(area.contains(outside), false);
}
