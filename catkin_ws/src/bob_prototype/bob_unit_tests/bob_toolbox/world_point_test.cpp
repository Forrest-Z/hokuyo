#include <gtest/gtest.h>
#include <bob_toolbox/world_point.h>

using namespace bob;

// WorldPoint + WorldVector Test
TEST (PointAndVector, add) 
{ 
    ASSERT_EQ (WorldPoint(2, 2), WorldPoint(1, 1) + WorldVector(1, 1));
    ASSERT_EQ (WorldPoint(1001, 205), WorldPoint(1000, 200) + WorldVector(1, 5));
    ASSERT_EQ (WorldPoint(-999, 239), WorldPoint(-1000, 234) + WorldVector(1, 5));
    ASSERT_EQ (WorldPoint(1950, 3201), WorldPoint(-50, -10) + WorldVector(2000,3211));
}

TEST (PointAndVector, subtract) 
{
    ASSERT_EQ (WorldPoint(0, 0), WorldPoint(1, 1) - WorldVector(1, 1));
    ASSERT_EQ (WorldPoint(999, 195), WorldPoint(1000, 200) - WorldVector(1, 5));
    ASSERT_EQ (WorldPoint(-1001, 229), WorldPoint(-1000, 234) - WorldVector(1, 5));
    ASSERT_EQ (WorldPoint(-2000, -3201), WorldPoint(0, 10) - WorldVector(2000,3211));
}


