#include <gtest/gtest.h>
#include <bob_toolbox/angular_range.h>
#include <bob_toolbox/angles.h>

using namespace bob;

// Angles test
TEST (AnglesTest, normalizeAnglePosTest)
{
	float val = -1.435;
	ASSERT_NEAR((val + 2 * M_PI), normalizeAnglePos(val), 0.0001);
	val = 8.6476;
	ASSERT_NEAR((val - 2 * M_PI), normalizeAnglePos(val), 0.0001);
	ASSERT_NEAR(0, normalizeAnglePos(2 * M_PI), 0.0001);
	ASSERT_NEAR(M_PI, normalizeAnglePos(M_PI), 0.0001);
}

TEST (AnglesTest, normalizeAngleTest)
{
	float val = -1.435;
	ASSERT_NEAR(val, normalizeAngle(val), 0.0001);
	val = 11.6537;
	ASSERT_NEAR((val - 4 * M_PI), normalizeAngle(val), 0.0001);
	ASSERT_NEAR(0, normalizeAngle(2 * M_PI), 0.0001);
	ASSERT_NEAR((M_PI + 0.01 - 2 * M_PI), normalizeAngle(M_PI + 0.01), 0.0001);

}

TEST (AnglesTest, shortestAngularDistanceTest)
{
	ASSERT_NEAR(toRadian(360 - 237), shortestAngularDistance(toRadian(237), 0), 0.0001);
	ASSERT_NEAR(toRadian(-70), shortestAngularDistance(toRadian(70), 0), 0.0001);
	ASSERT_NEAR(toRadian(100), shortestAngularDistance(toRadian(-70), toRadian(30)), 0.0001);
	ASSERT_NEAR(toRadian(-120), shortestAngularDistance(toRadian(-70), toRadian(170)), 0.0001);
}


// SimpleAngularRange test
TEST (SimpleAngularRangeTest, isInsideTest)
{
	// Boundary 
	float lower = toRadian(45);
	float upper = toRadian(270);
	SimpleAngularRange range(lower, upper);

	// Normal condition
	ASSERT_TRUE (range.isInside(M_PI / 2));
	ASSERT_TRUE (range.isInside(11 * M_PI / 10));
	ASSERT_FALSE (range.isInside(-M_PI / 4));
	ASSERT_FALSE (range.isInside(17 * M_PI / 10));

	// Boundary condition
	ASSERT_TRUE (range.isInside(lower));
	ASSERT_TRUE (range.isInside(lower + 0.0001)); 
	ASSERT_FALSE (range.isInside(lower - 0.0001));

	ASSERT_TRUE (range.isInside(upper));
	ASSERT_TRUE (range.isInside(upper - 0.0001));
	ASSERT_FALSE (range.isInside(upper + 0.0001));
	
	// Revert boundary
	SimpleAngularRange range2(upper, lower);

	// Normal condition
	ASSERT_FALSE (range2.isInside(M_PI / 2));
	ASSERT_FALSE (range2.isInside(11 * M_PI / 10));
	ASSERT_TRUE (range2.isInside(-M_PI / 4));
	ASSERT_TRUE (range2.isInside(17 * M_PI / 10));

	// Boundary condition
	ASSERT_TRUE (range2.isInside(upper));
	ASSERT_FALSE (range2.isInside(upper - 0.0001));
	ASSERT_TRUE (range2.isInside(upper + 0.0001));

	ASSERT_TRUE (range2.isInside(lower));
	ASSERT_FALSE (range2.isInside(lower + 0.0001)); 
	ASSERT_TRUE (range2.isInside(lower - 0.0001));
}

TEST (SimpleAngularRangeTest, overlapsWithTest)
{
	// Boundary 
	float lower = toRadian(-23);
	float upper = toRadian(26);
	SimpleAngularRange range(lower, upper);
	
	// Normal condition
	ASSERT_TRUE (range.overlapsWith(SimpleAngularRange(23 * M_PI / 180, 64 * M_PI / 180)));
	ASSERT_TRUE (range.overlapsWith(SimpleAngularRange(-80 * M_PI / 180, -18 * M_PI / 180)));
	ASSERT_FALSE (range.overlapsWith(SimpleAngularRange(200 * M_PI / 180, 250 * M_PI / 180)));
	ASSERT_FALSE (range.overlapsWith(SimpleAngularRange(30 * M_PI / 180, 80 * M_PI / 180)));

	// Boundary condition
	ASSERT_TRUE (range.overlapsWith(SimpleAngularRange(26 * M_PI / 180, 64 * M_PI / 180)));
	ASSERT_TRUE (range.overlapsWith(SimpleAngularRange(-50 * M_PI / 180, -23 * M_PI / 180)));
}

TEST (SimpleAngularRangeTest, isFullCircleTest)
{
// TODO
	SimpleAngularRange range1(0, 2 * M_PI);
	SimpleAngularRange range2(0, 0);
	SimpleAngularRange range3(-M_PI, M_PI);

	SimpleAngularRange range4(0, M_PI - 0.001);
	SimpleAngularRange range5(0, M_PI);

	ASSERT_TRUE (range1.isFullCircle());
	ASSERT_TRUE (range2.isFullCircle());
	ASSERT_TRUE (range3.isFullCircle());

	ASSERT_FALSE (range4.isFullCircle());
	ASSERT_FALSE (range5.isFullCircle());
}


// AngularRange test
TEST (AngularRangeTest, addAndGetRangesTestFullCircle)
{
	SimpleAngularRange range1(toRadian(36), toRadian(190));
	SimpleAngularRange range2(toRadian(-180), toRadian(358));
	SimpleAngularRange range3(toRadian(-45), toRadian(90));
	AngularRange angularRange;
	angularRange.add(range1);
	angularRange.add(range2);
	angularRange.add(range3);
		
	// Get ranges test 
	std::list<SimpleAngularRange> data = angularRange.getRanges();
	
	// size
	ASSERT_EQ (1, data.size());

	AngularRange::container::iterator ranges = data.begin();
	ASSERT_TRUE (ranges->isFullCircle());
}

TEST (AngularRangeTest, addAndGetRangesTestNoOverlap)
{	
	SimpleAngularRange range1(toRadian(35 ),toRadian(55));
	SimpleAngularRange range2(toRadian(-83 ), toRadian(-70));
	SimpleAngularRange range3(toRadian(-10), toRadian(21));
	SimpleAngularRange range4(toRadian(23), toRadian(33));
	AngularRange angularRange;
	angularRange.add(range1);
	angularRange.add(range2);
	angularRange.add(range3);
	angularRange.add(range4);

	// Get ranges test
	std::list<SimpleAngularRange> data = angularRange.getRanges();

	// size
	ASSERT_EQ (4, data.size());

	AngularRange::container::iterator ranges = data.begin();
	ASSERT_NEAR (normalizeAnglePos(toRadian(23)), ranges->lower, 0.001);
	ASSERT_NEAR (normalizeAnglePos(toRadian(33)), ranges->upper, 0.001); ++ranges;
	ASSERT_NEAR (normalizeAnglePos(toRadian(35)), ranges->lower, 0.001);
	ASSERT_NEAR (normalizeAnglePos(toRadian(55)), ranges->upper, 0.001); ++ranges;
	ASSERT_NEAR (normalizeAnglePos(toRadian(-83)), ranges->lower, 0.001);
	ASSERT_NEAR (normalizeAnglePos(toRadian(-70)), ranges->upper, 0.001); ++ranges; 
	ASSERT_NEAR (normalizeAnglePos(toRadian(-10)), ranges->lower, 0.001);
	ASSERT_NEAR (normalizeAnglePos(toRadian(21)), ranges->upper, 0.001); 
	
}

TEST (AngularRangeTest, addAndGetRangesTestOverlap)
{
	// Add overlap test
	SimpleAngularRange range1(toRadian(25 ),toRadian(55));
	SimpleAngularRange range2(toRadian(-83 ), toRadian(-70));
	AngularRange angularRange;
	angularRange.add(range1);
	angularRange.add(range2);

	SimpleAngularRange range3(toRadian(-72),toRadian(28));
	angularRange.add(range3);
	
	// Get ranges test 
	std::list<SimpleAngularRange> data = angularRange.getRanges();
	
	// size
	ASSERT_EQ (1, data.size());

	AngularRange::container::iterator ranges = data.begin();
	ASSERT_NEAR (normalizeAnglePos(toRadian(-83)), ranges->lower, 0.001);
	ASSERT_NEAR (normalizeAnglePos(toRadian(55)), ranges->upper, 0.001);
}


TEST (AngularRangeTest, isInsideTest)
{
	SimpleAngularRange range1(toRadian(25), toRadian(55));
	SimpleAngularRange range2(toRadian(-83), toRadian(-70));
	AngularRange angularRange;
	angularRange.add(range1);
	angularRange.add(range2);

	ASSERT_TRUE (angularRange.isInside(toRadian(30)));
	ASSERT_TRUE (angularRange.isInside(toRadian(-77)));
	ASSERT_FALSE (angularRange.isInside(M_PI));
	ASSERT_FALSE (angularRange.isInside(0));

	ASSERT_TRUE (angularRange.isInside(toRadian(25)));
	ASSERT_TRUE (angularRange.isInside(toRadian(55)));
	ASSERT_TRUE (angularRange.isInside(toRadian(-83)));
	ASSERT_TRUE (angularRange.isInside(toRadian(-70)));
	
	// overlap	
	SimpleAngularRange range3(toRadian(-72), toRadian(28));
	angularRange.add(range3);

	ASSERT_TRUE (angularRange.isInside(0));
	ASSERT_FALSE (angularRange.isInside(M_PI));

	ASSERT_TRUE (angularRange.isInside(toRadian(25)));
	ASSERT_TRUE (angularRange.isInside(toRadian(55 )));
	ASSERT_TRUE (angularRange.isInside(toRadian(-83 )));
	ASSERT_TRUE (angularRange.isInside(toRadian(-70 )));
	ASSERT_TRUE (angularRange.isInside(toRadian(-72 )));
	ASSERT_TRUE (angularRange.isInside(toRadian(28 )));
}


TEST (AngularRangeTest, invertTest)
{
	SimpleAngularRange range1(toRadian(25 ), toRadian(55 ));
	SimpleAngularRange range2(toRadian(-83 ), toRadian(-70));
	AngularRange angularRange;
	angularRange.add(range1);
	angularRange.add(range2);

	AngularRange angularRangeInv = angularRange.invert();

	ASSERT_FALSE (angularRangeInv.isInside(toRadian(30 )));
	ASSERT_FALSE (angularRangeInv.isInside(toRadian(-77 )));
	ASSERT_TRUE (angularRangeInv.isInside(M_PI));
	ASSERT_TRUE (angularRangeInv.isInside(0));

	// Boundary remains in range
	ASSERT_TRUE (angularRangeInv.isInside(toRadian(25 )));
	ASSERT_TRUE (angularRangeInv.isInside(toRadian(55 )));
	ASSERT_TRUE (angularRangeInv.isInside(toRadian(-83 )));
	ASSERT_TRUE (angularRangeInv.isInside(toRadian(-70 )));
}

TEST (SimpleAngularRangeTest, limitingTest)
{
	// The range is totally clipped to nothingness
	SimpleAngularRange range1(-0.2, 0.2);
	SimpleAngularRange limit1(0.4, 0.5);
	ASSERT_FALSE(range1.limitBy(limit1));

	// The range is clipped on both sides
	SimpleAngularRange range2(-0.2, 0.4);
	SimpleAngularRange limit2(0.0, 0.2);
	ASSERT_TRUE(range2.limitBy(limit2));
	ASSERT_NEAR(range2.lower, 0.0, 0.001);
	ASSERT_NEAR(range2.upper, 0.2, 0.001);	

	// The range is clipped on the lower side
	SimpleAngularRange range3(-0.2, 0.4);
	SimpleAngularRange limit3(-0.1, 0.5);
	ASSERT_TRUE(range3.limitBy(limit3));
	ASSERT_NEAR(range3.lower, normalizeAnglePos(-0.1), 0.001);
	ASSERT_NEAR(range3.upper, 0.4, 0.001);	

	// The range is clipped on the upper side
	SimpleAngularRange range4(-0.2, 0.4);
	SimpleAngularRange limit4(-0.3, 0.3);
	ASSERT_TRUE(range4.limitBy(limit4));
	ASSERT_NEAR(range4.lower, normalizeAnglePos(-0.2), 0.001);
	ASSERT_NEAR(range4.upper, 0.3, 0.001);	

	// Limit is the exact same as range, should be no change
	SimpleAngularRange range5(-0.2, 0.4);
	SimpleAngularRange limit5(-0.2, 0.4);
	ASSERT_TRUE(range5.limitBy(limit5));
	ASSERT_NEAR(range5.lower, normalizeAnglePos(-0.2), 0.001);
	ASSERT_NEAR(range5.upper, 0.4, 0.001);	
}
