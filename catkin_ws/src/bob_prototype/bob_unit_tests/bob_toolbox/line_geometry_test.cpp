#include <gtest/gtest.h>
#include <bob_toolbox/line_geometry.h>

using namespace bob;

TEST (PerpendicularBoundsTest, simple)
{
	LineSegment segment(WorldPoint(4, 0), WorldPoint(10, 0));

	ASSERT_EQ(OutsideFirst, withinPerpendicularBounds(segment, WorldPoint(0, 0)));
	ASSERT_EQ(OutsideFirst, withinPerpendicularBounds(segment, WorldPoint(3, -2)));
	ASSERT_EQ(Inside, withinPerpendicularBounds(segment, WorldPoint(4, 6)));
	ASSERT_EQ(Inside, withinPerpendicularBounds(segment, WorldPoint(6, -10)));
	ASSERT_EQ(Inside, withinPerpendicularBounds(segment, WorldPoint(10, -6)));
	ASSERT_EQ(OutsideSecond, withinPerpendicularBounds(segment, WorldPoint(16, 6)));
}

TEST (PerpendicularBoundsTest, shifted)
{
	LineSegment segment(WorldPoint(-2, -2), WorldPoint(2, 2));

	ASSERT_EQ(OutsideFirst, withinPerpendicularBounds(segment, WorldPoint(-500, 0)));
	ASSERT_EQ(OutsideFirst, withinPerpendicularBounds(segment, WorldPoint(-3, -3)));
	ASSERT_EQ(Inside, withinPerpendicularBounds(segment, WorldPoint(-2, 5)));
	ASSERT_EQ(Inside, withinPerpendicularBounds(segment, WorldPoint(-2, 6)));
	ASSERT_EQ(Inside, withinPerpendicularBounds(segment, WorldPoint(3, -3)));
	ASSERT_EQ(OutsideSecond, withinPerpendicularBounds(segment, WorldPoint(6, 1)));
}

TEST (PerpendicularBoundsTest, shiftedReference)
{
	LineSegment segment(WorldPoint(2, 2), WorldPoint(-2, -2));

	ASSERT_EQ(OutsideSecond, withinPerpendicularBounds(segment, WorldPoint(-500, 0)));
	ASSERT_EQ(OutsideSecond, withinPerpendicularBounds(segment, WorldPoint(-3, -3)));
	ASSERT_EQ(Inside, withinPerpendicularBounds(segment, WorldPoint(-2, 5)));
	ASSERT_EQ(Inside, withinPerpendicularBounds(segment, WorldPoint(-2, 6)));
	ASSERT_EQ(Inside, withinPerpendicularBounds(segment, WorldPoint(3, -3)));
	ASSERT_EQ(OutsideFirst, withinPerpendicularBounds(segment, WorldPoint(6, 1)));
}

// Testing if the intersections were correctly detected
TEST (LineSegmentIntersection, intersectionTest)
{
	LineSegment intersectWithNone1(WorldPoint(0, 1), WorldPoint(1, 2));
	LineSegment intersectWithNone2(WorldPoint(3, 0), WorldPoint(3, 1));

	LineSegment segment1(WorldPoint(0, 4), WorldPoint(-1, -1));
	LineSegment segment2(WorldPoint(-1, 3), WorldPoint(2, 2));
	LineSegment segment3(WorldPoint(-2, -1), WorldPoint(2, 2));

	WorldPoint placeholder;
	ASSERT_EQ(false, lineSegmentIntersection(intersectWithNone1, segment1, placeholder));
	ASSERT_EQ(false, lineSegmentIntersection(intersectWithNone1, segment2, placeholder));
	ASSERT_EQ(false, lineSegmentIntersection(intersectWithNone1, segment3, placeholder));
	ASSERT_EQ(false, lineSegmentIntersection(intersectWithNone2, segment1, placeholder));
	ASSERT_EQ(false, lineSegmentIntersection(intersectWithNone2, segment2, placeholder));
	ASSERT_EQ(false, lineSegmentIntersection(intersectWithNone2, segment3, placeholder));

	ASSERT_EQ(true, lineSegmentIntersection(segment1, segment2, placeholder));
	ASSERT_EQ(true, lineSegmentIntersection(segment2, segment3, placeholder));
	ASSERT_EQ(true, lineSegmentIntersection(segment1, segment3, placeholder));
}

TEST (LineSegmentIntersection, validIntersectionPoint)
{
	LineSegment segment1(WorldPoint(2, 1), WorldPoint(0, 2));
	LineSegment segment2(WorldPoint(1, 2), WorldPoint(1, -1));
	LineSegment segment3(WorldPoint(0, 1), WorldPoint(2, 1));

	WorldPoint placeholder;
	ASSERT_EQ(true, lineSegmentIntersection(segment1, segment2, placeholder));
	ASSERT_EQ(WorldPoint(1, 1.5), placeholder);

	ASSERT_EQ(true, lineSegmentIntersection(segment2, segment3, placeholder));
	ASSERT_EQ(WorldPoint(1, 1), placeholder);

	ASSERT_EQ(true, lineSegmentIntersection(segment1, segment3, placeholder));
	ASSERT_EQ(WorldPoint(2, 1), placeholder);
}

TEST (ClosestLineBetweenSegments, zeroLengthForIntersection)
{
	LineSegment segment1(WorldPoint(2, 1), WorldPoint(0, 2));
	LineSegment segment2(WorldPoint(1, 2), WorldPoint(1, -1));
	LineSegment segment3(WorldPoint(0, 1), WorldPoint(2, 1));

	SegmentLengthPair result = shortestLineBetweenSegments(segment1, segment2);
	ASSERT_EQ(0, result.length);
	ASSERT_EQ(LineSegment(WorldPoint(1, 1.5), WorldPoint(1, 1.5)), result.segment);

	result = shortestLineBetweenSegments(segment2, segment3);
	ASSERT_EQ(0, result.length);
	ASSERT_EQ(LineSegment(WorldPoint(1, 1), WorldPoint(1, 1)), result.segment);

	result = shortestLineBetweenSegments(segment1, segment3);
	ASSERT_EQ(0, result.length);
	ASSERT_EQ(LineSegment(WorldPoint(2, 1), WorldPoint(2, 1)), result.segment);
}

TEST (ClosestLineBetweenSegments, endPointSegmentTest)
{
	// Define three line segments for which the shortest distance in each
	// case is the line endpoints
	LineSegment segment1(WorldPoint(-1, -1), WorldPoint(1, -1));
	LineSegment segment2(WorldPoint(2, 0), WorldPoint(1, 2));
	LineSegment segment3(WorldPoint(-1, 2), WorldPoint(-1, 0));

	SegmentLengthPair result = shortestLineBetweenSegments(segment1, segment2);
	ASSERT_NEAR(sqrt(2), result.length, 0.001);
	ASSERT_EQ(LineSegment(WorldPoint(1, -1), WorldPoint(2, 0)), result.segment);

	result = shortestLineBetweenSegments(segment2, segment3);
	ASSERT_NEAR(2, result.length, 0.001);
	ASSERT_EQ(LineSegment(WorldPoint(1, 2), WorldPoint(-1, 2)), result.segment);

	result = shortestLineBetweenSegments(segment1, segment3);
	ASSERT_NEAR(1, result.length, 0.001);
	ASSERT_EQ(LineSegment(WorldPoint(-1, 0), WorldPoint(-1, -1)), result.segment);
}
