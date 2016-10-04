#ifndef _BOB_TOOLBOX_LINE_GEOMETRY_H_
#define _BOB_TOOLBOX_LINE_GEOMETRY_H_

#include <bob_toolbox/world_point.h>
#include <bob_toolbox/line.h>
#include <bob_toolbox/line_segment.h>

namespace bob
{
	
	//! Represents the three cases of the perpendicular bounds of a line segment
	enum BoundsResult
	{
		//! The point is outside the bounds behind the first point
		OutsideFirst,

		//! The point is outside the bounds in front of the second point
		OutsideSecond,
	
		//! The point is within the perpendicular bounds of the segment
		Inside
	};

	//! Determine where a point lies within the perpendicular bounds of a line segment
	//! Here, the "perpendicular bounds" refers to the area that is defined by extending
	//! lines from the segment endpoints in perpendicular to the lines. This creates
	//! three regions (two half spaces on each end and one rectangular region in between)
	BoundsResult withinPerpendicularBounds(const LineSegment& segment, const WorldPoint& point);

	struct SegmentLengthPair
	{
		LineSegment segment;
		float length;
	};

	//! Determines the shortest line between two line segments where each endpoint of the
	//! resulting line segment lies on the one of the input line segments.
	//! The reason the length is also returned is to make other algorithms more efficient so
	//! it needent be recalculated.
	//! Note: This function also has the useful property that the resulting line segment has its
	//! first point lying on the first line segment and the second point lying on the second line segment.
	//! It is also possible that the lines intersect, resulting in a zero-length line segment
	//! with a single duplicate point that lies on the intersection.
	SegmentLengthPair shortestLineBetweenSegments(const LineSegment& first, const LineSegment& second);

	//! Determines the closest point on a line segment to another point. Makes sure that
	//! the resulting point lies on the inputted segment.
	WorldPoint closestPointOnLineSegment(const LineSegment& segment, const WorldPoint& point);

	//! Determines the closest point on a line to another point.
	WorldPoint closestPointOnLine(const Line& line, const WorldPoint& point);

	//! Calculates the perpendicular distance between a line and a point 
	float perpendicularDistance(const Line& line, const WorldPoint& point);

	//! Calculates the perpendicular distance between a line and a point with sign 
	float perpendicularDistanceWithSign(const Line& line, const WorldPoint& point);
	
	//! Calculates the intersection between two line segments, if one exists
	//
	//! Input:
	//! first - First line segment
	//! second - Second line segment
	//! intersection - Output variable for intersection resulting from the two segments
	//
	//! Returns: True if the intersection point exists (the lines do intersect)
	bool lineSegmentIntersection(const LineSegment& first, const LineSegment& second, WorldPoint& intersection);

	//! Converts LineSegment to Line
	Line segmentToLine(const LineSegment& segment);

}

#endif
