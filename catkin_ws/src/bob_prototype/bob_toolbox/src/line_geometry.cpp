#include <bob_toolbox/line_geometry.h>

#include <bob_toolbox/geometry.h>
#include <bob_toolbox/world_point.h>

#include <assert.h>
#include <cmath>
#include <limits>

namespace bob
{
	bool lineSegmentIntersection(const LineSegment& first, const LineSegment& second, WorldPoint& intersection)
	{
		Line firstLine = segmentToLine(first);
		Line secondLine = segmentToLine(second);

		if(!firstLine.intersection(secondLine, intersection))
			return false;

		BoundsResult firstBoundsInfo = withinPerpendicularBounds(first, intersection);
		BoundsResult secondBoundsInfo = withinPerpendicularBounds(second, intersection);

		if (firstBoundsInfo == Inside && secondBoundsInfo == Inside)
		{
			// Intersection is inside the bounds of both lines (intersection is valid)
			return true;	
		}
		else
		{
			// Intersection is outside of one or both bounds - intersection is invalid
			return false;		
		}
	}


	BoundsResult withinPerpendicularBounds(const LineSegment& segment, const WorldPoint& point)
	{
		WorldVector ray = segment.second - segment.first;

		if (dotProduct(point - segment.second, ray) > 0)
		{
			return OutsideSecond;
		}
		else if (dotProduct(point - segment.first, ray) < 0)
		{
			return OutsideFirst;
		}
		else
		{
			return Inside;
		}
	}

	SegmentLengthPair shortestLineBetweenSegments(const LineSegment& first, const LineSegment& second)
	{
		// There are essentially 5 when considereding two lines (AB, CD):
		// Intersection of AB and CD (if exists)
		// LineSegment from A->CD
		// LineSegment from B->CD
		// LineSegment from C->AB
		// LineSegment from D->AB

		bool firstSegmentIsPoint = (first.first == first.second);
		bool secondSegmentIsPoint = (second.first == second.second);

		if (firstSegmentIsPoint && secondSegmentIsPoint)
		{
			// Degenerate case. Both segments are points.
			SegmentLengthPair result;
			result.segment = LineSegment(first.first, second.first);
			result.length = result.segment.length();	
			return result;
		}

		if (!firstSegmentIsPoint && !secondSegmentIsPoint)
		{
			// If either segment is a point then we cannot calculate the intersection
			// Note: this ignores the case where a point lies on a line segment.
			// That case is tricky to identify.
			// However, it will be caught in the other checks anyway
			WorldPoint intersection;
			if (lineSegmentIntersection(first, second, intersection))
			{
				SegmentLengthPair toReturn;
				toReturn.segment = LineSegment(intersection, intersection);
				toReturn.length = 0;
				// Effectively a zero-length segment
				return toReturn;
			}
		}

		// A helper struct used (hopefully) to simplify things
		struct ShortestSegmentTracker
		{
			ShortestSegmentTracker()
			{
				shortestSoFar.length = std::numeric_limits<float>::max();	
			}

			float update(LineSegment newSegment)
			{
				float segmentLength = newSegment.length();
				if (segmentLength < shortestSoFar.length)
				{
					shortestSoFar.segment =  newSegment;
					shortestSoFar.length = segmentLength;	
				}
			}
			SegmentLengthPair shortestSoFar;
		};
		
		ShortestSegmentTracker tracker;
			
		if (!secondSegmentIsPoint)	
		{
			// A->CD
			tracker.update(LineSegment(first.first, closestPointOnLineSegment(second, first.first))); 

			// B->CD
			tracker.update(LineSegment(first.second, closestPointOnLineSegment(second, first.second))); 
		}

		if (!firstSegmentIsPoint)
		{
			// AB->C
			tracker.update(LineSegment(closestPointOnLineSegment(first, second.first), second.first)); 
		
			// AB->D
			tracker.update(LineSegment(closestPointOnLineSegment(first, second.second), second.second)); 
		}
	
		return tracker.shortestSoFar;
	}

	WorldPoint closestPointOnLineSegment(const LineSegment& segment, const WorldPoint& point)
	{
		Line convertedToLine = segmentToLine(segment);
		WorldPoint candidateResult = closestPointOnLine(convertedToLine, point);	
		
		BoundsResult boundsInfo = withinPerpendicularBounds(segment, candidateResult);

		// We have to test if the candidate point is outside of the line segment
		if (boundsInfo == OutsideSecond)
		{
			return segment.second;
		}
		else if (boundsInfo == OutsideFirst)
		{
			return segment.first;
		}
		else
		{
			// Within the bound of the segment, so we can just return the point
			return candidateResult;
		}
	}

	WorldPoint closestPointOnLine(const Line& line, const WorldPoint& point)
	{
		WorldVector pointToOrigin = point - line.origin;
		float k = dotProduct(line.vector, pointToOrigin) / dotProduct(line.vector, line.vector);
		return line.origin + k * line.vector;
	}

	float perpendicularDistance(const Line& line, const WorldPoint& point)
	{
		// Get distance to closest point
		WorldPoint closestPoint = closestPointOnLine(line, point);
		return diagonalDistance(point, closestPoint);	
	}

	float perpendicularDistanceWithSign(const Line& line, const WorldPoint& point)
	{
		WorldPoint from = line.origin;
		WorldPoint to = line.origin + line.vector;

		// This is actually twice the area of the triangle drawn between from, to and point
		float shoelaceArea = from.x * to.y - from.x * point.y + 
			to.x * point.y - to.x * from.y +
			point.x * from.y - point.x * to.y;

		float triangleBase = diagonalDistance(from, to);

		// A = (1/2)*b*h -> h = 2A/b
		return shoelaceArea/triangleBase;
	}

	Line segmentToLine(const LineSegment& segment)
	{
		// Test for degenerate case of a point (no reasonable line can be produced)
		assert(segment.first != segment.second);

		WorldVector lineVector = segment.second - segment.first;
		return Line(segment.first, lineVector);
	}
}
