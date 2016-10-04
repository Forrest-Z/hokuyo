#include <bob_boustrophedon/boustrophedon_task_parameters.h>

#include <bob_toolbox/simple_patterns.h>
#include <bob_toolbox/angles.h>
#include <bob_toolbox/point_geometry.h>
#include <bob_toolbox/easy_print.h>
#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_toolbox/geometry.h>
#include <bob_coverage/area_tools.h>
#include <bob_visualization/visualization.h>
#include <boost/bind.hpp>

#include <vector>

namespace bob
{
	
	BoustrophedonTaskParameters::BoustrophedonTaskParameters(const DiscreteArea& area, WorldPoint robotPosition)
	{
		rectangle = minAreaBoundingBox(area);
		std::vector<WorldPoint> vertices = rectangle.getPoints();
		WorldPoint closestCorner = closestPointTo(vertices, robotPosition);

		visualizer->visualize("covering_rectangle", MarkerLine(vertices), greenMarkerStyle());

		// Note: These functions must occur in this order because they rely on member
		// 	 variables that are set by the preceding function
		determineLengths(rectangle);
		determineAngles(vertices, closestCorner);
		determineFirstSubtask(closestCorner, area);
	}

	void BoustrophedonTaskParameters::determineLengths(const WorldRectangle& rectangle)
	{
		if(rectangle.bounds.height > rectangle.bounds.width)
		{
			majorLength = rectangle.bounds.height;
			normalLength = rectangle.bounds.width;
			rectangleMajorAngle = normalizeAnglePos(M_PI / 2 + rectangle.angle);
		}
		else
		{
			majorLength = rectangle.bounds.width;
			normalLength = rectangle.bounds.height;
			rectangleMajorAngle = normalizeAnglePos(rectangle.angle);
		}
	}

	void BoustrophedonTaskParameters::determineAngles(const std::vector<WorldPoint> rectangleVertices, WorldPoint chosenCorner)
	{
		// Loop over all the vertices
		for(std::vector<WorldPoint>::const_iterator itr = rectangleVertices.begin(); itr != rectangleVertices.end(); ++itr)
		{
			// Ignore chosenCorner in this calculation
			if(*itr == chosenCorner)
				continue;
				
			// Find the angle between the chosen corner and the rectangle vertex
			float angle = rayAngle(chosenCorner, *itr);

			// Normalize the angle because the bounding box may be rotated
			float diffFromOrientation = normalizeAngle(angle - rectangleMajorAngle);
			
			if (approximatelyEqual(diffFromOrientation, 0) || approximatelyEqual(fabs(diffFromOrientation), M_PI))
			{
				// The angle points along or opposite to the rectangleMajorAngle
				majorAngle = angle;
			}
	
			if (approximatelyEqual(diffFromOrientation, M_PI / 2) || approximatelyEqual(diffFromOrientation, -M_PI / 2))
			{
				// The angle is perpendicular to the rectangleMajorAngle
				normalAngle = angle;
			}
		}
	}

	bool BoustrophedonTaskParameters::betterStartPoint(MapLocation firstLocation, MapLocation secondLocation, WorldPoint origin, WorldVector majorVector, WorldVector normalVector, const DiscreteArea& area) const
	{
		//TODO: Make this more efficient. Either compute all the WorldPoints beforehand, or do all the algebra in Map co-ordinates
		// Converting the points
		WorldPoint first = area.mapToWorld(firstLocation);
		WorldPoint second = area.mapToWorld(secondLocation);

		// Vectors from origin to the points
		WorldVector firstVector = first - origin;
		WorldVector secondVector = second - origin;
	
		// The dot product is a scaled projection 
		// First project along normal vector because that is more important
		float firstDotProduct = dotProduct(firstVector, normalVector);
		float secondDotProduct = dotProduct(secondVector, normalVector);

		// If they are about the same along normal axis, swith the major axis
		if (approximatelyEqual(firstDotProduct, secondDotProduct))
		{
			// Project along major axis
			firstDotProduct = dotProduct(firstVector, majorVector);
			secondDotProduct = dotProduct(secondVector, majorVector);
		}
		
		// The point is better if its projection is smaller (closer to origin)
		return (firstDotProduct < secondDotProduct);
	}

	void BoustrophedonTaskParameters::determineFirstSubtask(WorldPoint startCorner, const DiscreteArea& area)
	{
		WorldVector majorVector = unitVector(majorAngle);
		WorldVector normalVector = unitVector(normalAngle);
	
		// Get the starting location for the task
		MapLocation startLocation = *std::min_element(area.begin(), area.end(), 
				boost::bind(&BoustrophedonTaskParameters::betterStartPoint, this, _1, _2, startCorner, majorVector, normalVector, area));

		// Convert to world co-ordinates
		WorldPoint startPoint = area.mapToWorld(startLocation);

		// Calculate the location of the next curve
		WorldPoint firstCurveStart = startCorner + majorLength * unitVector(majorAngle);

		Subtask action;
		action.toTrack = Line(startPoint, majorAngle);
		action.toCurveTo = Line(firstCurveStart, normalAngle);
		firstSubtask = action;
	}
}

