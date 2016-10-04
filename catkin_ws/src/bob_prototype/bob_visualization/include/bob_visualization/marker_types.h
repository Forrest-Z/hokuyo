#ifndef _BOB_TOOLBOX_MARKER_TYPES_H_
#define _BOB_TOOLBOX_MARKER_TYPES_H_

#include <bob_toolbox/world_point.h>
#include <bob_toolbox/pose2d.h>
#include <bob_toolbox/dimension2d.h>

#include <vector>

namespace bob
{

	//! \brief A visualizable line consisting of a series of points connected by segments.
	struct MarkerLine
	{

		//! Construct a MarkerLine and initialize the data using a container
		//! \param data A container containing points which define the MarkerLine
		template <typename DataContainer>
			MarkerLine(DataContainer data) :
				data(data.begin(), data.end()) 
		{}

		//! \brief Construct an empty MarkerLine
		MarkerLine()
		{}

		//! The points defining the line
		std::vector<WorldPoint> data;	

	};

	//! \brief Similar to MarkerLine, except that alternating line segments are not drawn.
	//! Produces a "dotted line." Can be useful in producing certain kinds of visualizations.
	struct MarkerLineList
	{

		//! \brief Construct a MarkerLineList instance
		//! \param data The points on the line
		MarkerLineList(std::vector<WorldPoint> data) : 
			data(data) 
		{}

		//! The points on the line
		std::vector<WorldPoint> data;

	};

	//! \brief A visualizable circle. The circle is not filled-in. It is just the outline.
	struct MarkerCircle
	{

		//! \brief Construct a MarkerCircle instance
		//! \param center The center of the circle
		//! \param radius The radius of the circle
		MarkerCircle(WorldPoint center, float radius) : 
			center(center), 
			radius(radius) 
		{}

		//! Center of circle
		WorldPoint center;

		//! Radius of circle
		float radius;

	};

	//! \brief A visualizable group of points.
	struct MarkerPoints
	{

		//! \brief Construct a MarkerPoints instance
		//! \param data The location of the points
		template <typename DataContainer>
			MarkerPoints(DataContainer data) :
				data(data.begin(), data.end())
		{}

		//! \brief Construct an empty instance of MarkerPoints
		MarkerPoints()
		{}

		//! The locations of the points to be visualized
		std::vector<WorldPoint> data;

	};

	//! \brief A visualizable arrow.
	struct MarkerArrow
	{

		//! \brief Construct a MarkerArrow instance
		//! \param pose The pose defining the position and orientation of the arrow
		//! \param length Defines the length of the arrow in meters
		MarkerArrow(Pose2D pose, float length = 0.3) : 
			from(pose), 
			to(WorldPoint(pose.x + length * cos(pose.theta), pose.y + length * sin(pose.theta))) 
		{}		

		//! The arrow base
		WorldPoint from;

		//! The arrow head
		WorldPoint to;

	};

	//! \brief A visualization consisting of many squares placed in the world.
	//! The squares are aligned with the x and y axis.
	struct MarkerSquares
	{

		//! \brief Construct an instance of MarkerSquares
		//! \param data The data used to initialize the location of centers of the squares
		//! \param width The width of the sides of each square
		template <typename DataContainer>
			MarkerSquares(DataContainer data, float width = 0.05) :
				data(data.begin(), data.end()),
				width(width)
		{}

		//! \brief Construct an empty instance of MarkerSquares
		//! \param width The width of the sides of each square
		MarkerSquares(float width = 0.05) :
		width(width)
		{}

		//! The locations of the center of each square
		std::vector<WorldPoint> data;

		//! The width of the sides of each square
		float width;

	};

	//! \brief A rotated and stretched circle. 
	struct MarkerEllipse
	{

		//! \brief Construct an instance of MarkerEllipse
		//! \param pose The pose of the ellipse, specifying the origin and rotation
		//! \param dimensionds Specifies the width and height of the ellipse. Note, width refers to the
		//! pose.theta direction while height refers to the (pose.theta + pi / 2) direction
		MarkerEllipse(Pose2D pose, Dimension2D<float> dimensions) :
		pose(pose),
		dimensions(dimensions)
		{}

		//! The pose of the ellipse, specifying the origin and rotation
		Pose2D pose;

		//! The width and height of the ellipse
		Dimension2D<float> dimensions;
		

	};

}

#endif
