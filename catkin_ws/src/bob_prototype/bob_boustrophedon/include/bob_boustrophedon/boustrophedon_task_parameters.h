#ifndef _BOB_BOUSTROPHEDON_BOUSTROPHEDON_TASK_PARAMETERS_H_
#define _BOB_BOUSTROPHEDON_BOUSTROPHEDON_TASK_PARAMETERS_H_

#include <bob_toolbox/world_rectangle.h>
#include <bob_toolbox/world_point.h>
#include <bob_coverage/discrete_area.h>

#include <bob_boustrophedon/boustrophedon_subtask_executor.h>
#include <bob_toolbox/subtask.h>

namespace bob
{

	//! The Boustrophedon parameters associated with an area to be covered.
	struct BoustrophedonTaskParameters
	{

		public:

			//! Constructor which calculates all the parameters of the task
			BoustrophedonTaskParameters(const DiscreteArea& area, WorldPoint robotPosition);

			//! Empty constructor

			//! Provided in order to support late initialization
			BoustrophedonTaskParameters()
			{}

			//! Angle pointing into the rectangle from the start point, along the longer side
			float majorAngle;

			//! The length of the longer side of the rectangle
			float majorLength;

			//! Angle pointing into the rectangle from the start point, along the shorter side
			float normalAngle;		

			//! The length of the shorter side of the rectangle
			float normalLength;

			//! The first subtask to execute in the Boustrophedon task.
			//! All the successive tasks will be created by modifying this subtask through .update()			
			Subtask firstSubtask;

		private:

			//! Determines the first subtask to be executed by the robot when executing the
			//! boustrophedon task.
			void determineFirstSubtask(WorldPoint startCorner, const DiscreteArea& area);

			//! A comparator that is used to determine if location is a better start point than another.
			//! This class is used within std::min_element as the comparator. 
			bool betterStartPoint(MapLocation firstLocation, MapLocation secondLocation, WorldPoint origin, WorldVector majorVector, WorldVector normalVector, const DiscreteArea& area) const;

			//! Determines the majorAngle and normalAngle based on the rectangle vertices and the rectangle corner.
			void determineAngles(const std::vector<WorldPoint> rectangleVertices, WorldPoint chosenCorner);

			//! Determines the majorLength and normalLength based on the rectangle. Also sets rectangleMajorAngle
			//! which is used later to determine the angles.
			void determineLengths(const WorldRectangle& rectangle);

			//! The major angle for the bounding rectangle
			float rectangleMajorAngle;

			//! The bounding rectangle calculated based on the area
			WorldRectangle rectangle;

	};

}

#endif
