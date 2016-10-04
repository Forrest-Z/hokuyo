#ifndef _BOB_BOUSTROPHEDON_BOUSTROPHEDON_CURVE_SPACES_H_
#define _BOB_BOUSTROPHEDON_BOUSTROPHEDON_CURVE_SPACES_H_

#include <bob_toolbox/map_half_space.h>
#include <bob_boustrophedon/boustrophedon_subtask_executor.h>
#include <bob_toolbox/subtask.h>

namespace bob
{

	//! Represents the MapHalfSpaces for a given Subtask.

	//! These half-spaces represent the relevant regions along the action that signify the state of	
	//! the task completion.
	struct BoustrophedonCurveSpaces	
	{

		public:

			//! Constructs a BoustrophedonCurveSpaces object based on an action. A failLength
			//! is also required, which determins the maximum deviation from the tracked line 
			//! before the action has failed.
			BoustrophedonCurveSpaces(Subtask action, float failLength);

			//! Reached the end of the action and now passing the end of the rectangle (the goal)
			MapHalfSpace passingGoal;

			//! Space representing the area at which the next line is abandoned because the robot
			//! has deviated too far from the tracking line.
			MapHalfSpace skipLine;

			//! The space that is used to represent when the robot has returned to the tracking line
			//! after deviating from it to get around an obstacle.
			MapHalfSpace backToLine;

			MapHalfSpace backToStart;

	};

}

#endif
