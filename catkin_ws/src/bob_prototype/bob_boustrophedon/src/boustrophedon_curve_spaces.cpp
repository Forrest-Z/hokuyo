#include <bob_boustrophedon/boustrophedon_curve_spaces.h>

#include <bob_toolbox/angles.h>
#include <bob_toolbox/easy_print.h>

namespace bob
{

	BoustrophedonCurveSpaces::BoustrophedonCurveSpaces(Subtask action, float failLength) : 
		passingGoal(WorldPoint(), 0),
		skipLine(WorldPoint(), 0),
		backToLine(WorldPoint(), 0),
		backToStart(WorldPoint(), 0)
	{
		float travellingAngle = action.toTrack.getAngle();
		float curveToAngle = action.toCurveTo.getAngle();

		// Determine a bunch of properties based on the direction we are going to turn
		float angularDiff = normalizeAngle(curveToAngle - travellingAngle);	
		float failNormalAngle;
		if (angularDiff > 0)
		{
			// Left-hand turn at end of action
			failNormalAngle = travellingAngle + M_PI / 2;
		}
		else
		{
			// Right-hand turn at end of action
			failNormalAngle = travellingAngle - M_PI / 2;
		}

		// The goal area lies at the end of the line, on the boustrophedon rectangle
		WorldPoint endOfRectangle;
		action.toTrack.intersection(action.toCurveTo, endOfRectangle);
		passingGoal = MapHalfSpace(endOfRectangle, travellingAngle);

		// This fail area is the next boustrophedon line. If we wall follow into this line, then we fail and switch to that line
		WorldPoint failPoint = action.toTrack.origin + (failLength * unitVector(failNormalAngle));
		skipLine = MapHalfSpace(failPoint, failNormalAngle);

		backToLine = MapHalfSpace(action.toTrack.origin, failNormalAngle + M_PI);
	
		backToStart = MapHalfSpace(action.toTrack.origin + 0.1 * unitVector(travellingAngle + M_PI), travellingAngle + M_PI);
	}

}

