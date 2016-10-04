#include <bob_control/curve_planner.h>

#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_visualization/marker_types.h>
#include <bob_toolbox/angles.h>
#include <bob_toolbox/line_geometry.h>
#include <bob_toolbox/sign.h>
#include <bob_visualization/visualization.h>

#include <bob_visualization/visualization.h>

namespace bob
{

	CurvePlanner::CurveSpecifics CurvePlanner::getCurve()
	{
		float angleDiff = normalizeAngle(robotPose.theta - tarLine.getAngle());
		float shortestDistanceToLine = perpendicularDistance(tarLine, robotPose);
		
		float distanceToLine = shortestDistanceToLine / fabs(sin(angleDiff));
		float radius = fabs(distanceToLine / tan(angleDiff / 2));
		
		CircularDirection turningDir = getTurningDirection();
	
		WorldPoint center = robotPose + radius * unitVector(robotPose.theta - sign(angleDiff) * M_PI / 2);

		visualizer->visualize("circle", MarkerCircle(center, radius), greenMarkerStyle());
		
		return CurveSpecifics(radius, turningDir, getCurveVel(radius), tarLine);
	}

 	float CurvePlanner::getCurveVel(float radius)
	{
		float maxLinearVel = 0.1;
		float standardAngularVel = 0.66;

		if (radius >= 0.1515)
		{
			return maxLinearVel;	
		}
		else
		{
			return standardAngularVel * radius;
			// Angular velocity = linear / radius
		}
	}

	CircularDirection CurvePlanner::getTurningDirection()
	{
		float angleDiff = normalizeAngle(robotPose.theta - tarLine.getAngle());

		if (angleDiff < 0)
			return CounterClockwise;

		return Clockwise;
	}

}

