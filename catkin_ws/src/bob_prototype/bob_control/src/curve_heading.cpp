#include <bob_control/curve_heading.h>

#include <bob_toolbox/line_geometry.h>
#include <bob_toolbox/sign.h>
#include <bob_toolbox/angles.h>

namespace bob
{

	float CurveHeading::desiredHeading(const WorldPoint& robotPoint)
	{
		// Current distance to target line
		float distanceToLine = perpendicularDistanceWithSign(line, robotPoint);

		// Cosine value of the desirge relative to line angle
		float trangleRatio = fabs(distanceToLine) > radius ? 0.0 : (radius - fabs(distanceToLine)) / radius;
		trangleRatio = sign(trangleRatio) * std::min(1.0, fabs(trangleRatio));

		// Get desired heading in world frame
		float lineAngleHeadingDiff = sign(distanceToLine) * acos(trangleRatio);

		return normalizeAngle(line.getAngle() - lineAngleHeadingDiff);

	}

}

