#ifndef _BOB_CONTROL_CURVE_TOOLS_H_
#define _BOB_CONTROL_CURVE_TOOLS_H_

#include <bob_control/curve_planner.h>

namespace bob
{
	
	class Line;
	class Pose2D;
	class SimpleCommander;
	class ITransformHandle;
	
	CurvePlanner::CurveSpecifics adjustAndGetCurve(Line toCurveTo, Pose2D robotPose, SimpleCommander& simpleCommander, const ITransformHandle& transformHandle);
	
}

#endif
