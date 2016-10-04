#include <bob_control/curve_tools.h>
#include <bob_control/simple_commander.h>
#include <bob_control/commands/heading_rotation_command.h>

#include <bob_toolbox/line_geometry.h>
#include <bob_toolbox/geometry.h>
#include <bob_toolbox/angles.h>

#include <bob_sensor/itransform_handle.h>
#include <bob_toolbox/logging.h>

namespace bob
{

	CurvePlanner::CurveSpecifics adjustAndGetCurve(Line toCurveTo, Pose2D robotPose, SimpleCommander& simpleCommander, const ITransformHandle& transformHandle)
	{
		WorldPoint closestPoint = closestPointOnLine(toCurveTo, robotPose);
		double angleToLine = rayAngle<WorldPoint>(robotPose, closestPoint);

		if (fabs(normalizeAngle(robotPose.theta  - angleToLine)) >= M_PI / 2)
		{
			// If heading away from the line, do heading rotation turn to the line
			LOG_CONTROL("heading rotation to line");
			HeadingRotationCommand command(angleToLine);
			simpleCommander.execute(command);
		}	
			
		robotPose = transformHandle.getLocalizedPose();

		CurvePlanner curvePlanner(toCurveTo, robotPose);

		return curvePlanner.getCurve();
	}


}

