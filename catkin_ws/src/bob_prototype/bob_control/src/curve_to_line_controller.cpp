#include <bob_control/curve_to_line_controller.h>
#include <bob_toolbox/angles.h>
#include <bob_toolbox/pose2d.h>

#include <bob_sensor/itransform_handle.h>
#include <bob_sensor/isensor_handle.h>

namespace bob
{
	Velocity2D CurveToLineController::nextCommand(const ISensorHandle& sensorHandle)
	{
		Pose2D pose = sensorHandle.getTransformHandle().getLocalizedPose();

		float desiredHeading = curveHeading.desiredHeading(pose);

		float headingError = normalizeAngle(desiredHeading - pose.theta);
		
		float angularVelIncrement = pidController.nextCommand(headingError);
		
		return Velocity2D(referenceVel.x, lowPassFilter.filterDataPoint(referenceVel.w + angularVelIncrement));	
			
	}

}

