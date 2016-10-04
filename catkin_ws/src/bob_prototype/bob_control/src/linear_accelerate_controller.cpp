#include <bob_control/linear_accelerate_controller.h>
#include <bob_toolbox/geometry.h>
#include <bob_toolbox/sign.h>

#include <bob_toolbox/pose2d.h>
#include <bob_toolbox/velocity2d.h>
#include <bob_sensor/isensor_handle.h>
#include <bob_sensor/iodometry_handle.h>
#include <bob_sensor/itransform_handle.h>

namespace bob
{

	Velocity2D LinearAccelerateController::nextCommand(const ISensorHandle& sensorHandle)
	{	
		Velocity2D robotVelocity = sensorHandle.getOdometryHandle().getRobotVel();
		Pose2D odomPose = sensorHandle.getTransformHandle().getOdomPose();
	
		if (state != Finished)
		{	
			if (state == Initializing)
			{
				lastCommand = robotVelocity.x;	
				initialAccelDir = targetVel - lastCommand;
				state = Accelerating;
			}

			float distanceRemaining = distance - diagonalDistance<WorldPoint>(odomPose, startPoint);
			
			// Adjust acceleration according to distance remaining.
			float acceleration = (targetVel * targetVel - lastCommand * lastCommand) / (2 * distanceRemaining);
			
			// Limit acceleration
			acceleration = sign(acceleration) * std::min(fabs(acceleration), 0.4);
	
			if (acceleration * initialAccelDir < 0)
			{
				state = Finished;
			}
			
			lastCommand = lastCommand + acceleration * period;

			return Velocity2D(lastCommand, 0); 
		}

		return Velocity2D(targetVel, 0);
	}

}

