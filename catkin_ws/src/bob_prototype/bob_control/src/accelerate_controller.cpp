#include <bob_control/accelerate_controller.h>

#include <bob_toolbox/logging.h>

namespace bob
{
	Velocity2D AccelerateController::nextCommand(const ISensorHandle& sensorHandle)
	{
		float period = 1.0 / desiredFrequency(); 		

		targetVel.x = targetVel.x > 0.0 ? std::min(targetVel.x, linearSpeedLimit) : std::max(targetVel.x, -linearSpeedLimit);	
		targetVel.w = targetVel.w > 0.0 ? std::min(targetVel.w, angularSpeedLimit) : std::max(targetVel.w, -angularSpeedLimit);
		
		Velocity2D cmdVel = targetVel;
		
		Velocity2D robotVelocity = sensorHandle.getOdometryHandle().getRobotVel();

		if (robotVelocity != targetVel)		
		{
			float vInc, wInc, maxVInc, maxWInc; // current linear and angular increment and maximum velocity increment
			
			// Linear pre processing
			vInc = targetVel.x - robotVelocity.x;
			if (robotVelocity.x * targetVel.x < 0.0)
			{	
				maxVInc = fabs(linearDecel * period);
			}
			else
			{
				maxVInc = fabs(((vInc * targetVel.x > 0.0) ? linearAccel : linearDecel) * period); 
			}

			// Angular pre processing
			wInc = targetVel.w - robotVelocity.w;
			if (robotVelocity.w * targetVel.w < 0.0)
                        {
                                maxWInc = angularDecel * period;
                        }
                        else
                        {
                                maxWInc = ((wInc * targetVel.w > 0.0) ? angularAccel : angularDecel) * period; 
                        }


			// Calculate and normalise vectors A (desired velocity increment) and B (maximum velocity increment),
			// where v acts as coordinate x and w as coordinate y; the sign of the angle from A to B determines
			// which velocity (v or w) must be overconstrained to keep the direction provided as command
			float MA = sqrt(vInc * vInc + wInc * wInc);
			float MB = sqrt(maxVInc * maxVInc + maxWInc * maxWInc);

			float Av = fabs(vInc) / MA;
			float Aw = fabs(wInc) / MA;
			float Bv = maxVInc / MB;
			float Bw = maxWInc / MB;
			
			float theta = atan2(Bw, Bv) - atan2(Aw, Av);
			
			if (theta < 0)
			{
				// overconstrain linear velocity
				maxVInc = (maxWInc * fabs(vInc)) / fabs(wInc);
			}
			else
			{
				// overconstrain angular velocity
				maxWInc = (maxVInc * fabs(wInc)) / fabs(vInc);
			}
			

			if (fabs(vInc) > maxVInc)
			{
				cmdVel.x = robotVelocity.x + sign(vInc) * maxVInc;
			}

			if (fabs(wInc) > maxWInc)
			{
				cmdVel.w = robotVelocity.w + sign(wInc) * maxWInc;
			}
			
			//LOG_CONTROL("cmdVel: (%f, %f)", cmdVel.x, cmdVel.w); 
			//LOG_CONTROL("current: (%f, %f)", robotVelocity.x, robotVelocity.w);
		}
	
		return cmdVel;	
	}
	

}

