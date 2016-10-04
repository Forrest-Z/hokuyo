#include <bob_control/velocity_smoother.h>



namespace bob {

	VelocitySmoother::VelocitySmoother(IController& controller, RobotControllerConfig& config): controller(controller)
	{
		speedLimV = config.speedLimitV();
		accelLimV = config.accelerationLimitV();
		decelLimV = config.decelerationLimitV();
		
		speedLimW = config.speedLimitW();
		accelLimW = config.accelerationLimitW();
		decelLimW = config.decelerationLimitW();
	}
	
	Velocity2D VelocitySmoother::nextCommand(const ISensorHandle& sensorHandle)
	{
		float period = 1.0 / desiredFrequency();		
		
		Velocity2D targetVel = controller.nextCommand(robotState);
		targetVel.x = targetVel.x > 0.0 ? std::min(targetVel.x, speedLimV) : std::max(targetVel.x, -speedLimV);		
		targetVel.w = targetVel.w > 0.0 ? std::min(targetVel.w, speedLimW) : std::max(targetVel.w, -speedLimW);
		
		Velocity2D cmdVel = targetVel;
		
		if (robotState.velocity != targetVel)		
		{
			float vInc, wInc, maxVInc, maxWInc; // current linear and angular increment and maximum velocity increment
			
			// Linear pre processing
			vInc = targetVel.x - robotState.velocity.x;
			if (robotState.velocity.x * targetVel.x < 0.0)
			{	
				maxVInc = fabs(decelLimV * period);
			}
			else
			{
				maxVInc = fabs(((vInc * targetVel.x > 0.0) ? accelLimV : decelLimV) * period); 
			}

			// Angular pre processing
			wInc = targetVel.w - robotState.velocity.w;
			if (robotState.velocity.w * targetVel.w < 0.0)
                        {
                                maxWInc = decelLimW * period;
                        }
                        else
                        {
                                maxWInc = ((wInc * targetVel.w > 0.0) ? accelLimW : decelLimW) * period; 
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
				cmdVel.x = robotState.velocity.x + sign(vInc) * maxVInc;
			}

			if (fabs(wInc) > maxWInc)
			{
				cmdVel.w = robotState.velocity.w + sign(wInc) * maxWInc;
			}
			 
		}
		return cmdVel;
	}


}
