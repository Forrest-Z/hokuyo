#include "bob_control/pid_controller.h"

#include <algorithm>

namespace bob
{

	float PIDController::nextCommand(float error, bool needInitialize) 
	{
		if (needInitialize || !initialized)
		{
			errorIntegral = 0;
			preError = error;
			initialized = true;
		}
		
		errorIntegral += error * period;
		float errorDerivative	= (error - preError) / period;	

		float output = kp * error + ki * errorIntegral + kd * errorDerivative;

		preError = error;
		
		output = std::min(output, maxOutput);
		output = std::max(output, minOutput);

		return output;
	}

}
