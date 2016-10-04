#ifndef _WALL_FOLLOWING_PID_CONTROLLER_H_
#define _WALL_FOLLOWING_PID_CONTROLLER_H_

namespace bob
{

	class PIDController
	{

		public:
			
			//! \brief A constructor.
			//! \param p Proportional gain
			//! \param i Integral gain
			//! \param d Derivative gain
			//! \paran period time interval
			//! \param maximum output
			//! \param minimum output
			PIDController(float kp, float ki, float kd, float period, float maxOutput, float minOutput):
				kp(kp), 
				ki(ki), 
				kd(kd), 
				period(period), 
				maxOutput(maxOutput), 
				minOutput(minOutput),
				initialized(false) 
				{}	

			//! \brief Calculate control command based on error.
			//! \param error Currnet error
			//! \param needInitialize Initialize requirement flag
			float nextCommand(float error, bool needInitialize = false);

		private:

			//! PID tuning gain
			float kp, ki, kd;

			//! Time interval
			float period;
		
			//! Error integral, previous error	
			float errorIntegral, preError; 
	
			float maxOutput, minOutput; 
			
			bool initialized;
	};

}

#endif
