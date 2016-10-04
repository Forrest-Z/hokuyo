#ifndef _BOB_CONTROL_CURVE_TO_LINE_CONTROLLER_H_
#define _BOB_CONTROL_CURVE_TO_LINE_CONTROLLER_H_

#include <bob_control/icontroller.h>
#include <bob_control/curve_heading.h>
#include <bob_control/pid_controller.h>

#include <bob_toolbox/circular_direction.h>

#include <bob_toolbox/low_pass_filter.h>
#include <bob_toolbox/velocity2d.h>
#include <bob_config/config.h>

namespace bob
{
		
	//! A controller to keep the robot rotating to the desired heading.
	class CurveToLineController : public IController
	{
		public:
			CurveToLineController(Line line, float radius, CircularDirection direction, float linearVelocity) :
				curveHeading(line, radius),
				pidController(7.0, 0.0, 0.0, 1.0 / desiredFrequency(), 0.4, -0.4),
				lowPassFilter(0.1, 1.0 / desiredFrequency())
			{
				float dirFactor = (direction == CounterClockwise ? 1.0 : -1.0);
                                referenceVel = Velocity2D(linearVelocity, dirFactor * (linearVelocity / radius));
			}

			
			virtual Velocity2D nextCommand(const ISensorHandle& sensorHandle);
			
		
		private:

			//! Used to calculate desired heading according to the line and robot current heading.
			CurveHeading curveHeading;

			PIDController pidController;

			Velocity2D referenceVel;

			LowPassFilter lowPassFilter;
	};

}

#endif
