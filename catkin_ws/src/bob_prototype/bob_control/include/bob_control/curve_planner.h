#ifndef _BOB_CONTROL_CURVE_PLANNER_H_
#define _BOB_CONTROL_CURVE_PLANNER_H_

#include <bob_toolbox/circular_direction.h>
#include <bob_toolbox/line.h>
#include <bob_toolbox/pose2d.h>

namespace bob
{

	//! \brief Given a line to curve to and current robot position, calculate a curved path.
	//! Note: Except for CurvedSubtaskExecutor, it's better to use function adjustAndGetCurve in
	//! bob_control/curve_tool.h to avoid invalid output. adjustAndGetCurve will check the robot
	//! heading, if it's facing away to the line, the function will command the robot to do 
	//! heading rotation to turn towards the line.
	class CurvePlanner
	{

		public:

			//! Stores curved path parameters including radius, turning direction, the linear velocity 
			//! to execute the curved path and the line to curve to.
			struct CurveSpecifics
			{

				//! \brief Constructor
				//! \param radius The curve radius
				//! \param turningDir The direction to turn 
				//! \param curveVel The velocity to execute the curve
				//! \param tarLine The line to curve to 
				CurveSpecifics(float radius, CircularDirection turningDir, float curveVel, Line tarLine) :
					radius(radius), 
					direction(turningDir),
					velocity(curveVel),
					toCurveTo(tarLine)
				{}
				
				float radius;
				CircularDirection direction;
				float velocity;
				Line toCurveTo;				
			};

			//! \brief Constructor
			//! \param line The line to curve to 
			//! \param robotPose robot current pose
			CurvePlanner(Line line, Pose2D robotPose) :
				tarLine(line),
				robotPose(robotPose)
			{}

			//! \brief Get the planned curve
			//! \return curve information stored in CurveSpecifics
			CurveSpecifics getCurve();


		private:

			CircularDirection getTurningDirection();
			float getCurveVel(float radius);

			Line tarLine;
			Pose2D robotPose;
			
	};

}

#endif
