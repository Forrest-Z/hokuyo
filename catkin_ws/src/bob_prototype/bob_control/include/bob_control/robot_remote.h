#ifndef _BOB_CONTROL_ROBOT_REMOTE_H_
#define _BOB_CONTROL_ROBOT_REMOTE_H_

#include <bob_control/control_result.h>
#include <bob_toolbox/circular_direction.h>
#include <bob_control/conditions/istop_condition.h>
#include <bob_control/conditions/null_condition.h>

namespace bob
{

	class SimpleCommander;
	class ISensorHandle;
	class Line;

	//! \brief This class provide basic actions of robot.
	class RobotRemote
	{

		public:

			//! Constructor
			RobotRemote(SimpleCommander& simpleCommander, const ISensorHandle& sensorHandle) : 
				simpleCommander(simpleCommander), 
				sensorHandle(sensorHandle)
			{}
			
			//! \brief Go backward for the given distance. Uses the robot position feedback from odometry.
			ControlResult backUp(float distance, IStopCondition::shared_ptr condition = NullCon) const;

			//! \brief Go forward for the given distance. Uses the robot position feedback from odometry.
			ControlResult goForward(float distance, IStopCondition::shared_ptr condition = NullCon) const;
			
			//! \brief Curve on to line
			ControlResult curveOnToLine(Line toCurveTo, IStopCondition::shared_ptr condition = NullCon) const;

			ControlResult basicCurve(float radius, float speed, CircularDirection direction, IStopCondition::shared_ptr condition = NullCon);
	
			ControlResult headingRotation(float offset, IStopCondition::shared_ptr condition = NullCon);

		private:
			
			ControlResult executeStraightCommand(float distance, float speed, IStopCondition::shared_ptr condition) const;

			SimpleCommander& simpleCommander;
			const ISensorHandle& sensorHandle;

	};

}

#endif
