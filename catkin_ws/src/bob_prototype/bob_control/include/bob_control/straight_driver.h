#ifndef _BOB_CONTROL_STRAIGHT_DRIVER_H_
#define _BOB_CONTROL_STRAIGHT_DRIVER_H_

#include <bob_sensor/itransform_handle.h>

#include <bob_control/simple_commander.h>
#include <bob_control/conditions/istop_condition.h>
#include <bob_control/conditions/null_condition.h>
#include <bob_control/commands/accelerate_command.h>
#include <bob_control/commands/track_line_command.h>

#include <bob_toolbox/line.h>
#include <bob_toolbox/world_point.h>
#include <bob_sensor/isensor_handle.h>

namespace bob
{

	class StraightDriver
	{
		
		public:

			//! A command that can be given to the class
			struct Command
			{
				Command(Line toTrack, WorldPoint goal, float finalSpeed, float safetyDistance) :
					toTrack(toTrack),
					goal(goal),
					finalSpeed(finalSpeed),
					safetyDistance(safetyDistance)
				{}

				//! The line that the robot will try to align itself with
				Line toTrack;
		
				//! The eventual goal point for the action
				WorldPoint goal;
	
				//! The final speed of the robot (when it reaches the goal)
				float finalSpeed;
	
				//! The minimum forward distance allowed before the robot will
				//! stop due to obstacle safety
				float safetyDistance;
			};

			StraightDriver(const ITransformHandle& transformHandle, SimpleCommander& simpleCommander) :
			simpleCommander(simpleCommander), 
			transformHandle(transformHandle)
			{}	
	
			//! Execute a command
			ControlResult execute(Command command, IStopCondition::shared_ptr additionalCondition = NullCon);		

		private:

			SimpleCommander& simpleCommander;

			const ITransformHandle& transformHandle;
	
	};

}

#endif
