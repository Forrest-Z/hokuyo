#ifndef _BOB_CONTROL_ACCELERATE_COMMAND_H_
#define _BOB_CONTROL_ACCELERATE_COMMAND_H_

#include <bob_control/commands/isimple_command.h>
#include <bob_control/conditions/null_condition.h>

#include <bob_toolbox/line.h>
#include <bob_control/runnable.h>

namespace bob
{
	
	//! \brief Defines an ISimpleCommand for accelerate and decelerate.
	//!
	//! AccelerateCommand include the line to accelerate along, the distance
	//! to accelerate, the final speed after acceleration and an optional 
	//! stop condition.
	//!
	//! By executing the runnable generated by AccelerateCommand, the robot will
	//! accelerate along the given line for the given distance to the given
	//! final speed, and stop on obstacle, goal and the given additional 
	//! condition. 
	struct AccelerateCommand : public ISimpleCommand
	{

		//! \brief Constructor
		//! \param line The line to accelerate along
		//! \param distance The distance to accelerate for
		//! \param finalSpeed The final speed to accelerate to
		//! \param additionalCondition Optional additional stop condition
		AccelerateCommand(Line line, float distance, float finalSpeed, IStopCondition::shared_ptr additionalCondition = NullCon) :
			line(line),
			distance(distance),
			finalSpeed(finalSpeed),
			additionalCondition(additionalCondition)
		{}
		
		virtual IController::shared_ptr generateController(const ISensorHandle& sensorHandle) const;

		virtual OredCondition::shared_ptr generateCondition(const ISensorHandle& sensorHandle, IStopCondition*& obstacleCondition, IStopCondition*& goalCondition) const;

		Line line;	
		float distance;
		float finalSpeed;
		IStopCondition::shared_ptr additionalCondition;

	};

}

#endif
