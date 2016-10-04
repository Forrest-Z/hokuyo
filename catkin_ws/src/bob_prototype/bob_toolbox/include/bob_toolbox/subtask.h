#ifndef _BOB_TOOLBOX_SUBTASK_H_
#define _BOB_TOOLBOX_SUBTASK_H_

#include <bob_toolbox/line.h>

namespace bob
{

	//! A basic boustrophedon subclass. Consists of two lines, the line to track and
	//! the next line, which the robot will curve onto and eventually track when it	
	//! becomes the next subtask.

	//! This Subtask is a fundamental component of the Boustrophedon task. A Boustrophedon
	//! task actually consists of many of these Subtasks which are executed one after the
	//! other. The nature of the Boustrophedon algorithm is such that it allows us to simply
	//! instantiate an "initial subtask," which the robot follows first. All subsequent
	//! Subtasks can be calculated by "updating" the previous subtask. This will adjust the
	//! Subtask so that it becomes the next Subtask. This is possible because Boustrophedon
	//! is a simple repeatable pattern and each subsequent task can be determined from the
	//! previous one.
	struct Subtask 
	{	

		//! The line to track while completing the task
		Line toTrack;
	
		//! The line to curve onto in order to align with the next task.
		//! This line will become the "toTrack" line for the next task.
		Line toCurveTo;


	};

}

#endif
