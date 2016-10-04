#ifndef _BOB_CONTROL_SIMPLE_COMMANDS_H_
#define _BOB_CONTROL_SIMPLE_COMMANDS_H_

#include <bob_toolbox/world_point.h>
#include <bob_toolbox/line.h>

#include <bob_control/wall_follow_side.h>
#include <bob_control/commands/accelerate_command.h>
#include <bob_control/commands/heading_rotation_command.h>

#include <bob_config/config.h>

namespace bob
{
	


	struct StraightSettings
	{
		StraightSettings(float safetyDistance = Config::FRONT_SAFETY_DISTANCE, float finalSpeed = 0) : 
			safetyDistance(safetyDistance),
			finalSpeed(finalSpeed)
		{}

		//! The distance to avoid obstacles
		float safetyDistance;

		//! The intended final speed
		float finalSpeed;
	};




}

#endif
