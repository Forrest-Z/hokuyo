#include <bob_control/simple_commander.h>

#include <bob_control/commands/isimple_command.h>
#include <bob_control/recovery_behavior.h>
#include <bob_toolbox/strict_lock.h>

#include <bob_sensor/itransform_handle.h>
#include <bob_config/config.h>
#include <bob_toolbox/velocity2d.h>
#include <bob_toolbox/logging.h>

namespace bob
{

	ControlResult SimpleCommander::execute(ISimpleCommand& simpleCommand)
	{
		ControlResult result = runCommand(simpleCommand);
		if (result == BumperHit)
		{
			insertHiddenObstacle();	

			// We need to clear flags before recorvery because otherwise will 
			// stop simpleCommander performing actions
			sensorHandle.getBumperHandle().clearFlags();
			recoverFromBumperEvent(*this, sensorHandle);
		}
		else if (result == ObstacleSafety)
		{
			// Publish zero velocity
			velocityPublisher.publish(Velocity2D(0, 0));			
		}

		return result;
	}

	ControlResult SimpleCommander::runCommand(ISimpleCommand& simpleCommand)
	{
		Runnable runnable(simpleCommand.generateRunnable(sensorHandle));
		controllerRunner.run(runnable);
		return checkStopCondition(runnable);
	}

	void SimpleCommander::insertHiddenObstacle()
	{
		BumperState state = sensorHandle.getBumperHandle().getState();
		WorldVector offset = offsetFromBumper(state);
		Pose2D robotPose = sensorHandle.getTransformHandle().getLocalizedPose();
		WorldVector shiftedOffset = rotate(offset, robotPose.theta);
		WorldPoint hiddenObstaclePoint = robotPose + shiftedOffset;

		StrictLock lock(lockableMap.getLock());
		Costmap& costmap = lockableMap.getLockedResource(lock);
		costmap.getHiddenObstacleMap().addNewHiddenObstacle(hiddenObstaclePoint);
	}
	
	WorldVector SimpleCommander::offsetFromBumper(const BumperState& state)
	{
		float anglePIMultiple;	
		switch (state)
		{
			case LeftEvent:
				anglePIMultiple = 1.0 / 3.0;
				break;
			
			case LeftCenterEvent:
				anglePIMultiple = 1.0 / 5.0;
				break;

			case CenterEvent:
				anglePIMultiple = 0.0;	
				break;

			case RightCenterEvent:
				anglePIMultiple = - 1.0 / 5.0;	
				break;

			case RightEvent:
				anglePIMultiple = - 1.0 / 3.0;	
				break;

			case AllEvent:
				anglePIMultiple = 0.0;	
				break;

			case NoEvent:
				LOG_ERROR("ERROR: BUMPER TRIGGERED WITH NO EVENT");
				break;
		}
		return Config::ROBOT_RADIUS * unitVector(anglePIMultiple * M_PI);
	}
}

