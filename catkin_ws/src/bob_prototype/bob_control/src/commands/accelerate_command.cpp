#include <bob_control/commands/accelerate_command.h>

#include <bob_control/conditions/ored_condition.h>
#include <bob_control/conditions/front_obstacle_condition.h>
#include <bob_control/conditions/distance_condition.h>
#include <bob_control/linear_accelerate_controller.h>
#include <bob_control/line_pid_controller.h>

#include <bob_toolbox/pose2d.h>

#include <bob_sensor/itransform_handle.h>

namespace bob
{


	IController::shared_ptr AccelerateCommand::generateController(const ISensorHandle& sensorHandle) const
	{
		Pose2D startingOdomPose = sensorHandle.getTransformHandle().getOdomPose();

		// Accelerate controller for linear speed		
		boost::shared_ptr<LinearAccelerateController> controller(new LinearAccelerateController(startingOdomPose, distance, finalSpeed));

		// PID controller wrapper for angular speed to track a line
		LinePIDController PIDController(controller, line, true);
		boost::shared_ptr<IController> finalController(new LinePIDController(controller, line, true));
		
		return finalController;
	}

	OredCondition::shared_ptr AccelerateCommand::generateCondition(const ISensorHandle& sensorHandle, IStopCondition*& obstacleCondition, IStopCondition*& goalCondition) const
	{
		OredCondition::shared_ptr condition(new OredCondition());
		condition->add(additionalCondition);
		
		FrontObstacleCondition::shared_ptr obstacle(new FrontObstacleCondition(Config::FRONT_SAFETY_DISTANCE));
		condition->add(obstacle);

		DistanceCondition::shared_ptr distanceCondition(new DistanceCondition(sensorHandle, distance));
		condition->add(distanceCondition);

		obstacleCondition = obstacle.get();
		goalCondition = distanceCondition.get();

		return condition;
	}
}

