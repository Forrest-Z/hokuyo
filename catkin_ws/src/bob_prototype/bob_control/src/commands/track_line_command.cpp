#include <bob_control/commands/track_line_command.h>
#include <bob_control/velocity_repeater.h>
#include <bob_control/line_pid_controller.h>
#include <bob_control/conditions/ored_condition.h>
#include <bob_control/conditions/front_obstacle_condition.h>
#include <bob_control/conditions/half_space_condition.h>

#include <boost/shared_ptr.hpp>

namespace bob
{

	IController::shared_ptr TrackLineCommand::generateController(const ISensorHandle& sensorHandle) const
	{
		// Controller
		boost::shared_ptr<VelocityRepeater> velocityRepeator(new VelocityRepeater(Velocity2D(speed, 0)));

		boost::shared_ptr<IController> controller(new LinePIDController(velocityRepeator, line, true)); 

		return controller;
	}

	OredCondition::shared_ptr TrackLineCommand::generateCondition(const ISensorHandle& sensorHandle, IStopCondition*& obstacleCondition, IStopCondition*& goalCondition) const
	{
		OredCondition::shared_ptr condition(new OredCondition());
		// additional condition
		condition->add(additionalCondition);

		// Goal condition
		HalfSpaceCondition::shared_ptr pastGoalCondition(new HalfSpaceCondition(MapHalfSpace(goal, line.getAngle())));	
		condition->add(pastGoalCondition);		
		
		// Obstacle condition
		FrontObstacleCondition::shared_ptr obstacle(new FrontObstacleCondition(safetyDistance));
		condition->add(obstacle);

		goalCondition = pastGoalCondition.get();
		obstacleCondition = obstacle.get();
		return condition;
	}

}

