#include <bob_control/commands/curve_command.h>
#include <bob_control/conditions/ored_condition.h>
#include <bob_control/conditions/front_obstacle_condition.h>
#include <bob_control/curve_to_line_controller.h>
#include <bob_control/conditions/pointing_to_goal_condition.h>


#include <boost/shared_ptr.hpp>
namespace bob
{

	IController::shared_ptr CurveCommand::generateController(const ISensorHandle& sensorHandle) const
	{
		boost::shared_ptr<CurveToLineController> controller(new CurveToLineController(toCurveTo, radius, direction, linearVelocity));
		return controller;
	}

	OredCondition::shared_ptr CurveCommand::generateCondition(const ISensorHandle& sensorHandle, IStopCondition*& obstacleCondition, IStopCondition*& goalCondition) const
	{
		OredCondition::shared_ptr condition(new OredCondition());
		condition->add(additionalCondition);

		FrontObstacleCondition::shared_ptr obstacle(new FrontObstacleCondition(Config::FRONT_SAFETY_DISTANCE));
		condition->add(obstacle);

		PointingToGoalCondition::shared_ptr goal(new PointingToGoalCondition(toCurveTo.origin + 3 * toCurveTo.vector));
		condition->add(goal);

		goalCondition = goal.get();
		obstacleCondition = obstacle.get();
	
		return condition;
	}

}

