#include <bob_config/config.h>

namespace bob 
{

	//! The radius of the robot, in meters
	const float Config::ROBOT_RADIUS = 0.175;

	//! The linear velocity output of controllers is limited by this amount, for safety reasons
	const float Config::LINEAR_VELOCITY_BOUNDS = 0.3;

	//! The angular velocity output of controllers is limited by this amount, for safety reasons
	const float Config::ANGULAR_VELOCITY_BOUNDS = 2.5;

	//! The operating frequency of the rotation controller. Increase for smoother controller behavior
	const float Config::ROTATION_CONTROLLER_FREQUENCY = 20;

	//! Maximum angular velocity while rotating in place
	const float Config::ROTATION_MAX_VELOCITY = 1.0;

	//! Maximum angular acceleration while rotating in place
	const float Config::ROTATION_ACCELERATION = 1.0;

	//! Proportional component of wall following PID constants
	const float Config::WALL_FOLLOW_P = 15;

	//! Derivative component of wall following PID constants
	const float Config::WALL_FOLLOW_D = 30;

	//! The desired wall-following distance 
	const float Config::WALL_FOLLOW_DISTANCE = 0.03;

	//! The speed to use during wall following
	const float Config::WALL_FOLLOW_SPEED = 0.1;

	//! The number of angles to consider for brute for STC approach (affects STC with O(n) complexity)
	const float Config::STC_NUM_ANGLES = 6;

	//! The number of spacial shifts to consider for brute for STC approach (affects STC with O(n^2) complexity)
	const float Config::STC_NUM_SHIFTS = 2;

	//! The radius used when making the STC grid. Can be slightly larger or smaller than actual radius. Represents the area that is actually vacuumed under the robot.
	const float Config::STC_ROBOT_COVERAGE_RADIUS = 0.170;

	//! The scaling of the grid relative to the robot coverage radius. At 4.0, this generates the "typical" STC. At smaller values the robot will cover areas twice
	const float Config::STC_GRID_RADIUS_MULTIPLE = 2.0;

	//! The resolution of the map used and procuded by SLAM
	const float Config::MAP_RESOLUTION = 0.05;

	//! The rate at which to update the localized transform
	const float Config::SLAM_TRANSFORM_PUBLISH_PERIOD = 0.05;

	//! The distance to inflate obstacles
	const float Config::OBSTACLE_INFLATION_DISTANCE = 0.6;

	//! The safety distance for robot to stop
	const float Config::FRONT_SAFETY_DISTANCE = 0.1;

	//! If the probability of an obstacle is above this amount it is considered an obstacle by the ObstacleMap
	const float Config::OBSTACLE_THRESHOLD_PROBABILITY = 0.25;

	//! The constant decay parameter used for updating probablility of points in the map
	const float Config::SLAM_PROBABILITY_DECAY_FACTOR = 0.1;

	//! The minimun accepted scan matching score to correct the odom pose estimation 
	const float Config::MIN_SCAN_MATCH_SCORE = 0.7;

	const float Config::BLIND_WALL_FOLLOW_FORWARD_DISTANCE = 0.13;

	const float Config::BLIND_WALL_FOLLOW_ROTATION_ANGLE = 0.7;

	const float Config::BLIND_WALL_FOLLOW_CURVE_RADIUS = 0.15;

	//! The maximun range for the distance sensor [m]
	const float Config::MAX_SENSOR_RANGE = 3;

}
