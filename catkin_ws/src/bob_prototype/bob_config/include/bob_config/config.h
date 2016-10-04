#ifndef _BOB_CONFIG_CONFIG_H_
#define _BOB_CONFIG_CONFIG_H_

namespace bob
{

	//! \brief Provides access to a list of constants set using the ros parameter system.
	//! If a constant is not set in the parameter system, an assert will be triggered.
	//! During migration to embedded code, this class will need to be replaced with one
	//! that has its values set at compile-time.

	//! Use config() to get the config, then access the parameters directly.
	//! For example, to get the robot radius, use: Config::ROBOT_CONFIG
	class Config
	{

		public:

			// Available params:

			//! The radius of the robot, in meters
			static const float ROBOT_RADIUS;

			//! The linear velocity output of controllers is limited by this amount, for safety reasons
			static const float LINEAR_VELOCITY_BOUNDS;

			//! The angular velocity output of controllers is limited by this amount, for safety reasons
			static const float ANGULAR_VELOCITY_BOUNDS;

			//! The operating frequency of the rotation controller. Increase for smoother controller behavior
			static const float ROTATION_CONTROLLER_FREQUENCY;

			//! Maximum angular velocity while rotating in place
			static const float ROTATION_MAX_VELOCITY;

			//! Maximum angular acceleration while rotating in place
			static const float ROTATION_ACCELERATION;

			//! Proportional component of wall following PID constants
			static const float WALL_FOLLOW_P;
	
			//! Derivative component of wall following PID constants
			static const float WALL_FOLLOW_D;

			//! The desired wall-following distance 
			static const float WALL_FOLLOW_DISTANCE;

			//! The speed to use during wall following
			static const float WALL_FOLLOW_SPEED;

			//! The number of angles to consider for brute for STC approach (affects STC with O(n) complexity)
			static const float STC_NUM_ANGLES;

			//! The number of spacial shifts to consider for brute for STC approach (affects STC with O(n^2) complexity)
			static const float STC_NUM_SHIFTS;

			//! The radius used when making the STC grid. Can be slightly larger or smaller than actual radius. Represents the area that is actually vacuumed under the robot.
			static const float STC_ROBOT_COVERAGE_RADIUS;

			//! The scaling of the grid relative to the robot coverage radius. At 4.0, this generates the "typical" STC. At smaller values the robot will cover areas twice
			static const float STC_GRID_RADIUS_MULTIPLE;

			//! The resolution of the map used and procuded by SLAM
			static const float MAP_RESOLUTION;

			//! The rate at which to update the localized transform
			static const float SLAM_TRANSFORM_PUBLISH_PERIOD;

			//! The distance to inflate obstacles
			static const float OBSTACLE_INFLATION_DISTANCE;

			//! The safety distance for robot to stop
			static const float FRONT_SAFETY_DISTANCE;

			//! If the probability of an obstacle is above this amount it is considered an obstacle by the ObstacleMap
			static const float OBSTACLE_THRESHOLD_PROBABILITY;

			//! The constant decay parameter used for updating probablility of points in the map
			static const float SLAM_PROBABILITY_DECAY_FACTOR;

			//! The minimun accepted scan matching score to correct the odom pose estimation 
			static const float MIN_SCAN_MATCH_SCORE;

			static const float BLIND_WALL_FOLLOW_FORWARD_DISTANCE;

			static const float BLIND_WALL_FOLLOW_ROTATION_ANGLE;

			static const float BLIND_WALL_FOLLOW_CURVE_RADIUS;
			
			//! The maximun range for the distance sensor
			static const float MAX_SENSOR_RANGE;

		private:

			//! Private constructor. Instance creation is disallowed.
			Config();

	};

}

#endif
