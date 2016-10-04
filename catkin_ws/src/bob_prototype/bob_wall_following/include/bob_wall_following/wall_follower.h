#ifndef _BOB_WALL_FOLLOWING_WALL_FOLLOWER_H_
#define _BOB_WALL_FOLLOWING_WALL_FOLLOWER_H_


#include <bob_control/wall_follow_side.h>
#include <bob_control/simple_commander.h>
#include <bob_control/conditions/istop_condition.h>
#include <bob_control/conditions/null_condition.h>

namespace bob
{

	class ISensorHandle;
	class LidarBeam;
	class LidarScan;

	class ReturnToSensed : public IStopCondition
	{

		public:

			ReturnToSensed(WallFollowSide side) : 
				side(side)
		{}


		private:

			WallFollowSide side;

			virtual bool condition(const ISensorHandle& sensorHandle);

	};


	//! \brief Implements a wall following algorithm. The system will drive along a wall until a parameterized condition
	//! is satisfied.
	class WallFollower
	{

		public:

			enum State
			{
				SenseTracking,

				BumperTracking
			};

			//! \brief Construct a wall follower instance
			//! \param commander Provides control of the robot
			//! \param sensorHandle Provides access to robot sensor data
			//! \param side The side to wall follow along
			//! \param initialState Defines if the wall follower should start with sensed wall following or bumper following
			WallFollower(SimpleCommander& commander, const ISensorHandle& sensorHandle, WallFollowSide side, State initialState = SenseTracking) :
				commander(commander),
				sensorHandle(sensorHandle),
				side(side),
				currentState(initialState)
		{}

			//! \brief Runs the wall follower
			void run(IStopCondition::shared_ptr condition = NullCon);

		private:


			bool sensedAlgorithm(IStopCondition::shared_ptr stopCondition);

			bool bumperAlgorithm(IStopCondition::shared_ptr stopCondition);

			LidarBeam getShortestBeam(const LidarScan& rangeData) const;

			float getRequiredHeading(float robotHeading, float closestBeamAngle) const;

			SimpleCommander& commander;

			const ISensorHandle& sensorHandle;

			WallFollowSide side;

			State currentState;

	};

}

#endif
