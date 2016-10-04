#ifndef _BOB_WALL_FOLLOWING_WALL_FOLLOW_OVERLAP_CONDITION_H_
#define _BOB_WALL_FOLLOWING_WALL_FOLLOW_OVERLAP_CONDITION_H_

#include <bob_control/conditions/istop_condition.h>
#include <bob_toolbox/pose2d.h>
#include <bob_system/itimer.h>
#include <bob_system/system_utilities.h>

namespace bob
{

	//! \brief An IStopCondition used to detect wall follow back to covered area.
	class WallFollowOverlapCondition : public IStopCondition
	{
		public:
			
			//! \brief Constructor
			WallFollowOverlapCondition() : 
				initialized(false), 
				nextAdded(false),
				maxNumOfCheckPoint(100),
				period(10),
				lastAddTime(systemUtilities->timer())
			{}			

		private:
			
			virtual bool condition(const ISensorHandle& sensorHandle);
			
			void updateCheckList(const Pose2D& robotPose);
	
			bool check(const Pose2D& robotPose);

			std::vector<Pose2D> checkList;
			
			Pose2D nextCheckPoint;

			bool initialized;

			bool nextAdded;
			
			size_t maxNumOfCheckPoint;
			
			Timer lastAddTime;	
			
			float period;		

	};

}

#endif
