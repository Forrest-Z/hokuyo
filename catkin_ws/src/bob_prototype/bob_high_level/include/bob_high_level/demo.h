#ifndef _BOB_DEMO_DEMO_H_
#define _BOB_DEMO_DEMO_H_

#include <bob_control/simple_commander.h>
#include <bob_control/curved_path_executor.h>

#include <bob_navigation/navigation_manager.h>
#include <bob_frontier_exploration/frontier_tracker.h>
#include <bob_control_handle/control_handle.h>

#include <bob_boustrophedon/boustrophedon_executor.h>
#include <bob_coverage/area_chooser.h>
#include <bob_sensor/isensor_handle.h>
#include <bob_ros_implementations/ros_sensor_handle.h>

#include <bob_control/conditions/istop_condition.h>
#include <bob_control/ivelocity_publisher.h>

#include <bob_sensor/itransform_handle.h>


#include <bob_coverage/wall_chooser.h>
#include <bob_core/area_processor.h>

namespace bob
{

	class Demo
	{

		public:

			Demo(ISensorHandle& sensorHandle, LockableMap& lockableMap, IVelocityPublisher& velocityPublisher);

			void operator()();	

		private:

			void exploreEverything();

			void boustrophedonCoverage();

			void alternateExplorationAndCoverage();		

			void stcCoverage();

			void wallFollowObstacles();

			void backToStart();

			const LockableMap& lockableMap;

			ISensorHandle& sensorHandle;

			IVelocityPublisher& velocityPublisher;

			ControlHandle controlHandle;

			AreaProcessor areaProcessor;

	};

}

#endif
