#include <ros/ros.h>
#include <bob_toolbox/logging.h>
#include <bob_ros_implementations/ros_map_listener.h>
#include <bob_visualization/bob_toolbox_visualization.h>
#include <bob_grid_map/toolbox.h>
#include <bob_toolbox/world_point.h>
#include <bob_sensor/itransform_handle.h>
#include <nav_msgs/Path.h>
#include <bob_visualization/visualization.h>

#include <bob_navigation/path_splitter.h>
#include <boost/bind.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <bob_toolbox/strict_lock.h>
#include <bob_grid_map/lockable_map.h>

#include <bob_navigation/dijkstra_planner.h>

// This test file is used to received published points using rviz.
// You must click the "publish point" button in rviz and click somewhere in the map.
// This publishes to clicked_point topic, which we are subscribed to.
namespace bob
{

	namespace test
	{

		//! 
		class PathTester
		{

			public:

				PathTester() :
				mapUpdater(lockableMap)
				{
					waitForMapAvailable(lockableMap);
					LOG_TEST("Map Received");
				}


				void planPoint(WorldPoint point)
				{
					StrictLock lock(lockableMap.getLock());
					Costmap& costmap = lockableMap.getLockedResource(lock);
			
					LOG_TEST("Got lock");

					bob::MarkerStyle styleOriginal(1, 0.03, new Green());
					bob::MarkerStyle styleNew(1, 0.03, new Blue());
					std::vector<WorldPoint> plan;

					// start and goal	
					WorldPoint robotPoint(0.0, 0.0);
					WorldPoint goal = point;

					DijkstraPlanner dijkstraPlanner(costmap);
					ros::Time start = ros::Time::now();
					std::vector<WorldPoint> otherPlan;
					if(dijkstraPlanner.makePlan(robotPoint, goal, otherPlan))
					{
						LOG_TEST("Took: " << (ros::Time::now() - start).toSec());
						visualizer->visualize("OldPlan", MarkerLine(otherPlan), styleOriginal);

						std::vector<WorldPoint> optimized = optimizePlan(costmap, otherPlan);
						visualizer->visualize("OptimizedPlan", MarkerLine(optimized), styleNew);
					}
					else
					{
						LOG_TEST("No Plan found");
					}


				}

			private:

				
					LockableMap lockableMap;
					ROSMapListener mapUpdater;

		};

		void testPoint(geometry_msgs::PointStamped point)
		{
			static PathTester pathTester;
			pathTester.planPoint(WorldPoint(point.point.x, point.point.y));	
		}

	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_test");
	ros::NodeHandle n;
	ros::Publisher globalPlanPublisher;
	ros::Publisher globalOptimizedPlanPublisher;

	ros::Subscriber clickedPointSubscriber = n.subscribe<geometry_msgs::PointStamped>("clicked_point", 1, &bob::test::testPoint);
	LOG_TEST("Running");

	ros::spin();
}
