#include <bob_ros_implementations/ros_map_listener.h>

#include <bob_grid_map/lockable_map.h>
#include <bob_ros_implementations/from_ros.h>
#include <bob_toolbox/strict_lock.h>
#include <bob_grid_map/costmap.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>
#include <ros/ros.h>

namespace bob
{

	ROSMapListener::ROSMapListener(LockableMap& lockableMap) :
		lockableMap(lockableMap),
		staticMapPublisher("static_costmap"),
		distanceMapPublisher("distance_costmap"),
		publishTime(5.0),
		mapUpdateThread(boost::thread(boost::bind(&ROSMapListener::mapUpdateLoop, this, 3.0)))
	{
		ros::NodeHandle nh;

		mapSubscriber = nh.subscribe("map", 1, &ROSMapListener::newMapData, this); 
	}
	
	void ROSMapListener::newMapData(const nav_msgs::OccupancyGridConstPtr& new_map)
	{
		// Lock map
		StrictLock lock(lockableMap.getLock());	
		Costmap& costmap = lockableMap.getLockedResource(lock);
	
		// Load new data into map
		importROSMapData(costmap.getProbabilityMap(), new_map);

		costmap.inflateObstacles();
	}

	void ROSMapListener::mapUpdateLoop(float frequency)
	{
		ros::NodeHandle nh;
		ros::Rate r(frequency);
		ros::Time last_publish_(0);
		while (nh.ok())
		{
			if (publishTime.toSec() > 0)
			{
				ros::Time now = ros::Time::now();
				if (now > last_publish_ + publishTime)
				{
					// Lock map
					StrictLock lock(lockableMap.getLock());	
					Costmap& costmap = lockableMap.getLockedResource(lock);
			
					// Echo back map data
					staticMapPublisher.publishCostmap(costmap.getObstacleMap());
					distanceMapPublisher.publishCostmap(costmap.getObstacleDistanceMap());

					// Update publishing time
					last_publish_ = now;
				}
			}
			r.sleep();
		}
	}
}
