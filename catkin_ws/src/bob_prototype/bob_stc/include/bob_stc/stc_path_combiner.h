#ifndef _BOB_STC_STC_PATH_COMBINER_H_
#define _BOB_STC_STC_PATH_COMBINER_H_

#include <bob_stc/stc_plan.h>
#include <bob_toolbox/world_point.h>

#include <list>

namespace bob
{
	//! TODO: This class needs to be optimized. Right now it checks everything many more times than necessary 
	//! It should be possible to do a single pass through at which point all the connections are determined.
	//! Then, the resulting plan can be calculated all at once.
	//! The unit tests would likely need to be adjusted, because then the order of the resulting plan may
	//! not be consistent or predictable.


	//! Finds the shortest distance between STCPaths and combines them into an "STCPlan."
	//! This plan consists of adjacent STCPaths that have been split up at their closest points.
	class STCPathCombiner
	{

		public:

			//! Combines STCPaths to form STCPlan. All of the paths should be looped (front() == back())
			STCPlan combinePaths(std::list<STCPath> paths) const;
	
		private:

			//! Represents a connection between two STCPaths.
			struct STCConnection
			{
				//! Iterators pointing to the point in the path lying before the connection
				STCPath::iterator firstPointBefore;
				STCPath::iterator secondPointBefore;

				//! Points that lie on the paths, actually representing the connection itself
				WorldPoint firstSegmentPoint;
				WorldPoint secondSegmentPoint;
			};

			//! newSTCPaths.front() is automatically added, while the next path is added based on what is closest. The added path is deleted.
			void addFirstTwoElements(STCPlan& plan, std::list<STCPath>& newSTCPaths) const;

			//! Finds the closest path in newSTCPaths and adds it to the plan. The added path is deleted from newSTCPaths.
			void findAndAddElement(STCPlan& plan, std::list<STCPath>& newSTCPaths) const;

			//! Finds the shortest STCConnection between two paths. This is an O(nm) algorithm.
			STCConnection shortestConnection(STCPath& first, STCPath& second) const;

			//! Reorders an STC path around a position and a new point. The new point is intended to lie on the path before splitting.
			//! This function is intended to be called on STCPaths that are loops, and will produce a loop.
			void rearrangeAroundNewPoint(STCPath& path, STCPath::iterator pointBefore, WorldPoint newPoint) const;

			//! Splits a path and inserts a new point at the split point. The new point is intended to lie on the path before splitting.
			//! The resulting STCPaths will not be loops.
			std::pair<STCPath, STCPath> splitAroundNewPoint(const STCPath& path, STCPath::const_iterator pointBefore, WorldPoint newPoint) const;
	};

}

#endif
