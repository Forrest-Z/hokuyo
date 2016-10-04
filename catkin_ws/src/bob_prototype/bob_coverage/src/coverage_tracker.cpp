#include <bob_coverage/coverage_tracker.h>

#include <vector>

#include <bob_toolbox/world_point.h>
#include <bob_toolbox/pose2d.h>

#include <bob_coverage/discrete_area_shapes.h>

namespace bob
{

	DiscreteArea CoverageTrackerThread::getCoveredArea() const
	{
		boost::mutex::scoped_lock lock(coveredPointsMutex);
		return coveredArea;
	}

	DiscreteArea CoverageTracker::getCoveredArea() const
	{
		return thread.getCoveredArea();
	}

	void CoverageTrackerThread::insertMask(const DiscreteArea& mask)
	{
		boost::mutex::scoped_lock lock(coveredPointsMutex);
		coveredArea.insert(mask);
	}

	void CoverageTrackerThread::operator()()
	{
		WorldPoint robotPoint = transformHandle.getLocalizedPose();

		DiscreteArea robotFootprint = discreteAreaCircle(robotPoint, 0.18, 0.05);

		insertMask(robotFootprint);
	}

}

