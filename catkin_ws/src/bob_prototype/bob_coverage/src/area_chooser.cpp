#include <bob_coverage/area_chooser.h>
#include <bob_coverage/area_tools.h>
#include <bob_toolbox/world_point_shapes.h>
#include <bob_toolbox/pose2d.h>

namespace bob
{

	AreaChooser::AreaChooser(Costmap& costmap, const ITransformHandle& transformHandle) : 
		costmap(costmap),
		coverageTracker(transformHandle, 2),
		transformHandle(transformHandle),
		boundRegion(worldPointSquare(WorldPoint(0.0, 0.0), 0.0, 6.0))
	{}

	DiscreteArea AreaChooser::nextArea() const
	{
		WorldPoint robotPosition = transformHandle.getLocalizedPose();

		float searchDistance = 5;
		WorldRectangle coveredRectangle;
		coveredRectangle.angle = 0;
		coveredRectangle.origin = robotPosition - WorldVector(searchDistance, searchDistance);
		coveredRectangle.bounds = Dimension2D<float>(2 * searchDistance, 2 * searchDistance);

		// Extract the areas of the costmap that we want to cover
		float bufferedRadius = 0.35;
		DiscreteArea area = extractThresholdedArea(costmap, coveredRectangle.getPoints(), bufferedRadius, 0.05);

		// Erase the paths the robot has already passed over
		area.erase(coverageTracker.getCoveredArea());

		return area;
	}
}

