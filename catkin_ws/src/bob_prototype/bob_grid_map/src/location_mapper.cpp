#include <bob_grid_map/location_mapper.h>

namespace bob
{

	LocationMapper::LocationMapper(float resolution) :
		resolution(resolution)
	{}

	WorldPoint LocationMapper::mapToWorld(const MapLocation location) const
	{
		float x = location.x * resolution;
		float y = location.y * resolution;
		return WorldPoint(x, y);
	}

	MapLocation LocationMapper::worldToMap(const WorldPoint point) const
	{
		int x = (int)round(point.x / resolution);
		int y = (int)round(point.y / resolution);
		return MapLocation(x, y);
	}

	WorldPoint LocationMapper::mapToWorldContinuous(const GenericPoint<float>& point) const
	{
		return WorldPoint(point.x * resolution, point.y * resolution);
	}

	GenericPoint<float> LocationMapper::worldToMapContinuous(const WorldPoint& point) const
	{
		return GenericPoint<float>(point.x / resolution, point.y / resolution);
	}

	float LocationMapper::getResolution() const
	{
		return resolution;
	}
}

