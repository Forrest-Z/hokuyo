#include <ros/ros.h>

#include <bob_ros_implementations/from_ros.h>
#include <bob_grid_map/location_mapper.h>

#include <bob_grid_map/map_metadata.h>
#include <bob_grid_map/iprobability_map.h>

namespace bob
{


	MapMetadata extractMsgMetadata(const MapMsgPtr& new_map)
	{
		MapMetadata result;
		float resolution = new_map->info.resolution;
		result.bounds.width = new_map->info.width;
		result.bounds.height = new_map->info.height;	

		WorldPoint mapOrigin(new_map->info.origin.position.x, new_map->info.origin.position.y);

		// Need to adjust origin because bottom left corner is not on point
		WorldPoint adjustedOrigin = mapOrigin + WorldVector(resolution / 2, resolution / 2);

		LocationMapper mapper(resolution);
		result.bottomLeftCorner = mapper.worldToMap(adjustedOrigin);

		return result;
	}

	void importROSMapData(IProbabilityMap& costmap, const MapMsgPtr& new_map)
	{
		MapMetadata newMetadata = extractMsgMetadata(new_map);

		// Resize costmap if size, resolution or origin do not match
		if (costmap.getMapMetadata() != newMetadata)
		{
			// Only update the size of the costmap stored locally in this layer
			costmap.resizeClear(newMetadata);
		}

		// Initialize the costmap with static data
		unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
		unsigned int index = 0;

		MapLocation upperRight(newMetadata.upperRightCorner());

		for (int y = newMetadata.bottomLeftCorner.y; y <= upperRight.y; ++y)
		{
			for (int x = newMetadata.bottomLeftCorner.x; x <= upperRight.x; ++x)
			{
				unsigned char value = new_map->data[index];

				MapLocation point(x, y);
				float probability;
				if (value != 255)
				{
					// Not unknown
					probability = (float)value / 100;
				}
				else
				{
					// Unknown
					probability = -1;
 				}

				
				// Define a convertDataFromROS for each MapType if you want to publish a different/new MapType
				costmap.setValue(point, probability);
				++index;
			}
		}

	}

}
