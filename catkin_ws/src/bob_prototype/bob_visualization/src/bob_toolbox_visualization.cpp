#include <bob_visualization/bob_toolbox_visualization.h>

#include <bob_toolbox/angular_range.h>
#include <bob_toolbox/world_point.h>

#include <vector>

namespace bob
{

	MarkerLine visualization(const SimpleAngularRange& range)
	{
		std::vector<WorldPoint> points;
		points.push_back(WorldPoint(0, 0));
		points.push_back(unitVector(range.lower));

		float resolution = 0.05;

		float angleDist = normalizeAnglePos(range.upper - range.lower);
		int numSteps = (int)(angleDist / resolution);

		for (int i = 1; i < numSteps; i++)
		{
			points.push_back(unitVector(range.lower + i * resolution));	
		}
		points.push_back(unitVector(range.upper));
		points.push_back(WorldPoint(0, 0));

		return MarkerLine(points);
	}

	MarkerLine visualization(const AngularRange& range)
	{
		std::vector<WorldPoint> points;
		AngularRange::container data = range.getRanges();
		for (AngularRange::container::iterator itr = data.begin();
				itr != data.end();
				++itr)
		{
			if (itr->isFullCircle())
			{
				float resolution = 0.05;
				float angleDist = 2 * M_PI;
				int numSteps = (int)(angleDist / resolution) + 1;

				for (int i = 0; i <= numSteps; i++)
				{
					points.push_back(unitVector((float)i * resolution));
				}

			}
			else
			{
				MarkerLine newPoints = visualization(*itr);
				points.insert(points.end(), newPoints.data.begin(), newPoints.data.end());
			}
		}
		return MarkerLine(points);
	}

}
