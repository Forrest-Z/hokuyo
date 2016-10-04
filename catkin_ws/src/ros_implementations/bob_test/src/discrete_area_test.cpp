#include <bob_coverage/discrete_area.h>
#include <bob_coverage/binary_map_area.h>
#include <bob_toolbox/dimension2d.h>
#include <bob_toolbox/logging.h>

#include <ros/ros.h>

#include <cstdlib>

using namespace bob;

std::vector<int> primes { 2, 3, 5, 7, 11, 13, 17, 19 };

struct DiscreteAreaTestParams
{
	// The number of points to add into the area collection
	int pointsToFill;

	// The width and height of the area where we will be adding points
	int widthOfBounds;
	int heightOfBounds;

	// The resolution of the areas to generate
	float resolution;
};

int decidePrimeFactor(int totalBoundsSize)
{
	// Choose smallest prime that is not a factor of totalBoundsSize
	for (int i : primes)
	{
		if (totalBoundsSize % i != 0)
			return i;
	}
	return 0;
}

template <typename AreaType>
void fillAreaWithPoints(const DiscreteAreaTestParams& testParams, AreaType& area)
{
	// The total number of points in the square
	int totalSize = testParams.widthOfBounds * testParams.heightOfBounds;

	if (testParams.pointsToFill > totalSize)
	{
		LOG_ERROR("Points wont fit in bounds. Choose a bigger bounds");
		return;
	}

	// Choose a prime factor that will allow us to deterministically fill the square 
	// in a way that won't over lap any points
	int increment = decidePrimeFactor(totalSize);

	// The number of points added to the area being tested
	int count = 0;

	// Used in conjungtion with the prime factor increment to place points
	int pointOffset = 0;
	while (count < testParams.pointsToFill && ros::ok())
	{
		int x = pointOffset % testParams.widthOfBounds;
		int y = pointOffset / testParams.widthOfBounds;
		MapLocation point(x, y);

		// Add point if not already in the area
		if (!area.contains(point))
		{
			area.insert(point);
			count++;
		}	
		else
		{
			// This should never happen
			LOG_ERROR("Point duplicate found " << point);
		}
	
		// Update the offset and wrap around
		pointOffset += increment;
		pointOffset %= totalSize;
	}
}

void discreteTest(const DiscreteAreaTestParams& testParams)
{
	// Test version that uses discreteArea
	DiscreteArea inefficientArea(testParams.resolution);
	fillAreaWithPoints(testParams, inefficientArea);
}

void binaryMapTest(const DiscreteAreaTestParams& testParams)
{
	// Test version that uses binaryMapArea
	BinaryMapArea efficientArea(testParams.resolution);
	MapMetadata mapSize;
	mapSize.bottomLeftCorner = MapLocation(0, 0);
	mapSize.bounds = Dimension2D<int>(testParams.widthOfBounds, testParams.heightOfBounds);	
	
	efficientArea.resizeClear(mapSize);

	fillAreaWithPoints(testParams, efficientArea);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "discrete_area_test");
	ros::NodeHandle n;

	// Setup test params
	DiscreteAreaTestParams testParams;
	testParams.pointsToFill = 600;
	testParams.widthOfBounds = 100;
	testParams.heightOfBounds = 100;
	testParams.resolution = 0.05;

	// Testing with discreteArea
	LOG_TEST("Discrete");
	discreteTest(testParams);

	// Testing with binary area
	LOG_TEST("Binary");
	binaryMapTest(testParams);

	LOG_TEST("Done");
	
}
