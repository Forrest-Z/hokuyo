#include <gtest/gtest.h>

#include <bob_stc/stc_path_combiner.h>
#include <bob_toolbox/world_point_shapes.h>
#include <bob_toolbox/easy_print.h>

using namespace bob;

// All these tests rely on the fact that the first path passed to the path combiner will
// be the first path in the result. It is probably not good to rely on this in the future.
// In order to fix this we should probably have some function like:
// bool functionallyEquivalent(const STCPlan&, const STCPlan&)
// This would test if the same paths occur in any order. 

TEST (STCPathCombiner, degenerateCase)
{
	// Test that makes sure system doesn't crash when given
	// a single path
	STCPath path1;
	path1.push_back(WorldPoint(1, 1));
	path1.push_back(WorldPoint(-1, 1));
	path1.push_back(WorldPoint(-1, -1));
	path1.push_back(WorldPoint(1, -1));
	path1.push_back(WorldPoint(1, 1));

	std::list<STCPath> pathsToCombine;
	pathsToCombine.push_back(path1);
	STCPathCombiner combiner;

	STCPlan plan = combiner.combinePaths(pathsToCombine);
	
	STCPlan result;
	result.push_back(path1);
	
	ASSERT_EQ(plan, result);
}

TEST (STCPathCombiner, SimpleTwoPath)
{
	// This is a square of width 2 centered at (0, 0)
	STCPath path1;
	path1.push_back(WorldPoint(1, 1));
	path1.push_back(WorldPoint(-1, 1));
	path1.push_back(WorldPoint(-1, -1));
	path1.push_back(WorldPoint(1, -1));
	path1.push_back(WorldPoint(1, 1));

	// This is a diamond centered at (1, 0)
	STCPath path2;
	path2.push_back(WorldPoint(2, 0));
	path2.push_back(WorldPoint(3, 1));
	path2.push_back(WorldPoint(4, 0));
	path2.push_back(WorldPoint(3, -1));
	path2.push_back(WorldPoint(2, 0));

	std::list<STCPath> pathsToCombine;
	pathsToCombine.push_back(path1);
	pathsToCombine.push_back(path2);
	STCPathCombiner combiner;

	STCPlan plan = combiner.combinePaths(pathsToCombine);

	STCPath result1 = path1;
	// This is the closest point to the two paths
	result1.insert(result1.begin(), WorldPoint(1, 0));
	// Remove the last point because it is redundant
	result1.erase(--result1.end());
	// Add the closest point - forming a loop
	result1.insert(result1.end(), WorldPoint(1, 0));

	// Path2 is unchanged
	
	STCPlan result;
	result.push_back(result1);
	result.push_back(path2);

	ASSERT_EQ(result, plan);
}

TEST (STCPathCombiner, SimpleThreePath)
{
	// Square of width 1 centered at (3.5, 3.5)
	STCPath path1;
	path1.push_back(WorldPoint(4, 3));
	path1.push_back(WorldPoint(4, 4));
	path1.push_back(WorldPoint(3, 4));
	path1.push_back(WorldPoint(3, 3));
	path1.push_back(WorldPoint(4, 3));

	// Square of width 1 centered at (1.5, 1.5)
	STCPath path2;
	path2.push_back(WorldPoint(1, 2));
	path2.push_back(WorldPoint(2, 2));
	path2.push_back(WorldPoint(2, 1));
	path2.push_back(WorldPoint(1, 1));
	path2.push_back(WorldPoint(1, 2));

	// diamond centered at (-1.5, 1.5)
	STCPath path3;
	path3.push_back(WorldPoint(-1, 1.5));
	path3.push_back(WorldPoint(-1.5, 2));
	path3.push_back(WorldPoint(-2, 1.5));
	path3.push_back(WorldPoint(-1.5, 1));
	path3.push_back(WorldPoint(-1, 1.5));

	std::list<STCPath> pathsToCombine;
	pathsToCombine.push_back(path1);
	pathsToCombine.push_back(path2);
	pathsToCombine.push_back(path3);
	STCPathCombiner combiner;

	STCPlan plan = combiner.combinePaths(pathsToCombine);

	STCPath result1;
	result1.push_back(WorldPoint(3, 3));
	result1.push_back(WorldPoint(4, 3));
	result1.push_back(WorldPoint(4, 4));
	result1.push_back(WorldPoint(3, 4));
	result1.push_back(WorldPoint(3, 3));

	STCPath result2;
	result2.push_back(WorldPoint(2, 2));
	result2.push_back(WorldPoint(2, 1));
	result2.push_back(WorldPoint(1, 1));
	result2.push_back(WorldPoint(1, 1.5));

	STCPath result3;
	result3.push_back(WorldPoint(-1, 1.5));
	result3.push_back(WorldPoint(-1.5, 2));
	result3.push_back(WorldPoint(-2, 1.5));
	result3.push_back(WorldPoint(-1.5, 1));
	result3.push_back(WorldPoint(-1, 1.5));

	STCPath result4;
	result4.push_back(WorldPoint(1, 1.5));
	result4.push_back(WorldPoint(1, 2));
	result4.push_back(WorldPoint(2, 2));
	
	STCPlan result;
	result.push_back(result1);
	result.push_back(result2);
	result.push_back(result3);
	result.push_back(result4);

	ASSERT_EQ(result, plan);
}

TEST (STCPathCombiner, SimpleFourPath)
{
	// Diamond centered at (2.5, 2.5)
	STCPath path1;
	path1.push_back(WorldPoint(1, 2));
	path1.push_back(WorldPoint(2, 1));
	path1.push_back(WorldPoint(3, 2));
	path1.push_back(WorldPoint(2, 3));
	path1.push_back(WorldPoint(1, 2));

	// Same as path1, but centered at (-2.5, 2.5)
	STCPath path2;
	path2.push_back(WorldPoint(-1, 2));
	path2.push_back(WorldPoint(-2, 1));
	path2.push_back(WorldPoint(-3, 2));
	path2.push_back(WorldPoint(-2, 3));
	path2.push_back(WorldPoint(-1, 2));

	// Square of length 1 centered at (4.5, 2.5)
	STCPath path3;
	path3.push_back(WorldPoint(5, 3));
	path3.push_back(WorldPoint(4, 3));
	path3.push_back(WorldPoint(4, 2));
	path3.push_back(WorldPoint(5, 2));
	path3.push_back(WorldPoint(5, 3));

	// Square of length 1 centered at (0.5, 0.5)
	STCPath path4;
	path4.push_back(WorldPoint(0, 0));
	path4.push_back(WorldPoint(1, 0));
	path4.push_back(WorldPoint(1, 1));
	path4.push_back(WorldPoint(0, 1));
	path4.push_back(WorldPoint(0, 0));

	std::list<STCPath> pathsToCombine;
	pathsToCombine.push_back(path1);
	pathsToCombine.push_back(path2);
	pathsToCombine.push_back(path3);
	pathsToCombine.push_back(path4);
	STCPathCombiner combiner;

	STCPlan plan = combiner.combinePaths(pathsToCombine);

	STCPlan result;
	STCPath result1;
	result1.push_back(WorldPoint(1.5, 1.5));
	result1.push_back(WorldPoint(2, 1));
	result1.push_back(WorldPoint(3, 2));

	STCPath result2;
	result2.push_back(WorldPoint(4, 2));
	result2.push_back(WorldPoint(5, 2));
	result2.push_back(WorldPoint(5, 3));
	result2.push_back(WorldPoint(4, 3));
	result2.push_back(WorldPoint(4, 2));

	STCPath result3;
	result3.push_back(WorldPoint(3, 2));
	result3.push_back(WorldPoint(2, 3));
	result3.push_back(WorldPoint(1, 2));
	result3.push_back(WorldPoint(1.5, 1.5));

	STCPath result4;
	result4.push_back(WorldPoint(1, 1));
	result4.push_back(WorldPoint(0, 1));

	STCPath result5;
	result5.push_back(WorldPoint(-1, 2));
	result5.push_back(WorldPoint(-2, 1));
	result5.push_back(WorldPoint(-3, 2));
	result5.push_back(WorldPoint(-2, 3));
	result5.push_back(WorldPoint(-1, 2));

	STCPath result6;
	result6.push_back(WorldPoint(0, 1));
	result6.push_back(WorldPoint(0, 0));
	result6.push_back(WorldPoint(1, 0));
	result6.push_back(WorldPoint(1, 1));

	result.push_back(result1);
	result.push_back(result2);
	result.push_back(result3);
	result.push_back(result4);
	result.push_back(result5);
	result.push_back(result6);

	ASSERT_EQ(result, plan);
}
