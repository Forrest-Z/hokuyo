#include <gtest/gtest.h>

#include <iterator>

#include <bob_stc/stc_plan.h>
#include <bob_toolbox/easy_print.h>

#include <bob_stc/stc_plan_tools.h>

using namespace bob;

STCPlan testPlan()
{
	STCPlan unorderedPlan;
	STCPath path1;
	path1.push_back(WorldPoint(0, 0));
	path1.push_back(WorldPoint(1, 0));
	path1.push_back(WorldPoint(1, 1));
	path1.push_back(WorldPoint(0, 1));
	path1.push_back(WorldPoint(0, 0));
	
	STCPath path2;
	path2.push_back(WorldPoint(-1, -1));
	path2.push_back(WorldPoint(-2, -1));

	STCPath path3;
	path3.push_back(WorldPoint(-2, 0));
	path3.push_back(WorldPoint(-2, 1));
	path3.push_back(WorldPoint(-3, 1));
	path3.push_back(WorldPoint(-3, 0));
	path3.push_back(WorldPoint(-2, 0));

	STCPath path4;
	path4.push_back(WorldPoint(-2, -1));
	path4.push_back(WorldPoint(-2, -2));
	path4.push_back(WorldPoint(-1, -2));
	path4.push_back(WorldPoint(-1, -1));

	unorderedPlan.push_back(path1);
	unorderedPlan.push_back(path2);
	unorderedPlan.push_back(path3);
	unorderedPlan.push_back(path4);

	return unorderedPlan;
}

TEST (STCPlanReorganization, closestAtStartOfPath)
{
	STCPlan unorderedPlan = testPlan();
	STCPlan reorderedPlan = reorganizeAboutPoint(unorderedPlan, WorldPoint(-1.9, 0));
	
	STCPlan result;

	STCPlan::iterator itr = unorderedPlan.begin();
	std::advance(itr, 2);
	result.insert(result.end(), *(itr++));
	result.insert(result.end(), *itr);

	itr = unorderedPlan.begin();
	result.insert(result.end(), *(itr++));
	result.insert(result.end(), *itr);

	ASSERT_EQ(reorderedPlan, result);
}

TEST (STCPlanReorganization, closestAtEndOfPath)
{
	STCPlan unorderedPlan = testPlan();
	STCPlan reorderedPlan = reorganizeAboutPoint(unorderedPlan, WorldPoint(-1.9, -1));
	
	STCPlan result;

	STCPlan::iterator itr = unorderedPlan.begin();
	std::advance(itr, 2);
	result.insert(result.end(), *(itr++));
	result.insert(result.end(), *itr);

	itr = unorderedPlan.begin();
	result.insert(result.end(), *(itr++));
	result.insert(result.end(), *itr);

	ASSERT_EQ(reorderedPlan, result);
}
TEST (STCPlanReorganization, closestInMiddleOfPath)
{
	STCPlan unorderedPlan = testPlan();
	STCPlan reorderedPlan = reorganizeAboutPoint(unorderedPlan, WorldPoint(-4, 2));
	
	STCPlan result;
	STCPath result1;
	result1.push_back(WorldPoint(-3, 1));
	result1.push_back(WorldPoint(-3, 0));
	result1.push_back(WorldPoint(-2, 0));
	result.push_back(result1);

	STCPlan::iterator itr = unorderedPlan.begin();
	std::advance(itr, 3);
	result.insert(result.end(), *itr);

	itr = unorderedPlan.begin();
	result.insert(result.end(), *(itr++));
	result.insert(result.end(), *itr);

	STCPath result2;
	result2.push_back(WorldPoint(-2, 0));
	result2.push_back(WorldPoint(-2, 1));
	result2.push_back(WorldPoint(-3, 1));
	result.push_back(result2);

	ASSERT_EQ(reorderedPlan, result);
}
