#include <bob_stc/visualization.h>

namespace bob
{

	MarkerLine visualization(const STCPlan& plan)
	{
		std::vector<WorldPoint> points;
		for (STCPlan::const_iterator itr = plan.begin(); itr != plan.end(); ++itr)
		{
			points.insert(points.end(), itr->begin(), itr->end());
		}
		return MarkerLine(points);
	}

	MarkerLineList visualization(const SpanningTreeGrid& spanningTreeGrid)
	{
		std::vector<WorldPoint> points;
		SpanningTreeGrid::DepthReturnIterator iterator = spanningTreeGrid.depthReturnIterator();
	
		std::stack<GridPoint> parentStack;

		GridPoint previousPoint = iterator.currentPoint();
		iterator.next();
		GridPoint currentPoint;
		while (!iterator.done())
		{
			currentPoint = iterator.currentPoint();

			if (parentStack.empty() || currentPoint != parentStack.top())
			{
				parentStack.push(previousPoint);
			}
			else
			{
				points.push_back(spanningTreeGrid.gridToWorld(previousPoint));
				points.push_back(spanningTreeGrid.gridToWorld(currentPoint));
				parentStack.pop();
			}

			previousPoint = currentPoint;
			iterator.next();
		}
		return MarkerLineList(points);
	}	

}

