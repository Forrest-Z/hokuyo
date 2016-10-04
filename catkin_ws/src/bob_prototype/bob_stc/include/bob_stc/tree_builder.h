#ifndef _BOB_STC_TREE_BUILDER_H_
#define _BOB_STC_TREE_BUILDER_H_

namespace bob
{

	class SpanningTreeGrid;
	class GridSet;

	class TreeBuilder 
	{

		virtual SpanningTreeGrid buildTree(GridSet freeCells)=0;


	};

}

#endif 


