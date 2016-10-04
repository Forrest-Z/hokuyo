#ifndef _BOB_MAP_ALGORITHMS_MAP_FUNCTOR_H_
#define _BOB_MAP_ALGORITHMS_MAP_FUNCTOR_H_

#include <bob_grid_map/costmap.h>
#include <boost/shared_ptr.hpp>

namespace bob
{

	//! \brief This is a abstruct class for point map status checking.
	class MapFunctor
	{
		public:
			typedef boost::shared_ptr<MapFunctor> shared_ptr;

			virtual bool operator()(const MapLocation& point) const = 0;

	};

	 
	class CellStateIs: public MapFunctor
	{
		public:
			CellStateIs(const Costmap& costmap, FreeState state): 
				costmap(costmap), 
				state(state) 
				{}

			virtual bool operator()(const MapLocation& point) const;

		private:
			const Costmap& costmap;

			FreeState state;
	}; 

	class CellAwayFromObstacles: public MapFunctor
	{
		public:
			CellAwayFromObstacles(const Costmap& costmap, float minDistance):
				costmap(costmap),
				minDistance(minDistance) {}
			
			virtual bool operator()(const MapLocation& point) const;

		private:
			const Costmap& costmap;

			float minDistance;
	};


	class CompositeMapFunctor : public MapFunctor
	{
		public:
			typedef boost::shared_ptr<CompositeMapFunctor> shared_ptr;

			void add(const MapFunctor::shared_ptr functor);

		protected:
			std::vector<MapFunctor::shared_ptr> functors;
	};


	class OredMapFunctor : public CompositeMapFunctor
	{
		public:
			virtual bool operator()(const MapLocation& point) const;
	};


	class AndedMapFunctor : public CompositeMapFunctor
	{
		public:
			virtual bool operator()(const MapLocation& point) const;
	};
	

	class NotMapFunctor : public MapFunctor
	{
		public:
			NotMapFunctor(MapFunctor::shared_ptr toBeNotFunctor) : 
				toBeNotFunctor(toBeNotFunctor)
			{}
		
			virtual bool operator()(const MapLocation& point) const;

		private:
			MapFunctor::shared_ptr toBeNotFunctor;
	};
	

}

#endif
