#ifndef _BOB_COVERAGE_DISCRETE_AREA_H_
#define _BOB_COVERAGE_DISCRETE_AREA_H_

#include <bob_toolbox/world_point.h>

#include <bob_toolbox/grid_algorithms.h>
#include <bob_toolbox/world_rectangle.h>
#include <bob_grid_map/map_location.h>
#include <bob_grid_map/map_location_set.h>
#include <bob_grid_map/location_mapper.h>

namespace bob
{

	//! \brief Defines a 2D area in the world. Implemented a hash table. The class is a wrapper
	//! for a hash table, but it also inherits from LocationMapper, which gives it a resolution
	//! and associated member functions. It is therefore possible to define areas of different
	//! resolutions. Since the class wraps a hash table, many of the functions are just simple
	//! wrappers, and are often self explanatory.
	class DiscreteArea : public LocationMapper
	{

		public:

			//! \brief Construct an area.
			//! \param resolution The resolution of the pixels in the area.
			DiscreteArea(float resolution) : 
				LocationMapper(resolution) 
		{}

			//! \brief Constructor that allows you to copy data into the area using iterators.
			//! This is an ugly way to do this and should be replaced with a copy constructor.
			//! \param resolution The resolution of the area.
			//! \param from Beginning iterator
			//! \param to The ending iterator
			template <typename Iterator>
				DiscreteArea(float resolution, Iterator from, Iterator to) :
					LocationMapper(resolution),
					mask(from, to)
		{}

			//! \brief Get number of mapLocations in the area
			//! \return The number of mapLocations
			int size() const { return mask.size(); }

			//! \brief Check if area is empty
			//! \return True if the area is empty
			bool empty() const { return mask.empty(); }

			MapLocationSet::size_type count(const MapLocation& mapLocation) const
			{
				return mask.count(mapLocation);
			}	

			void insert(const MapLocation& toInsert) 
			{ 
				mask.insert(toInsert); 
			}

			void erase(const MapLocation& toErase) 
			{ 
				mask.erase(toErase); 
			}

			void clear(void)
			{
				mask.clear();
			}

			template <typename Container>
				void insert(const Container& data)
				{
					for (typename Container::const_iterator itr = data.begin(); itr != data.end(); ++itr)
						mask.insert(*itr);
				}

			template <typename Container>
				void erase(const Container& data)
				{
					for (typename Container::const_iterator itr = data.begin(); itr != data.end(); ++itr)
						mask.erase(*itr);
				}

			typedef MapLocationSet::iterator iterator;
			typedef MapLocationSet::const_iterator const_iterator;

			bool contains(MapLocation mapLocation) const { return (mask.count(mapLocation) == 1); }
			MapLocation arbitraryCell() { return *(mask.begin()); }

			MapLocationSet::iterator begin() { return mask.begin(); }			
			MapLocationSet::iterator end() { return mask.end(); }

			MapLocationSet::const_iterator begin() const { return mask.begin(); }			
			MapLocationSet::const_iterator end() const { return mask.end(); }

			//! \brief Divides the set into subsets. Each subset will have a path to all
			//! the other points in the subset, where the path passes over points horizontally or vertically.
			//! This function is required because of the resolution information associated with a DiscreteArea
			//! object. It is really a wrapper for a function defind elsewhere, but it must attach its resolution
			//! information.
			std::vector<DiscreteArea> fourConnectedSets() const;

			//! \brief Divides the set into subsets. Each subset will have a path to all
			//! the other points in the subset, where the path pases over points in any direction, including diagonals.
			//! 
			template <typename OutputContainer>
				OutputContainer eightConnectedSets() const
				{
					OutputContainer sets;
					std::vector<MapLocationSet> connectedSets = eightConnected(mask);
					for (std::vector<MapLocationSet>::iterator itr = connectedSets.begin(); itr != connectedSets.end(); ++itr)
					{
						DiscreteArea newMask(getResolution());
						newMask.insert(*itr);
						sets.push_back(newMask);
					}
					return sets;
				}

			inline MapLocationSet& getRawData()
			{	
				return mask;
			}

		private:

			MapLocationSet mask;

	};

}

#endif
