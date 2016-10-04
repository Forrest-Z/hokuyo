#ifndef GRIDSLAMPROCESSOR_H
#define GRIDSLAMPROCESSOR_H

#include <vector>
#include <bob_toolbox/pose2d.h>
#include <bob_toolbox/world_point.h>
#include <bob_slam/binary_search_scan_matcher.h>
#include <bob_visualization/visualization.h>
#include "motion_model.h"
#include <bob_config/particle_slam_config.h>
#include <bob_sensor/iodometry_handle.h>
#include <bob_lidar/laser_reading.h>

#include <bob_lidar/lidar_properties.h>
#include <bob_slam/map_filler.h>
#include <bob_slam/icp_scan_matcher.h>

namespace bob 
{

	//! THIS CLASS IS DEPRECATED!
	//! Particle SLAM algorithm is too slow to use in our application
	//! The code shouldn't be deleted because work was undertaken to refactor it
	class ParticleSlamProcessor
	{

		public:

			typedef std::vector<Pose2D> Trajectory;

			struct Particle
			{

				Particle(float resolution);

				std::vector<ConcreteMap> lowResMaps;
				ConcreteMap map;

				Pose2D pose;

				float weight;

				Trajectory trajectory;

			};

			typedef std::vector<Particle> ParticleVector;

			void init(Pose2D initialPose=Pose2D(0,0,0));

			inline const ParticleVector& getParticles() const { return particles; }

			Particle& getBestParticle();

			ParticleSlamProcessor();

			void setLaserParam(LidarProperties properties);

			ConcreteMap map();

			void advanceParticles(const Pose2D& odomPose, const Pose2D& lastOdomPose);

			void flattenMap();

			void addScanToParticles(const LaserReading& reading);


			int numCachedScans()
			{
				return cachedReadings.size();
			}

			const ConcreteMap& getCachedMap()
			{
				return cachedMap;
			}

			bool isCachedMapAvailable()
			{
				return cachedMapAvailable;
			}

		private:

			LidarProperties lidarProperties;

			ParticleSlamConfig config;

			//BinarySearchScanMatcher m_matcher;
			ICPScanMatcher scanMatcher;
			MapFiller mapFiller;

			ParticleVector particles;

			MotionModel motionModel;

			bool firstScanProcessed;
			bool cachedMapAvailable;

			float neff;

			bool scanMatchParticles(const std::vector<float> plainReading);

			void normalizeParticleWeights();

			void resample(const std::vector<float> reading);

			void extendParticleTrajectories();

			std::vector<LaserReading> cachedReadings;

			void buildParticleMaps(const std::vector<float>& reading);

			float computePoseEntropy();

			OdometryHelper odomHelper;

			ConcreteMap cachedMap;

			std::vector<float> laserAngles;
	};

};

#endif
