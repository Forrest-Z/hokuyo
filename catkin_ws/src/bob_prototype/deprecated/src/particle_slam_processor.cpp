#include <bob_slam/particle_slam_processor.h>

#include <ros/ros.h>
#include <limits>

#include <bob_toolbox/geometry.h>
#include <bob_slam/scan_scoring.h>

namespace bob 
{

	ParticleSlamProcessor::Particle::Particle(float resolution) :
		map(resolution), 
		pose(0, 0, 0), 
		weight(0)
	{
		int numberOfMaps = 2;
		for (int i = 0; i < numberOfMaps; ++i)
		{
			lowResMaps.push_back(ConcreteMap(resolution * pow(2, i)));
		}	

	}

	ParticleSlamProcessor::ParticleSlamProcessor() :
	motionModel(config.srr(), config.str(), config.srt(), config.stt()),
	cachedMap(config.delta()),
	mapFiller(5.5, 5.5),
	firstScanProcessed(false),
	cachedMapAvailable(false),
	odomHelper("odom")
	{}

	void ParticleSlamProcessor::setLaserParam(LidarProperties properties)
	{
		// We shouldn't be saving it. This should be fixed later
		lidarProperties = properties;

		laserAngles = std::vector<float>(properties.beamCount);
		float theta = properties.minAngle;

		for(unsigned int i = 0; i < properties.beamCount; i ++)
		{
			if (properties.angleIncrement < 0)
				laserAngles[properties.beamCount - i - 1] = theta;
			else
				laserAngles[i] = theta;
			theta += properties.angleIncrement;
		}
	}

	void ParticleSlamProcessor::init(Pose2D initialPose)
	{
		particles.clear();
		cachedReadings.clear();

		for (unsigned int i = 0; i < config.numParticles(); i++)
		{
			particles.push_back(Particle(config.delta()));
			particles.back().pose = initialPose;
			particles.back().weight = 0;
		}
		neff = (float)config.numParticles();
	}

	void ParticleSlamProcessor::advanceParticles(const Pose2D& odomPose, const Pose2D& lastOdomPose)
	{
		// Update all the particles using the motion model
		for (ParticleVector::iterator it = particles.begin(); it != particles.end(); it++)
		{
			it->pose = Pose2D(it->pose.x + odomPose.x - lastOdomPose.x, it->pose.y + odomPose.y - lastOdomPose.y, it->pose.theta + odomPose.theta - lastOdomPose.theta);//motionModel.drawFromMotion(it->pose, odomPose, lastOdomPose);
		}
	}

	void ParticleSlamProcessor::addScanToParticles(const LaserReading& reading)
	{
		bool scanMatchSuccess = true;
		if (firstScanProcessed)
		{
			// Scan match the particles against their existing maps, which modifies their pose and weight
			scanMatchSuccess = scanMatchParticles(reading);
		
			normalizeParticleWeights();
		}

		if (scanMatchSuccess)
		{
			// Cache the reading
			cachedReadings.push_back(reading);
		
			// Lengthen the particle trajectory with the current particle pose
			extendParticleTrajectories();

			// Update each particle's map with the reading data
			buildParticleMaps(reading);
		
		}
	

		firstScanProcessed = true;
	}

	ParticleSlamProcessor::Particle& ParticleSlamProcessor::getBestParticle()
	{
		unsigned int bestIndex;
		float bestWeight = -std::numeric_limits<float>::max();
		for (unsigned int i = 0; i < particles.size(); i++)
		{
			if (bestWeight < particles[i].weight)
			{
				bestWeight = particles[i].weight;
				bestIndex = i;
			}
		}
		return particles[bestIndex];
	}

	bool ParticleSlamProcessor::scanMatchParticles(std::vector<float> laserReading)
	{
		bool scanMatchSuccess = true;

		// Sample a new pose from each scan in the reference
		for (ParticleVector::iterator it = particles.begin(); it != particles.end(); it++)
		{
			Pose2D correctedPose = it->pose;
			for (std::vector<ConcreteMap>::iterator mapItr = it->lowResMaps.begin(); mapItr != it->lowResMaps.end(); ++mapItr)
			{
				correctedPose = scanMatcher.scanMatchPose(*mapItr, correctedPose, laserReading, lidarProperties);
			}
			correctedPose = scanMatcher.scanMatchPose(it->map, correctedPose, laserReading, lidarProperties);
			//Pose2D correctedPose = m_matcher.scanMatchPose(it->map, it->pose, laserReading, laserAngles);

			float score = scoreScan(it->map, correctedPose, laserReading, laserAngles);

			float normalizedScore = score/lidarProperties.beamCount;
			if (normalizedScore > config.minimumScore())
			{
				// Scan matching succeeded
				it->pose = correctedPose;
				it->weight += score;
			}
			else
			{
				scanMatchSuccess = false;
			}
		}
		return scanMatchSuccess;
	}

	// Normalizes the particle weights so that the max weight is 1 and the rest have a weight less than one,
	// as caulcated based on an exponential equation.	
	// Neff is also recalculated
	void ParticleSlamProcessor::normalizeParticleWeights()
	{
		// Find the maximum weight
		float maxWeight = -std::numeric_limits<float>::max();
		for (ParticleVector::iterator it = particles.begin(); it != particles.end(); it++)
		{
			maxWeight = it->weight > maxWeight ? it->weight : maxWeight;
		}

		// Calculate new m_weights
		float weightSum = 0;
		float gain = 1.0 / (config.ogain() * particles.size());
		for (std::vector<Particle>::iterator it = particles.begin(); it != particles.end(); it++)
		{
			it->weight = exp(gain * (it->weight - maxWeight));
			weightSum += it->weight;
		}

		// Compute Neff
		/*
		neff = 0;
		for (std::vector<float>::iterator it = m_weights.begin(); it != m_weights.end(); it++)
		{
			*it = *it / weightSum;
			float w = *it;
			neff += w * w;
		}
		neff = 1.0 / neff;
		*/
	}

	void ParticleSlamProcessor::flattenMap()
	{
		// Note that we are building the map off of the previous cachedMap!
		cachedMap = map();

		Particle bestParticle = getBestParticle();
		bestParticle.trajectory.clear();
	
		particles.clear();
		cachedReadings.clear();

		for (unsigned int i = 0; i < config.numParticles(); i++)
		{
			particles.push_back(bestParticle);
			particles.back().weight = 0;
		}
		cachedMapAvailable = true;
	}

	void ParticleSlamProcessor::resample(const std::vector<float> reading)
	{
		/*
		TNodeVector oldGeneration;
		for (unsigned int i = 0; i < particles.size(); i++)
		{
			oldGeneration.push_back(particles[i].node);
		}

		// Resampling
		uniform_resampler<float, float> resampler;
		std::vector<unsigned int> indexes = resampler.resampleIndexes(m_weights, 0);

		// Building tree
		ParticleVector temp;
		unsigned int j = 0;
		std::vector<unsigned int> deletedParticles;  		//this is for deleteing the particles which have been resampled away.

		// Existing nodes
		for (unsigned int i = 0; i < indexes.size(); i++)
		{
			while(j < indexes[i])
			{
				deletedParticles.push_back(j);
				j++;
			}
			if (j == indexes[i])
				j++;

			Particle& p = particles[indexes[i]];
			TNode * node = 0;
			TNode * oldNode = oldGeneration[indexes[i]];
			node = new TNode(p.pose, 0, oldNode, 0);
			node->reading = reading;

			temp.push_back(p);
			temp.back().node = node;
		}

		while(j < indexes.size())
		{
			deletedParticles.push_back(j);
			j++;
		}

		// Deleting nodes
		for (unsigned int i = 0; i < deletedParticles.size(); i++)
		{
			delete particles[deletedParticles[i]].node;
			particles[deletedParticles[i]].node = 0;
		}

		// Deleting old particls
		particles.clear();

		// Copying particles and registering scans
		for (ParticleVector::iterator it = temp.begin(); it != temp.end(); it++)
		{
			it->weight = 0;
			mapFiller.updateMap(it->map, it->pose, reading);
			particles.push_back(* it);
		}
		*/
	}

	void ParticleSlamProcessor::extendParticleTrajectories()
	{
		// Registering scans
		for (ParticleVector::iterator it = particles.begin(); it != particles.end(); it++)
		{
			// Create a new node in the particle tree and add it to the old tree
			it->trajectory.push_back(it->pose);
		}
	}

	void ParticleSlamProcessor::buildParticleMaps(const std::vector<float>& reading)
	{
		// Registering scans
		for (ParticleVector::iterator it = particles.begin(); it != particles.end(); it++)
		{
			for (std::vector<ConcreteMap>::iterator mapItr = it->lowResMaps.begin(); mapItr != it->lowResMaps.end(); ++mapItr)
			{
				mapFiller.updateMap(*mapItr, it->pose, reading, laserAngles, false);
			}

			// Tree built
			mapFiller.updateMap(it->map, it->pose, reading, laserAngles, false);
		}
	}

	ConcreteMap ParticleSlamProcessor::map()
	{
		Particle best = getBestParticle();
		ConcreteMap smap(cachedMap);

		ros::Time start = ros::Time::now();

		std::vector<std::vector<float> >::iterator readingItr = cachedReadings.begin();
		for(ParticleSlamProcessor::Trajectory::iterator pointItr = best.trajectory.begin(); pointItr != best.trajectory.end(); ++pointItr, ++readingItr)
		{
			mapFiller.updateMap(smap, *pointItr, *readingItr, laserAngles, true);
		}
		return smap;
	}

	float ParticleSlamProcessor::computePoseEntropy()
	{
		float weight_total = 0.0;
		for(std::vector<ParticleSlamProcessor::Particle>::const_iterator it = getParticles().begin();
				it != getParticles().end();
				++ it)
		{
			weight_total += it->weight;
		}
		float entropy = 0.0;
		for(std::vector<ParticleSlamProcessor::Particle>::const_iterator it = getParticles().begin();
				it != getParticles().end();
				++ it)
		{
			if(it->weight/weight_total > 0.0)
				entropy += it->weight/weight_total * log(it->weight/weight_total);
		}
		return -entropy;
	}
}
