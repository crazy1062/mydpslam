#ifndef _PARTICLE_H_
#define _PARTICLE_H_

#include "utils.h"
#include "frwds.h"
#include <memory>
#include <vector>
#include <iostream>



class Particle{
friend class GridMap;
friend class Ancestry;
private:
	int _particle_num, _sample_num;
	TSense sense;
	std::vector<TParticle> particle; //PARTICLE_NUMBER
	std::vector<TParticle> savedParticle; //PARTICLE_NUMBER
	std::vector<TSample> newSample; // SAMPLE_NUMBER
	std::vector<int> children; // PARTICLE_NUMBER
	int cur_particles_used;
	int cur_saved_particles_used;
	std::weak_ptr<GridMap> gweak;
	std::weak_ptr<Ancestry> aweak;
protected:
	bool gptr_is_there();
	bool aptr_is_there();
public:
	Particle();
	Particle(int, int);
	virtual ~Particle();
	void setWPtr(std::shared_ptr<GridMap> gw, std::shared_ptr<Ancestry> aw);
	virtual void InitParticle(TOdo odo, TAncestor *root);
	virtual void FeedSensorData(TSense sense);
	virtual void AddToWorldModel(TSense sense, int particleNum);
	double CheckScore(TSense sense, int index, int sampleNum);
	double QuickScore(TSense sense, int index, int sampleNum);
	virtual void Localize(TSense sense, TOdo odometry, TOdo& lastOdo, int curGeneration);
	virtual void Resample();
	inline double gaussian(double mean, double coeff);
	void moveOneStep(THold h[], int i);
	TAncestor* getBestParticle();
	TAncestor* getLineage(int index);
	virtual void addSavedToAncestry(TSense sense, int curGeneration);
	int getCurUsedPNum();
};



#endif
