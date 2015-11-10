#ifndef _GRIDMAP_H_
#define _GRIDMAP_H_

#include <vector>
#include <iostream>
#include <memory>
#include "utils.h"
#include "frwds.h"



class GridMap{
friend class Ancestry;
friend class Particle;
private:
	int flagMap[H_MAP_WIDTH][H_MAP_WIDTH];
	int obsX[AREA], obsY[AREA];
	int observationArray[AREA][TOP_ID_NUMBER];
	int observationID;	
	int _width, _height, _particle_num, _id_num;
	std::vector<std::vector<PMapStarter>> gmap;
	std::weak_ptr<Particle> pweak;
	std::weak_ptr<Ancestry> aweak;

protected:
	void AddToWorkingArray(int i, TMapStarter *node, int workingArray[]);
	//virtual void ResizeArray(TMapStarter* node, int deadID);

	bool pptr_is_there();
	bool aptr_is_there();

public:
	GridMap();
	GridMap(int, int, int, int);
	virtual ~GridMap();
	void setWPtr(std::shared_ptr<Particle> pw, std::shared_ptr<Ancestry> aw);
	virtual void InitFlags();
	virtual void InitWorldMap();
	virtual void DestroyMap();
	virtual void BuildObservation(int x, int y, char usage);
	virtual void UpdateGridSquare(int x, int y, double distance, int hit, int parentID);
	virtual void DeleteObservation(int x, int y, int node);
	virtual double ComputeProbability(int x, int y, double distance, int parentID);
	virtual double ComputeProb(int x, int y, double distance, int ID);
	virtual void AddTrace(double startx, double starty, double MeasuredDist, double theta, int parentID, int addEnd);
	virtual double LineTrace(double startx, double starty, double theta, double MeasuredDist, int parentID, float culling);
	TMapStarter* getMapStarter(int x, int y);
};

#endif
