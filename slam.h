#ifndef _SLAM_H_
#define _SLAM_H_

#include "utils.h"
#include "frwds.h"
#include "gridmap.h"
#include "ancestry.h"
#include "particle.h"
#include "robot.h"
#include <string>
#include <iostream>
#include <memory>

class slam{
private:
	LogRobot<TSense, TOdo>* lbot;
	std::shared_ptr<GridMap> map_ptr;
	std::shared_ptr<Particle> particle_ptr;
	std::shared_ptr<Ancestry> tree_ptr;
	TAncestor* root;

	int curGeneration;
	TSense sense;
	TOdo odometry;
	std::string logname;
	TOdo lastOdo;

	unsigned char map[MAP_WIDTH][MAP_HEIGHT];
	THold hold[LOW_DURATION];

	int continueSlam;
public:
	slam();
	slam(std::string);
	~slam();
	void PrintMap(char* name, TAncestor *parent);
	void InitSlam();
	void CloseSlam();
	void ProcessSlam();
	void ProcessSlam(int& continueSlam, TPath **path, TSenseLog **obs);
	void PrintSense();
	void PrintOdometry();
};



#endif
