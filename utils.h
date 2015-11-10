#ifndef _UTILS_H_
#define _UTILS_H_

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <cmath>

#define SENSE_NUMBER 180

struct TSense_struct{
	double theta, distance;
	
	TSense_struct(){
		theta = 0.0;
		distance = 0.0;
	}
};

typedef TSense_struct TSenseSample;
typedef TSenseSample TSense[SENSE_NUMBER];

struct odo_struct{
	double x, y, theta;

	odo_struct(){
		x = 0.0;
		y = 0.0;
		theta = 0.0;
	}

};

typedef odo_struct TOdo;



#define MAP_SCALE 35

#define MAX_SENSE_RANGE 7.95*MAP_SCALE

#define TURN_RADIUS (0.40 * MAP_SCALE)

#define LOW_VARIANCE (0.017 * MAP_SCALE*MAP_SCALE)
#define HIGH_VARIANCE (0.17 * MAP_SCALE*MAP_SCALE)




#define UNKNOWN -2

#define START_ITERATION 0

#define MAP_WIDTH 1700
#define MAP_HEIGHT 1700

#define PARTICLE_NUMBER 50

#define SAMPLE_NUMBER (PARTICLE_NUMBER*10)

#define ID_NUMBER (int) (PARTICLE_NUMBER*2.25)

#define H_MAP_WIDTH 3000
#define H_MAP_HEIGHT 3000

#define H_PARTICLE_NUMBER 50

#define H_SAMPLE_NUMBER (H_PARTICLE_NUMBER*10)

#define H_ID_NUMBER (int) (H_SAMPLE_NUMBER*2.25)

#if (PARTICLE_NUMBER > H_PARTICLE_NUMBER)
#define TOP_ID_NUMBER ID_NUMBER
#else
#define TOP_ID_NUMBER H_ID_NUMBER
#endif

#define AREA (int) (3.1415 * 55.0 * MAP_SCALE*MAP_SCALE)

#define L_PRIOR (-1.0/(MAP_SCALE*8.0))

#define L_PRIOR_DIST 4.0




struct TPath_struct{
	float C, D, T;
	struct TPath_struct *next;
};
typedef struct TPath_struct TPath;


struct TSenseLog_struct{
	TSense sense;
	struct TSenseLog_struct *next;
};
typedef struct TSenseLog_struct TSenseLog;


struct MapNode_struct;
struct MapNode_struct{
	int source;
	float distance;
	int hits;
	int ID;
	int parentGen;
};
typedef struct MapNode_struct TMapNode;
typedef struct MapNode_struct *PMapNode;


struct MapNodeStarter_struct;
struct MapNodeStarter_struct{
	int total, size, dead;
	PMapNode array;
};
typedef struct MapNodeStarter_struct TMapStarter;
typedef struct MapNodeStarter_struct *PMapStarter;


struct TEntryList_struct;
struct TEntryList_struct{
	int node;
	int x, y;
};
typedef struct TEntryList_struct TEntryList;

struct TAncestor_struct;
struct TAncestor_struct{
	struct TAncestor_struct *parent;
	TEntryList *mapEntries;
	int size, total;
	int generation, ID, numChildren;
	TPath *path;
	char seen;
};
typedef struct TAncestor_struct TAncestor;
typedef struct TAncestor_struct *PAncestor;

struct TParticle_struct{
	float x, y, theta;
	float C, D, T;
	double probability;
	TAncestor_struct *ancestryNode;
};
typedef struct TParticle_struct TParticle;


#define meanC_D -0.0107
#define meanC_T 0.0061
#define varC_D 0.0630
#define varC_T 2.2992

#define meanD_D 0.9577
#define meanD_T -0.1731
#define varD_D 0.1560
#define varD_T 1.9924

#define meanT_D -0.0003
#define meanT_T 0.9437
#define varT_D 0.0008
#define varT_T 0.1405

#define THRESH 13.0

#define PASSES 9

#define MAX_TRACE_ERROR exp(-24.0/LOW_VARIANCE)

#define WORST_POSSIBLE -10000

#define LOW_DURATION 40

struct THold{
	TSense sense;
	double C, D, T;
};

struct TSample_struct{
	double x, y, theta;
	double C, D, T;
	double probability;
	int parent;
};
typedef struct TSample_struct TSample;




#endif
