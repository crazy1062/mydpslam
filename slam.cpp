#include "utils.h"
#include "frwds.h"
#include "gridmap.h"
#include "particle.h"
#include "ancestry.h"
#include "slam.h"

using namespace std;

slam::slam() {}

slam::slam(string name): logname(name){
	//lbot = new LogRobot<TSense, TOdo>(logname);
	map_ptr = make_shared<GridMap>(MAP_WIDTH, MAP_HEIGHT, PARTICLE_NUMBER, ID_NUMBER);
	particle_ptr = make_shared<Particle>(PARTICLE_NUMBER, SAMPLE_NUMBER);
	tree_ptr = make_shared<Ancestry>(ID_NUMBER);
	map_ptr->setWPtr(particle_ptr, tree_ptr);
	particle_ptr->setWPtr(map_ptr, tree_ptr);
	tree_ptr->setWPtr(map_ptr, particle_ptr);
}

slam::~slam(){
	
}

void slam::InitSlam(){
	int i;
	lbot = new LogRobot<TSense, TOdo>(logname);

	for(i = 0; i < SENSE_NUMBER; ++i){
		sense[i].theta = (i * M_PI / 180.0) - M_PI / 2;
	}

	curGeneration = 0;
	lbot->getOdometry(odometry);
	lbot->getSensor(sense);

	continueSlam = 1;

	cout << "Initialize slam successfully... " << endl;
}

void slam::CloseSlam(){
	delete lbot;
}

void slam::ProcessSlam(){

}

void slam::PrintSense(){
	int i;
	cout << "print current sense data: " << endl;
	for(i = 0; i < SENSE_NUMBER; ++i)
		cout << sense[i].distance << " ";
	cout << endl;
}

void slam::PrintOdometry(){
	int i;
	cout << "print current odometry data: " << endl;
	cout << odometry.x << " " << odometry.y << " " << odometry.theta << endl;
}

void slam::ProcessSlam(int& continueSlam, TPath **path, TSenseLog **obs){
	
	int counter;
	int i, j, overflow = 0;
	char name[32];
	TPath *tempPath;
	TSenseLog *tempObs;
	TAncestor *lineage;

	map_ptr->InitWorldMap();

	tree_ptr->InitAncestry();

	root = tree_ptr->getRootNode();

	particle_ptr->InitParticle(odometry, root);
	lastOdo.x = odometry.x;
	lastOdo.y = odometry.y;
	lastOdo.theta = odometry.theta;

	overflow = 1;


	if(curGeneration == 0){
		cout << "generation 0" << endl;
		//PrintSense();
		//PrintOdometry();
		particle_ptr->AddToWorldModel(sense, 0);
		cout << "first add world model ..." << endl;

		for(i = 0; i < SENSE_NUMBER; ++i){
			hold[0].sense[i].distance = sense[i].distance;
			hold[0].sense[i].theta = sense[i].theta;
		}
		++curGeneration;
	}
	else{
		
		map_ptr->InitFlags();

		particle_ptr->AddToWorldModel(hold[(int)(LOW_DURATION * 0.5)].sense, 0);
		for(i = (int)(LOW_DURATION * 0.5) + 1; i < LOW_DURATION; ++i){
			map_ptr->InitFlags();

			particle_ptr->moveOneStep(hold, i);

			particle_ptr->AddToWorldModel(hold[i].sense, 0);
		}

		for(i = 0; i < SENSE_NUMBER; ++i){
			hold[0].sense[i].distance = hold[LOW_DURATION - 1].sense[i].distance;
			hold[0].sense[i].theta = hold[LOW_DURATION - 1].sense[i].theta;
		}
	}

	(*obs) = new TSenseLog;
	for(i = 0; i < SENSE_NUMBER; ++i){
		(*obs)->sense[i].distance = hold[0].sense[i].distance;
		(*obs)->sense[i].theta = hold[0].sense[i].theta;
	}
	(*obs)->next = NULL;

	continueSlam = 1;
	counter = 0;

	while((continueSlam) && (counter < LOW_DURATION)){
		
		if(!lbot->eofSense()){
			lbot->getOdometry(odometry);
			lbot->getSensor(sense);
			overflow = 1;
			continueSlam = 1;
		}else{
			overflow = 0;
			continueSlam = 0;
		}

		if((sqrt(pow(odometry.x - lastOdo.x, 2) + pow(odometry.y - lastOdo.y, 2)) < 0.05) && (fabs(odometry.theta - lastOdo.theta) < 0.03)) overflow = 0;

		if(overflow > 0){
			--overflow;
			
			map_ptr->InitFlags();

			//particle_ptr->FeedSensorData(sense);

			particle_ptr->Localize(sense, odometry, lastOdo, curGeneration);

			//cout << "start to update ancestry" << endl;
			tree_ptr->UpdateAncestry(sense, curGeneration, true);
			//cout << "finished update" << endl;

			tempObs = (*obs);
			while(tempObs->next != NULL)
				tempObs = tempObs->next;

			tempObs->next = new TSenseLog;

			for(i = 0; i < SENSE_NUMBER; ++i){
				tempObs->next->sense[i].distance = sense[i].distance;
				tempObs->next->sense[i].theta = sense[i].theta;
			}
			tempObs->next->next = NULL;

			for(i = 0; i < SENSE_NUMBER; ++i){
				hold[counter].sense[i].distance = sense[i].distance;
				hold[counter].sense[i].theta = sense[i].theta;
			}
			
			++curGeneration;
			++counter;
			
			lastOdo.x = odometry.x;
			lastOdo.y = odometry.y;
			lastOdo.theta = odometry.theta;
		}
	}


	lineage = particle_ptr->getBestParticle();
	
	cout << "ID: " << lineage->ID << endl;
	cout << "numChildren: " << lineage->numChildren << endl;
	cout << "generation: " << lineage->generation << endl;
	cout << "size: " << lineage->size << endl;
	cout << "total: " << lineage->total << endl;
	(*path) = NULL;
	i = 0;
	while((lineage != NULL) && (lineage->ID != ID_NUMBER - 1)){
		tempPath = lineage->path;
		++i;
		
		while(tempPath->next != NULL){
			++i;
			tempPath = tempPath->next;
		}
		tempPath->next = (*path);

		(*path) = lineage->path;
		lineage->path = NULL;
		lineage = lineage->parent;
	}

	tempPath = (*path);
	i = 0;
	while(tempPath != NULL){
		hold[i].C = tempPath->C;
		hold[i].D = tempPath->D;
		hold[i].T = tempPath->T;
		tempPath = tempPath->next;
		++i;
	}

	lineage = particle_ptr->getBestParticle();
	
	sprintf(name, "lmap%.2d", (int) (curGeneration / LOW_DURATION) - 1);
	PrintMap(name, lineage);
	sprintf(name, "rm lmap%.2d.ppm", (int) (curGeneration / LOW_DURATION) - 1);
	system(name);

	//fix me later
	//continueSlam = 0;


	tree_ptr->DisposeAncestry();
	map_ptr->DestroyMap();
}

void slam::PrintMap(char* name, TAncestor *parent){
	FILE *printFile;
	int x, y, i;
	int width, height;
	int startx, starty, lastx, lasty;
	char sysCall[128];
	double hit, theta;

	width = MAP_WIDTH;
	height = MAP_HEIGHT;

	for(x = 0; x < width; ++x)
		for(y = 0; y < height; ++y)
			map[x][y] = 0;

	lastx = 0;
	lasty = 0;
	startx = width - 1;
	starty = height - 1;

	for(x = 0; x < width; ++x){
		for(y = 0; y < height; ++y){
			hit = map_ptr->ComputeProb(x, y, 1.4, parent->ID);
			if(hit == UNKNOWN)
				map[x][y] = 255;
			else{
				map[x][y] = (int)(230 - (hit * 230));

				if(x > lastx) lastx = x;
				if(y > lasty) lasty = y;
				if(x < startx) startx = x;
				if(y < starty) starty = y;
			}
		}
	}

	sprintf(sysCall, "%s.ppm", name);
	printFile = fopen(sysCall, "w");
	fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", lastx - startx + 1, lasty - starty + 1);
	fprintf(printFile, "255\n");

	for(y = lasty; y >= starty; --y){
		for(x = startx; x <= lastx; ++x){
			if(map[x][y] == 254)
				fprintf(printFile, "%c%c%c", 255, 0, 0);
			else if(map[x][y] == 253)
				fprintf(printFile, "%c%c%c", 0, 255, 200);
			else if(map[x][y] == 252)
				fprintf(printFile, "%c%c%c", 255, 55, 55);
			else if(map[x][y] == 251)
				fprintf(printFile, "%c%c%c", 50, 150, 255);
			else if(map[x][y] == 250)
				fprintf(printFile, "%c%c%c", 250, 200, 200);
			else if(map[x][y] == 0)
				fprintf(printFile, "%c%c%c", 100, 250, 100);
			else
				fprintf(printFile, "%c%c%c", map[x][y], map[x][y], map[x][y]);
		}
	}

	fclose(printFile);
	sprintf(sysCall, "convert %s.ppm %s.png", name, name);
	system(sysCall);
	sprintf(sysCall, "chmod 666 %s.ppm", name);
	system(sysCall);
	sprintf(sysCall, "chmod 666 %s.png", name);
	system(sysCall);
	fprintf(stderr, "Map dumped to file\n");

}

