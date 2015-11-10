#include "utils.h"
#include "particle.h"
#include "ancestry.h"
#include "gridmap.h"
#include <string>

using namespace std;

GridMap::GridMap() {}

GridMap::GridMap(int width, int height, int particle_num, int id_num): _width(width), _height(height), _particle_num(particle_num), _id_num(id_num){
	//gmap.reserve(_width);
	for(int i = 0; i < _width; ++i){
		//gmap[i].reserve(_height);
		gmap.push_back(vector<PMapStarter>(_height));
	}

	cout << "Initialize gridmap successfully ... " << endl;
}

GridMap::~GridMap() {}

void GridMap::setWPtr(shared_ptr<Particle> pw, shared_ptr<Ancestry> aw){
	pweak = pw;
	aweak = aw;
}

bool GridMap::pptr_is_there(){
	if(pweak.expired()){
		cout << "particle object is gone!" << endl;
		return false;
	}
	return true;
}

bool GridMap::aptr_is_there(){
	if(aweak.expired()){
		cout << "ancestry object is gone!" << endl;
		return false;
	}
	return true;
}

// safe function
void GridMap::InitFlags(){
	while(observationID > 0){
		--observationID;
		flagMap[obsX[observationID]][obsY[observationID]] = 0;
		obsX[observationID] = 0;
		obsY[observationID] = 0;
	}
	observationID = 1;
}

// safe function
void GridMap::InitWorldMap(){
	int x, y;

	for(y = 0; y < _height; ++y){
		for(x = 0; x < _width; ++x){
			flagMap[x][y] = 0;
			gmap[x][y] = NULL;
		}
	}

	for(x = 0; x < AREA; ++x){
		obsX[x] = 0;
		obsY[x] = 0;
	}

	observationID = 1;
}

// safe function
void GridMap::DestroyMap(){
	int x, y;

	for(y = 0; y < _height; ++y){
		for(x = 0; x < _width; ++x){
			while(gmap[x][y] != NULL){
				delete [] gmap[x][y]->array;
				delete gmap[x][y];
				gmap[x][y] = NULL;
			}
		}
	}
}




void GridMap::AddToWorkingArray(int i, TMapStarter *node, int workingArray[]){
	int j, source, last;
	TEntryList *entries;
	shared_ptr<Ancestry> ashared;
	if(aptr_is_there()) ashared = aweak.lock();
	else return;


	if(workingArray[node->array[i].ID] == -1){
		workingArray[node->array[i].ID] = i;
	}
	else{
		if(node->array[i].distance < node->array[workingArray[node->array[i].ID]].distance){
			j = i;
			if(node->array[i].parentGen >= 0){
				node->array[workingArray[node->array[i].ID]].parentGen = node->array[i].parentGen;
			}
		}
		else{
			j = workingArray[node->array[i].ID];
			workingArray[node->array[i].ID] = i;
			if(node->array[j].parentGen >= 0){
				node->array[i].parentGen = node->array[j].parentGen;
			}
		}
		
		
		ashared->particleID[node->array[j].ID].total--;

		entries = ashared->particleID[node->array[j].ID].mapEntries;
		source = node->array[j].source;
		last = ashared->particleID[node->array[j].ID].total;

		if(last != source){
			entries[source] = entries[last];

			gmap[entries[source].x][entries[source].y]->array[entries[source].node].source = source;
		}

		node->total--;
		node->dead--;

		if(j != node->total){
			node->array[j] = node->array[node->total];

			ashared->particleID[node->array[j].ID].mapEntries[node->array[j].source].node = j;

			if(workingArray[node->array[j].ID] == node->total)
				workingArray[node->array[j].ID] = j;
			else if(i != node->total)
				AddToWorkingArray(j, node, workingArray);
		}

	}

	return;
}


void GridMap::BuildObservation(int x, int y, char usage){
	TAncestor *lineage;
	PAncestor stack[_particle_num];
	int workingArray[_id_num + 1];
	int i, here, topStack;
	char flag;
	shared_ptr<Ancestry> ashared;
	if(aptr_is_there()) ashared = aweak.lock();
	else return;
	shared_ptr<Particle> pshared;
	if(pptr_is_there()) pshared = pweak.lock();
	else return;

	int cur_particles_used = pshared->getCurUsedPNum();

	if(observationID >= AREA) cout << "aRoll over!" << endl;

	flagMap[x][y] = observationID;
	obsX[observationID] = x;
	obsY[observationID] = y;
	++observationID;
	here = flagMap[x][y];

	for(i = 0; i < _id_num; ++i){
		observationArray[here][i] = -1;
		workingArray[i] = -1;
		ashared->particleID[i].seen = 0;
	}

	for(i = 0; i < gmap[x][y]->total; ++i){
		AddToWorkingArray(i, gmap[x][y], workingArray);
	}

	if(usage){
		flag = 1;
		for(i = 0; i < gmap[x][y]->total; ++i){
			if(gmap[x][y]->array[i].hits > 0)
				flag = 0;
			else
				workingArray[gmap[x][y]->array[i].ID] = -2;
		}
	}

	for(i = 0; i < cur_particles_used; ++i){
		lineage = pshared->particle[i].ancestryNode;
		topStack = 0;

		while((lineage != NULL) && (lineage->seen == 0)){
			stack[topStack] = lineage;
			++topStack;

			lineage->seen = 1;
			lineage = lineage->parent;
		}

		while(topStack > 0){
			--topStack;
			lineage = stack[topStack];

			if((workingArray[lineage->ID] == -1) && (lineage->parent != NULL)){
				workingArray[lineage->ID] = workingArray[lineage->parent->ID];
				if(workingArray[lineage->ID] == -1)
					flag = 0;
			}
		}
	}

	if((usage) && (flag))
		flagMap[x][y] = -2;
	else
		for(i = 0; i < _id_num; ++i)
			observationArray[here][i] = workingArray[i];

	

	return;
}



void GridMap::UpdateGridSquare(int x, int y, double distance, int hit, int parentID){
	
	int here, i;
	shared_ptr<Ancestry> ashared;
	if(aptr_is_there()) ashared = aweak.lock();
	else return;

	TEntryList *tempEntry;


	if(gmap[x][y] == NULL){
		if(observationID >= AREA) cout << "bRoll over!" << endl;

		flagMap[x][y] = observationID;
		obsX[observationID] = x;
		obsY[observationID] = y;
		++observationID;

		gmap[x][y] = new TMapStarter;
		gmap[x][y]->dead = 0;
		gmap[x][y]->total = 0;
		gmap[x][y]->size = 1;
		gmap[x][y]->array = new TMapNode;


		for(i = 0; i < _id_num; ++i)
			observationArray[flagMap[x][y]][i] = -1;
	}
	else if(flagMap[x][y] == 0)
		BuildObservation(x, y, 0);


	here = observationArray[flagMap[x][y]][parentID];
	
	if((here != -1) && (gmap[x][y]->array[here].ID == parentID)){
		gmap[x][y]->array[here].hits += hit;
		gmap[x][y]->array[here].distance += distance;
	}
	else{
		if(gmap[x][y]->size <= gmap[x][y]->total){
			ashared->ResizeArray(gmap[x][y], -71);
			if(gmap[x][y]->total == 0){
				delete [] gmap[x][y]->array;
				delete gmap[x][y];
				gmap[x][y] = NULL;
			}
		}


		observationArray[flagMap[x][y]][parentID] = gmap[x][y]->total;


		if(ashared->particleID[parentID].size == 0){
			ashared->particleID[parentID].size = 1;
			ashared->particleID[parentID].mapEntries = new TEntryList;
		}
		else if(ashared->particleID[parentID].size <= ashared->particleID[parentID].total){
			ashared->particleID[parentID].size = (int)(ceil(ashared->particleID[parentID].total * 1.25));
			tempEntry = new TEntryList[ashared->particleID[parentID].size];
			for(i = 0; i < ashared->particleID[parentID].total; ++i){
				tempEntry[i] = ashared->particleID[parentID].mapEntries[i];
			}

			delete [] ashared->particleID[parentID].mapEntries;
			ashared->particleID[parentID].mapEntries = tempEntry;
		}

		ashared->particleID[parentID].mapEntries[ashared->particleID[parentID].total].x = x;
		ashared->particleID[parentID].mapEntries[ashared->particleID[parentID].total].y = y;
		ashared->particleID[parentID].mapEntries[ashared->particleID[parentID].total].node = gmap[x][y]->total;

		i = gmap[x][y]->total;

		gmap[x][y]->array[i].source = ashared->particleID[parentID].total;
		gmap[x][y]->array[i].ID = parentID;
		ashared->particleID[parentID].total++;

		if(here == -1){
			gmap[x][y]->array[i].hits = hit;
			gmap[x][y]->array[i].distance = distance + L_PRIOR_DIST;
			gmap[x][y]->array[i].parentGen = -2;
		}
		else{
			
			gmap[x][y]->array[i].hits = gmap[x][y]->array[here].hits + hit;
			gmap[x][y]->array[i].distance = gmap[x][y]->array[here].distance + distance;
			gmap[x][y]->array[i].parentGen = ashared->particleID[gmap[x][y]->array[here].ID].generation;
		}

		gmap[x][y]->total++;
	}

	
}


void GridMap::DeleteObservation(int x, int y, int node){
	int total;
	shared_ptr<Ancestry> ashared;
	if(aptr_is_there()) ashared = aweak.lock();
	else return;
	

	if((node == -1) || (gmap[x][y] == NULL)) return;

	if((gmap[x][y]->total - gmap[x][y]->dead) == 1){
		delete [] gmap[x][y]->array;
		delete gmap[x][y];
		gmap[x][y] = NULL;
		return;
	}

	if((int)((gmap[x][y]->total - 1 - gmap[x][y]->dead)*2.5) <= gmap[x][y]->size){
		ashared->ResizeArray(gmap[x][y], gmap[x][y]->array[node].ID);
		if(gmap[x][y]->total == 0){
			delete [] gmap[x][y]->array;
			delete gmap[x][y];
			gmap[x][y] = NULL;
		}
		return;
	}

	gmap[x][y]->total--;

	total = gmap[x][y]->total;

	if(node != total){
		gmap[x][y]->array[node] = gmap[x][y]->array[total];
		
		ashared->particleID[gmap[x][y]->array[node].ID].mapEntries[gmap[x][y]->array[node].source].node = node;
	}

	
}

double GridMap::ComputeProbability(int x, int y, double distance, int parentID){

	if(gmap[x][y] == NULL) return (1.0 - exp(L_PRIOR * distance));

	if(flagMap[x][y] == 0) BuildObservation(x, y, 1);

	if(flagMap[x][y] == -2) return 0;

	if(observationArray[flagMap[x][y]][parentID] == -1)
		return (1.0 - exp(L_PRIOR * distance));

	if(observationArray[flagMap[x][y]][parentID] == -2)
		return 0;

	if(gmap[x][y]->array[observationArray[flagMap[x][y]][parentID]].hits == 0) return 0;

	return (1.0 - exp(-(gmap[x][y]->array[observationArray[flagMap[x][y]][parentID]].hits / gmap[x][y]->array[observationArray[flagMap[x][y]][parentID]].distance) * distance));

}


double GridMap::ComputeProb(int x, int y, double distance, int ID){
	int i;
	shared_ptr<Ancestry> ashared;
	if(aptr_is_there()) ashared = aweak.lock();
	else return 0;

	if(gmap[x][y] == NULL) return UNKNOWN;

	while(1){
		for(i = 0; i < gmap[x][y]->total; ++i){
			if(gmap[x][y]->array[i].ID == ID){
				if(gmap[x][y]->array[i].hits == 0)
					return 0;
				return (1.0 - exp(-(gmap[x][y]->array[i].hits / gmap[x][y]->array[i].distance) * distance));
			}
		}


		if(ashared->particleID[ID].parent == NULL)
			return UNKNOWN;
		else
			ID = ashared->particleID[ID].parent->ID;
	}

	return UNKNOWN;
}


void GridMap::AddTrace(double startx, double starty, double MeasuredDist, double theta, int parentID, int addEnd){
	double overflow, slope;
	int x, y, incX, incY, endx, endy;
	int xedge, yedge;
	double dx, dy;
	double distance, error;
	double secant, cosecant;

	secant = 1.0/fabs(cos(theta));
	cosecant = 1.0/fabs(sin(theta));

	distance = min(MeasuredDist, MAX_SENSE_RANGE);
	if(distance < 1.0) cout << "not enough distance" << endl;

	dx = (startx + (cos(theta) * distance));
	dy = (starty + (sin(theta) * distance));
	endx = (int) (dx);
	endy = (int) (dy);

	if(startx > dx){
		incX = -1;
		xedge = 1;
	}else{
		incX = 1;
		xedge = 0;
	}

	if(starty > dy){
		incY = -1;
		yedge = 1;
	}else{
		incY = 1;
		yedge = 0;
	}

	if(fabs(startx - dx) > fabs(starty - dy)){
		y = (int) (starty);
		overflow = starty - y;

		if(incY == 1) overflow = 1.0 - overflow;
		slope = fabs(tan(theta));

		error = fabs(((int)(startx) + incX + xedge) - startx);
		overflow = overflow - (slope * error);

		if(overflow < 0.0){
			y += incY;
			overflow += 1.0;
		}

		for(x = (int)(startx) + incX; x != endx; x += incX){
			overflow -= slope;

			if(overflow < 0.0)
				distance = (overflow + slope) * cosecant;
			else
				distance = fabs(slope) * cosecant;
			UpdateGridSquare(x, y, distance, 0, parentID);


			if(overflow < 0){
				y += incY;
				distance = -overflow * cosecant;
				overflow += 1.0;
				UpdateGridSquare(x, y, distance, 0, parentID);
			}
		}

		if(addEnd){
			if(incX < 0)
				distance = fabs((x + 1) - dx) * secant;
			else
				distance = fabs(dx - x) * secant;
			UpdateGridSquare(endx, endy, distance, 1, parentID);
		}
	}
	else{
		x = (int) (startx);
		overflow = startx - x;

		if(incX == 1) overflow = 1.0 - overflow;
		slope = 1.0 / fabs(tan(theta));

		error = fabs(((int)(starty) + incY + yedge) - starty);
		overflow = overflow - (slope * error);

		if(overflow < 0.0){
			x += incX;
			overflow += 1.0;
		}

		for(y = (int)(starty) + incY; y != endy; y += incY){
			overflow -= slope;

			if(overflow < 0)
				distance = (overflow + slope) * secant;
			else
				distance = fabs(slope) * secant;
			UpdateGridSquare(x, y, distance, 0, parentID);

			if(overflow < 0.0){
				x += incX;
				distance = -overflow * secant;
				overflow += 1.0;
				UpdateGridSquare(x, y, distance, 0, parentID);
			}
		}

		if(addEnd){
			if(incY < 0)
				distance = fabs(((y + 1) - dy) / sin(theta));
			else
				distance = fabs((dy - y) / sin(theta));
			UpdateGridSquare(endx, endy, distance, 1, parentID);
		}
	}

}



double GridMap::LineTrace(double startx, double starty, double theta, double MeasuredDist, int parentID, float culling){
	double overflow, slope;
	int x, y, incX, incY, endx, endy;
	double dx, dy;
	double totalProb;

	double eval;

	double prob, distance, error;
	double secant, cosecant;
	double xblock, yblock;
	double xMotion, yMotion;
	double standardDist;

	eval = 0.0;

	totalProb = 1.0;

	secant = 1.0 / fabs(cos(theta));
	cosecant = 1.0 / fabs(sin(theta));



	if(culling) distance = MeasuredDist + culling;
	else distance = min(MeasuredDist + 20.0, MAX_SENSE_RANGE);
	if(distance < 1.0) cout << "not enough distance" << endl;

	dx = (startx + (cos(theta) * distance));
	dy = (starty + (sin(theta) * distance));
	endx = (int) (dx);
	endy = (int) (dy);
	
	if(startx > dx){
		incX = -1;
		xblock = -startx;
	}else{
		incX = 1;
		xblock = 1.0 - startx;
	}

	if(starty > dy){
		incY = -1;
		yblock = -starty;
	}else{
		incY = 1;
		yblock = 1.0 - starty;
	}

	if(fabs(startx - dx) > fabs(starty - dy)){
		y = (int) (starty);

		overflow = starty - y;
		if(incY == 1) overflow = 1.0 - overflow;

		slope = fabs(tan(theta));
		if(slope > 1.0) slope = fabs((starty - dy) / (startx - dx));

		dx = fabs((int)(startx) + xblock);
		dy = fabs(tan(theta) * dx);

		if(overflow - dy < 0.0){
			y += incY;
			overflow = overflow - dy + 1.0;
		}
		else{
			overflow = overflow - dy;
		}

		standardDist = slope * cosecant;

		xMotion = -fabs(fabs(( ((int)(startx)) + xblock) * secant) - MeasuredDist);
		yMotion = -fabs(fabs((y + yblock) * cosecant) - MeasuredDist);

		for(x = (int)(startx) + incX; x != endx; x += incX){
			xMotion += secant;
			overflow -= slope;

			if(overflow < 0.0) distance = (overflow + slope) * cosecant;
			else distance = standardDist;

			prob = totalProb * ComputeProbability(x, y, distance, parentID);
			if(prob > 0){
				if(overflow < 0.0) error = fabs(yMotion);
				else error = fabs(xMotion);

				if(error < 20.0) eval += (prob * exp(-(error * error) / (2 * LOW_VARIANCE)));

				totalProb = totalProb - prob;
			}

			if(overflow < 0.0){
				y += incY;
				yMotion += cosecant;

				distance = -overflow * cosecant;
				overflow += 1.0;

				prob = totalProb * ComputeProbability(x, y, distance, parentID);
				if(prob > 0){
					error = fabs(xMotion);
					if(error < 20.0) eval += (prob * exp(-(error * error) / (2 * LOW_VARIANCE)));
				}

				totalProb = totalProb - prob;
			}
		}
	}
	else{
		x = (int) (startx);

		overflow = startx - x;
		if(incX == 1) overflow = 1.0 - overflow;

		slope = 1.0 / fabs(tan(theta));
		

		dy = fabs((int)(starty) + yblock);
		dx = fabs(dy / tan(theta));

		if(overflow - dx < 0){
			x += incX;
			overflow = overflow - dx + 1.0;
		}
		else{
			overflow = overflow - dx;
		}

		standardDist = slope * secant;

		yMotion = -fabs(fabs(( ((int)(starty)) + yblock) * cosecant) - MeasuredDist);
		xMotion = -fabs(fabs((x + xblock) * secant) - MeasuredDist);

		for(y = (int)(starty) + incY; y != endy; y += incY){
			yMotion += cosecant;
			overflow -= slope;

			if(overflow < 0.0) distance = (overflow + slope) * secant;
			else distance = standardDist;

			prob = totalProb * ComputeProbability(x, y, distance, parentID);
			if(prob > 0){
				if(overflow < 0.0) error = fabs(xMotion);
				else error = fabs(yMotion);

				if(error < 20.0) eval += (prob * exp(-(error * error) / (2 * LOW_VARIANCE)));

			}
			totalProb = totalProb - prob;

			if(overflow < 0.0){
				x += incX;
				xMotion += secant;

				distance = -overflow * secant;
				overflow += 1.0;

				prob = totalProb * ComputeProbability(x, y, distance, parentID);
				if(prob > 0){
					error = fabs(yMotion);
					if(error < 20.0) eval += (prob * exp(-(error * error) / (2 * LOW_VARIANCE)));
				}

				totalProb = totalProb - prob;
			}
		}
	}

	if(MeasuredDist >= MAX_SENSE_RANGE) return (eval + totalProb);

	if(totalProb == 1) return 0;
	
	return (eval / (1.0 - totalProb));

}


TMapStarter* GridMap::getMapStarter(int x, int y){
	
	return gmap[x][y];
	
}



