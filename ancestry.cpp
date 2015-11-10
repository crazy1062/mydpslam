#include "utils.h"
#include "ancestry.h"
#include "particle.h"
#include "gridmap.h"

using namespace std;

Ancestry::Ancestry() {}

Ancestry::Ancestry(int id_num): _id_num(id_num){
	particleID.reserve(_id_num);
	for(int i = 0; i < _id_num; ++i){
		particleID.push_back(TAncestor());
	}

	availableID.reserve(_id_num);
	for(int i = 0; i < _id_num; ++i){
		availableID.push_back(0);
	}
}

Ancestry::~Ancestry() {}

void Ancestry::setWPtr(shared_ptr<GridMap> gw, shared_ptr<Particle> pw){
	gweak = gw;
	pweak = pw;
}

bool Ancestry::gptr_is_there(){
	if(gweak.expired()){
		cout << "gridmap is gone! " << endl;
		return false;
	}
	return true;
}

bool Ancestry::pptr_is_there(){
	if(pweak.expired()){
		cout << "particle is gone! " << endl;
		return false;
	}
	return true;
}

void Ancestry::InitAncestry(){
	int i;
	cleanID = _id_num - 2;

	for(i = 0; i < _id_num; ++i){
		availableID[i] = i;

		particleID[i].generation = -1;
		particleID[i].numChildren = 0;
		particleID[i].ID = -1;
		particleID[i].parent = NULL;
		particleID[i].mapEntries = NULL;
		particleID[i].path = NULL;
		particleID[i].seen = 0;
		particleID[i].total = 0;
		particleID[i].size = 0;
	}

	particleID[_id_num - 1].generation = 0;
	particleID[_id_num - 1].numChildren = 1;
	particleID[_id_num - 1].size = 0;
	particleID[_id_num - 1].total = 0;
	particleID[_id_num - 1].ID = _id_num - 1;
	particleID[_id_num - 1].parent = NULL;
	particleID[_id_num - 1].mapEntries = NULL;
	
	cout << "Initialize ancestry successfully ..." << endl;
}


TAncestor* Ancestry::getRootNode(){
	
	return &particleID[_id_num - 1];
}

void Ancestry::ResizeArray(TMapStarter* node, int deadID){
	int i, j, ID, x, y;
	int hash[_id_num];
	int source, last;
	TMapNode *temp;
	shared_ptr<GridMap> gshared;
	if(gptr_is_there()) gshared = gweak.lock();
	else return;

	if(deadID >= 0) node->dead++;

	node->size = (int)(ceil((node->total - node->dead) * 1.75));
	temp = new TMapNode[node->size];

	for(i = 0; i < _id_num; ++i) hash[i] = -1;
	
	j = 0;
	for(i = 0; i < node->total; ++i){

		if(node->array[i].ID == deadID){
			particleID[deadID].mapEntries[node->array[i].source].node = -1;
		}
		else if(hash[node->array[i].ID] == -1){
			temp[j] = node->array[i];

			particleID[temp[j].ID].mapEntries[temp[j].source].node = j;
			hash[node->array[i].ID] = j;
			++j;
		}
		else if(node->array[i].distance > temp[hash[node->array[i].ID]].distance){
			ID = node->array[i].ID;
			source = temp[hash[ID]].source;

			particleID[ID].total--;
			last = particleID[ID].total;
			particleID[ID].mapEntries[source] = particleID[ID].mapEntries[last];

			x = particleID[ID].mapEntries[source].x;
			y = particleID[ID].mapEntries[source].y;

			
			if((gshared->gmap[x][y] == node) && (particleID[ID].mapEntries[source].node < i))
				temp[hash[ID]].source = source;
			else
				gshared->gmap[x][y]->array[particleID[ID].mapEntries[source].node].source = source;

			temp[hash[ID]].source = node->array[i].source;
			temp[hash[ID]].hits = node->array[i].hits;
			temp[hash[ID]].distance = node->array[i].distance;

			particleID[ID].mapEntries[node->array[i].source].node = hash[ID];
		}
		else{
			if(node->array[i].parentGen != -1)
				temp[hash[node->array[i].ID]].parentGen = node->array[i].parentGen;
			ID = node->array[i].ID;
			source = node->array[i].source;

			particleID[ID].total--;
			last = particleID[ID].total;

			if(last != source){
				particleID[ID].mapEntries[source] = particleID[ID].mapEntries[last];

				x = particleID[ID].mapEntries[source].x;
				y = particleID[ID].mapEntries[source].y;
				
				
				if((gshared->gmap[x][y] == node) && (particleID[ID].mapEntries[source].node <= i))
					temp[hash[ID]].source = source;
				else
					gshared->gmap[x][y]->array[particleID[ID].mapEntries[source].node].source = source;
			}
		}
	}

	node->total = j;
	node->dead = 0;
	delete [] node->array;
	node->array = temp;
}



void Ancestry::DisposeAncestry(){
	int i, j;
	TPath *tempPath, *trashPath;
	TEntryList *entry;
	shared_ptr<GridMap> gshared;
	if(gptr_is_there()) gshared = gweak.lock();
	else return;

	for(i = 0; i < _id_num; ++i){
		if(particleID[i].ID == i){
			entry = particleID[i].mapEntries;

			for(j = 0; j < particleID[i].total; ++j){
				gshared->DeleteObservation(entry[j].x, entry[j].y, entry[j].node);
			}
			
			delete [] entry;
			particleID[i].mapEntries = NULL;

			tempPath = particleID[i].path;

			while(tempPath != NULL){
				trashPath = tempPath;
				tempPath = tempPath->next;
				delete trashPath;
			}

			particleID[i].path = NULL;
			particleID[i].ID = -123;
		}

		for(cleanID = 0; cleanID < _id_num; ++cleanID)
			availableID[cleanID] = cleanID;

		cleanID = _id_num;
	}


}


void Ancestry::UpdateAncestry(TSense sense, int curGeneration, bool low){
	int i, j;
	TAncestor *temp, *hold, *parentNode;
	TEntryList *workArray, *entry;
	TMapStarter *node;
	TPath *tempPath, *trashPath;
	shared_ptr<Particle> pshared;
	if(pptr_is_there()) pshared = pweak.lock();
	else return;
	shared_ptr<GridMap> gshared;
	if(gptr_is_there()) gshared = gweak.lock();
	else return;

	// Remove dead nodes: low - path, high - no path
	int cur_particles_used = pshared->getCurUsedPNum();

	for(i = 0; i < cur_particles_used; ++i){
		//temp = pshared->getLineage(i);
		temp = pshared->particle[i].ancestryNode;

		while(temp->numChildren == 0){
			for(j = 0; j < temp->total; ++j){
				gshared->DeleteObservation(temp->mapEntries[j].x, temp->mapEntries[j].y, temp->mapEntries[j].node);
			}

			delete [] temp->mapEntries;
			temp->mapEntries = NULL;


			if(low){
				tempPath = temp->path;
				
				while(tempPath != NULL){
					trashPath = tempPath;
					tempPath = tempPath->next;
					delete trashPath;
				}
				temp->path = NULL;
			}

			++cleanID;
			availableID[cleanID] = temp->ID;
			temp->generation = curGeneration;
			temp->ID = -42;

			hold = temp;
			temp = temp->parent;
			hold->parent = NULL;

			temp->numChildren--;
		}	
	}
	
	for(i = 0; i < _id_num - 1; ++i){
		if((particleID[i].ID == i) && (particleID[i].parent != NULL) && (particleID[i].parent->numChildren == 1)){

			// identical part
			while(particleID[i].parent->generation == -111){
				particleID[i].parent = particleID[i].parent->parent;
			}

			parentNode = particleID[i].parent;

			if(parentNode->size < (parentNode->total + particleID[i].total)){
				parentNode->size = (int)(ceil((parentNode->size + particleID[i].size) * 1.75));

				workArray = new TEntryList[parentNode->size];
				for(j = 0; j < parentNode->total; ++j){
					workArray[j] = parentNode->mapEntries[j];
				}

				delete [] parentNode->mapEntries;
				parentNode->mapEntries = workArray;
			}

			entry = particleID[i].mapEntries;

			for(j = 0; j < particleID[i].total; ++j){
				//node = gshared->getMapStarter(entry[j].x, entry[j].y);
				node = gshared->gmap[entry[j].x][entry[j].y];

				node->array[entry[j].node].ID = parentNode->ID;
				node->array[entry[j].node].source = parentNode->total;

				parentNode->mapEntries[parentNode->total] = entry[j];
				parentNode->total++;

				if(node->array[entry[j].node].parentGen >= parentNode->generation){
					node->array[entry[j].node].parentGen = -1;
					node->dead++;
				}
			}

			for(j = 0; j < particleID[i].total; ++j){
				
				//node = gshared->getMapStarter(entry[j].x, entry[j].y);
				node = gshared->gmap[entry[j].x][entry[j].y];
				if((node->total - node->dead) * 2.5 < node->size)
					ResizeArray(node, -7);
			}

			
			delete [] particleID[i].mapEntries;
			particleID[i].mapEntries = NULL;

			// not identical part
			if(low){
				tempPath = parentNode->path;
				while(tempPath->next != NULL){
					tempPath = tempPath->next;
				}

				tempPath->next = particleID[i].path;
				particleID[i].path = NULL;
			}

			// idential part
			parentNode->numChildren = particleID[i].numChildren;

			particleID[i].generation = -111;
		}
	}


	for(i = 0; i < _id_num - 1; ++i){
		if(particleID[i].ID == i){
			while(particleID[i].parent->generation == -111){
				particleID[i].parent = particleID[i].parent->parent;
			}
		}
	}


	if(low) gshared->InitFlags();

	// not idential particle::addSavedToAncestry()
	pshared->addSavedToAncestry(sense, curGeneration);

	
	// identical part
	for(i = 0; i < _id_num - 1; ++i){
		if(particleID[i].generation == -111){
			particleID[i].generation = -1;
			particleID[i].numChildren = 0;
			particleID[i].parent = NULL;
			particleID[i].mapEntries = NULL;
			particleID[i].path = NULL;
			particleID[i].seen = 0;
			particleID[i].total = 0;
			particleID[i].size = 0;
			
			++cleanID;
			availableID[cleanID] = i;
			particleID[i].ID = -3;
		}
	}

}



TAncestor* Ancestry::popAvailableNode(){
	TAncestor *node = &(particleID[availableID[cleanID]]);
	node->ID = availableID[cleanID];
	--cleanID;
	if(cleanID < 0){
		cerr << "Insufficient Number of Particle IDs: Abandon Ship!" << endl;
		cleanID = 0;
	}

	return node;
}



TAncestor* Ancestry::getNode(int index){
	TAncestor *node = &(particleID[index]);

	return node;
}
