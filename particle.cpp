#include "utils.h"
#include "particle.h"
#include "gridmap.h"
#include "ancestry.h"
#include <random>
#include <cfloat>

using namespace std;

Particle::Particle() {}

Particle::Particle(int particle_num, int sample_num): _particle_num(particle_num), _sample_num(sample_num){
	particle.reserve(_particle_num);
	savedParticle.reserve(_particle_num);
	newSample.reserve(_sample_num);
	children.reserve(_particle_num);

	for(int i = 0; i < _particle_num; ++i){
		particle.push_back(TParticle());
		savedParticle.push_back(TParticle());
		children.push_back(0);
	}

	for(int i = 0; i < _sample_num; ++i){
		newSample.push_back(TSample());
	}
}

Particle::~Particle() {}

void Particle::setWPtr(shared_ptr<GridMap> gw, shared_ptr<Ancestry> aw){
	gweak = gw;
	aweak = aw;
}

void Particle::InitParticle(TOdo odo, TAncestor *root){
	int i;
	shared_ptr<Ancestry> ashared;
	if(aptr_is_there()) ashared = aweak.lock();
	else return;

	for(i = 0; i < _particle_num; ++i){
		particle[i].ancestryNode = &(ashared->particleID[ashared->_id_num - 1]);
		particle[i].x = MAP_WIDTH / 2;
		particle[i].y = MAP_HEIGHT / 2;
		particle[i].theta = 0.001;
		particle[i].probability = 0;
		children[i] = 0;
	}

	particle[0].probability = 1;
	children[0] = _sample_num;
	cur_saved_particles_used = 0;
	cur_particles_used = 1;

	cout << "Initialize particle successfully ..." << endl;
}


void Particle::FeedSensorData(TSense s){
	int i;
	for(i = 0; i < SENSE_NUMBER; ++i){
		sense[i] = s[i];
	}

	return;
}

bool Particle::gptr_is_there(){
	if(gweak.expired()){
		cout << "the GridMap is gone! " << endl;
		return false;
	}
	return true;
}

bool Particle::aptr_is_there(){
	if(aweak.expired()){
		cout << "the ancestry is gone! " << endl;
		return false;
	}
	return true;
}

void Particle::AddToWorldModel(TSense sense, int particleNum){
	int j;
	shared_ptr<GridMap> gshared;
	if(gptr_is_there()) gshared = gweak.lock();
	else return;

	for(j = 0; j < SENSE_NUMBER; ++j){
		//cout << sense[j].theta << " " << sense[j].distance << endl;
		gshared->AddTrace(particle[particleNum].x, particle[particleNum].y, sense[j].distance, (sense[j].theta + particle[particleNum].theta), particle[particleNum].ancestryNode->ID, (sense[j].distance < MAX_SENSE_RANGE));
	}
}

double Particle::CheckScore(TSense sense, int index, int sampleNum){
	double a;
	shared_ptr<GridMap> gshared;
	if(gptr_is_there()) gshared = gweak.lock();
	else return 0;

	a = gshared->LineTrace(newSample[sampleNum].x, newSample[sampleNum].y, (sense[index].theta + newSample[sampleNum].theta), sense[index].distance, particle[newSample[sampleNum].parent].ancestryNode->ID, 0);

	return max(MAX_TRACE_ERROR, a);
}

double Particle::QuickScore(TSense sense, int index, int sampleNum){
	double distance, eval;
	shared_ptr<GridMap> gshared;
	if(gptr_is_there()) gshared = gweak.lock();
	else return 0;

	if(sense[index].distance >= MAX_SENSE_RANGE) return 1;

	distance = max(0.0, sense[index].distance - 3.5);
	eval = gshared->LineTrace((int)(newSample[sampleNum].x + (cos(sense[index].theta + newSample[sampleNum].theta) * distance)), (int)(newSample[sampleNum].y + (sin(sense[index].theta + newSample[sampleNum].theta) * distance)), (sense[index].theta + newSample[sampleNum].theta), 3.5, particle[newSample[sampleNum].parent].ancestryNode->ID, 3);

	return max(MAX_TRACE_ERROR, eval);
}

double Particle::gaussian(double mean, double coeff){
	random_device rd;
	mt19937 e2(rd());
	normal_distribution<> dist(mean, coeff);
	return dist(e2);
}

void Particle::Localize(TSense sense, TOdo odometry, TOdo& lastOdo, int curGeneration){
	double ftemp;
	double threshold;
	double total;
	double turn, distance, moveAngle;
	double CCenter, DCenter, TCenter, CCoeff, DCoeff, TCoeff;
	double tempC, tempD;
	int i, j, k, p, best;
	int keepers = 0;
	int newchildren[_sample_num];


	distance = sqrt( pow(odometry.x - lastOdo.x, 2) + pow(odometry.y - lastOdo.y, 2) ) * MAP_SCALE;
	turn = (odometry.theta - lastOdo.theta);


	if(turn > M_PI/3) turn = turn - 2*M_PI;
	else if(turn < -M_PI/3) turn = turn + 2*M_PI;

	CCenter = distance * meanC_D + turn * meanC_T;
	DCenter = distance * meanD_D + turn * meanD_T;
	TCenter = distance * meanT_D + turn * meanT_T;

	CCoeff = max((fabs(distance * varC_D) + fabs(turn * varC_T)), 0.8);	
	DCoeff = max((fabs(distance * varD_D) + fabs(turn * varD_T)), 0.8);
	TCoeff = max((fabs(distance * varT_D) + fabs(turn * varT_T)), 0.10);

	i = 0;
	
	//cout << "turn: " << turn << endl;

	for(j = 0; j < _particle_num; ++j){
		for(k = 0; k < children[j]; ++k){
			newSample[i].parent = j;

			tempC = gaussian(CCenter, CCoeff);
			tempD = gaussian(DCenter, DCoeff);

			newSample[i].C = tempC;
			newSample[i].D = tempD;
			newSample[i].T = gaussian(TCenter, TCoeff);
			newSample[i].theta = particle[j].theta + newSample[i].T;

			moveAngle = (newSample[i].theta + particle[j].theta) / 2.0;

			newSample[i].x = particle[j].x + (TURN_RADIUS * (cos(newSample[i].theta) - cos(particle[j].theta))) + (tempD * cos(moveAngle)) + (tempC * cos(moveAngle + M_PI/2));
			newSample[i].y = particle[j].y + (TURN_RADIUS * (sin(newSample[i].theta) - sin(particle[j].theta))) + (tempD * sin(moveAngle)) + (tempC * sin(moveAngle + M_PI/2));
			newSample[i].probability = 0.0;
			
			++i;	
		}
	}

	threshold = WORST_POSSIBLE - 1;
	
	for(p = 0; p < PASSES; ++p){
		best = 0;

		for(i = 0; i < _sample_num; ++i){
			if(newSample[i].probability >= threshold){
				for(k = p; k < SENSE_NUMBER; k += PASSES){
					newSample[i].probability += log(QuickScore(sense, k, i));
				}

				if(newSample[i].probability > newSample[best].probability) best = i;

			}
			else{
				newSample[i].probability = WORST_POSSIBLE;
			}
		}
		
		threshold = newSample[best].probability - THRESH;
	}

	keepers = 0;
	
	for(i = 0; i < _sample_num; ++i){
		if(newSample[i].probability >= threshold){
			++keepers;
			newSample[i].probability = 0.0;
		}
		else{
			newSample[i].probability = WORST_POSSIBLE;
		}
	}

	cout << "Better: " << keepers << " ";
	
	threshold = -1;
	keepers = 0;

	for(p = 0; p < PASSES; ++p){
		best = 0;

		for(i = 0; i < _sample_num; ++i){
			if(newSample[i].probability >= threshold){
				if(p == PASSES - 1){
					++keepers;
				}

				for(k = p; k < SENSE_NUMBER; k += PASSES){
					newSample[i].probability += log(CheckScore(sense, k, i));
				}
					
				if(newSample[i].probability > newSample[best].probability) best = i;
				
			}
			else{
				newSample[i].probability = WORST_POSSIBLE;
			}
		}

		threshold = newSample[best].probability - THRESH;
	}

	cout << "Best of: " << keepers << " ";

	total = 0.0;
	threshold = newSample[best].probability;
	for(i = 0; i < _sample_num; ++i){
		if(newSample[i].probability == WORST_POSSIBLE)
			newSample[i].probability = 0.0;
		else{
			newSample[i].probability = exp(newSample[i].probability - threshold);
			total += newSample[i].probability;
		}
	}

	for(i = 0; i < _sample_num; ++i)
		newSample[i].probability /= total;

	total = 0.0;

	for(i = 0; i < _sample_num; ++i){
		newchildren[i] = 0;
		total += newSample[i].probability;
	}

	i = j = 0;

	random_device rd;
	mt19937 mt(rd());
	uniform_real_distribution<double> dist(0, nextafter(1, DBL_MAX));

	while((j < _sample_num) && (i < _particle_num)){
		k = 0;
		ftemp = total * dist(mt);

		while(ftemp > (newSample[k].probability)){
			ftemp = ftemp - newSample[k].probability;
			++k;
		}

		if(newchildren[k] == 0) ++i;

		newchildren[k]++;
		++j;

	}

	cout << "kept: " << i << endl;

	for(i = 0; i < _particle_num; ++i){
		children[i] = 0;
		savedParticle[i].probability = 0.0;
	}

	best = 0;
	k = 0;
	for(i = 0; i < _sample_num; ++i){
		if(newchildren[i] > 0){
			savedParticle[k].probability = newSample[i].probability;
			savedParticle[k].x = newSample[i].x;
			savedParticle[k].y = newSample[i].y;
			savedParticle[k].theta = newSample[i].theta;
			savedParticle[k].C = newSample[i].C;
			savedParticle[k].D = newSample[i].D;
			savedParticle[k].T = newSample[i].T;

			savedParticle[k].ancestryNode = particle[newSample[i].parent].ancestryNode;
			savedParticle[k].ancestryNode->numChildren++;
			children[k] = newchildren[i];

			if(savedParticle[k].probability > savedParticle[best].probability) best = k;
			++k;
		}
	}

	cur_saved_particles_used = k;

	if(j < _sample_num){
		total = 0.0;

		for(i = 0; i < cur_saved_particles_used; ++i){
			total += savedParticle[i].probability;
		}

		for(i = 0; i < cur_saved_particles_used; ++i){
			savedParticle[i].probability /= total;
		}

		total = 0.0;

		for(i = 0; i < cur_saved_particles_used; ++i){
			total += savedParticle[i].probability;
		}

		while(j < _sample_num){
			k = 0;
			ftemp = total * dist(mt);

			while(ftemp > (savedParticle[k].probability)){
				ftemp = ftemp - savedParticle[k].probability;
				++k;
			}
			children[k]++;

			++j;
		}
	}

	cout << "curGeneration: " << curGeneration << " best particle: " << savedParticle[best].x << " " << savedParticle[best].y << " " << savedParticle[best].theta << " " << savedParticle[best].probability << endl;


}

void Particle::Resample(){

}


void Particle::moveOneStep(THold h[], int i){
	double moveAngle;

	moveAngle = particle[0].theta + (h[i].T / 2.0);
	particle[0].x = particle[0].x + (TURN_RADIUS * (cos(particle[0].theta + h[i].T) - cos(particle[0].theta))) + (h[i].D * cos(moveAngle)) + (h[i].C * cos(moveAngle + M_PI/2));
	particle[0].y = particle[0].y + (TURN_RADIUS * (sin(particle[0].theta + h[i].T) - sin(particle[0].theta))) + (h[i].D * sin(moveAngle)) + (h[i].C * sin(moveAngle + M_PI/2));
	particle[0].theta += h[i].T;

}

TAncestor* Particle::getBestParticle(){
	int j = 0;
	int i;

	for(i = 0; i < cur_particles_used; ++i){
		if(particle[i].probability > particle[j].probability)
			j = i;
	}

	TAncestor *lineage = particle[j].ancestryNode;

	return lineage;

}

TAncestor* Particle::getLineage(int index){

	return particle[index].ancestryNode;
}


void Particle::addSavedToAncestry(TSense sense, int curGeneration){
	int i, j;
	TPath *tempPath, *trashPath;
	TAncestor* temp;
	shared_ptr<Ancestry> ashared;
	if(aptr_is_there()) ashared = aweak.lock();
	else return;

	j = 0;
	for(i = 0; i < cur_saved_particles_used; ++i){
		while(savedParticle[i].ancestryNode->generation == -111){
			savedParticle[i].ancestryNode = savedParticle[i].ancestryNode->parent;
		}

		if(savedParticle[i].ancestryNode->numChildren == 1){
			savedParticle[i].ancestryNode->generation = curGeneration;
			savedParticle[i].ancestryNode->numChildren = 0;
			particle[j].ancestryNode = savedParticle[i].ancestryNode;

			trashPath = new TPath;
			trashPath->C = savedParticle[i].C;
			trashPath->D = savedParticle[i].D;
			trashPath->T = savedParticle[i].T;
			trashPath->next = NULL;
			tempPath = particle[i].ancestryNode->path;

			while(tempPath->next != NULL){
				tempPath = tempPath->next;
			}
			tempPath->next = trashPath;

			particle[j].x = savedParticle[i].x;
			particle[j].y = savedParticle[i].y;
			particle[j].theta = savedParticle[i].theta;
			particle[j].probability = savedParticle[i].probability;
			++j;

		}
		else if(savedParticle[i].ancestryNode->numChildren > 0){
			
			//temp = &(particleID[availableID[cleanID]]);
			//temp->ID = availableID[cleanID];
			//--cleanID;

			//temp = ashared->popAvailableNode();

			temp = &(ashared->particleID[ashared->availableID[ashared->cleanID]]);
			temp->ID = ashared->availableID[ashared->cleanID];
			--ashared->cleanID;
			if(ashared->cleanID < 0){ 
				ashared->cleanID = 0;
				cout << "Insufficient clean ID" << endl;
			}

			temp->parent = savedParticle[i].ancestryNode;
			temp->mapEntries = NULL;
			temp->total = 0;
			temp->size = 0;

			temp->generation = curGeneration;
			temp->numChildren = 0;
			temp->seen = 0;

			trashPath = new TPath;
			trashPath->C = savedParticle[i].C;
			trashPath->D = savedParticle[i].D;
			trashPath->T = savedParticle[i].T;
			trashPath->next = NULL;
			temp->path = trashPath;

			particle[j].ancestryNode = temp;
			particle[j].x = savedParticle[i].x;
			particle[j].y = savedParticle[i].y;
			particle[j].theta = savedParticle[i].theta;

			particle[j].probability = savedParticle[i].probability;
			++j;
		}
	}

	cur_particles_used = cur_saved_particles_used;

	//cout << "current particles used: " << cur_particles_used << endl;

	for(i = 0; i < cur_particles_used; ++i)
		AddToWorldModel(sense, i);

	//cout << "finished add to map" << endl;

	temp = NULL;
}

int Particle::getCurUsedPNum(){
	return cur_particles_used;
}
