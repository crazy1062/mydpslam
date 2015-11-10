#include "slam.h"
#include <iostream>

using namespace std;

int main(){

	TPath *path, *trashPath;
	TSenseLog *obs, *trashObs;

	const string name = "Data/loop5.log";

	slam* lowslam = new slam(name);

	int continueSlam = 1;

	lowslam->InitSlam();

	lowslam->ProcessSlam(continueSlam, &path, &obs);

	while(path != NULL){
		trashPath = path;
		path = path->next;
		delete trashPath;
	}

	while(obs != NULL){
		trashObs = obs;
		obs = obs->next;
		delete trashObs;
	}

	lowslam->CloseSlam();


	delete lowslam;

}
