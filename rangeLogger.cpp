#include "logger.h"
#include "utils.h"
using namespace std;


rangeLogger::rangeLogger(){
	open = false;
	endLog = false;
}


rangeLogger::rangeLogger(string name) : filename(name){
	cout << "creating range sensor logger ..." << endl;
	open = false;
	endLog = false;
}

rangeLogger::~rangeLogger(){
	cout << "Destroying range sensor logger ..." << endl;
	if(open){
		closeLog();
		open = false;
	}
}

bool rangeLogger::is_open(){
	return open;
}


bool rangeLogger::openLog(){
	iFile.open(filename.c_str());
	
	open = iFile.is_open();
	
	return open;
}

bool rangeLogger::openLog(string name){
	filename = name;
	iFile.open(name.c_str());

	open = iFile.is_open();

	return open;
}


bool rangeLogger::closeLog(){
	iFile.close();
	open = false;
	return true;
}


const vector<TSenseSample> rangeLogger::readLaser(){
	string line;
	vector<TSenseSample> sdata;
	if(open){
		if(!getline(iFile, line, '\n').eof()){
			istringstream reader(line);
			
			istream_iterator<string> begin(reader);
			istream_iterator<string> end;
			if(*begin != "Laser"){
				cerr << "not a laser reading ..." << endl;
				return sdata;
			}
			else begin++;

			int size = stoi(*begin++);
			
			TSenseSample tmp;
			sdata.reserve(size);

			double num;
			for(int i = 0; i < 180; ++i){
				num = stod(*begin++);
				tmp.distance = num;
				sdata.push_back(tmp);
			}
		}
		else endLog = true;
	}
	else cout << "file is not open" << endl;
	
	return sdata;
}

void rangeLogger::readLaser(TSense& sense){
	string line;
	if(open){
		if(!getline(iFile, line, '\n').eof()){
			istringstream reader(line);

			istream_iterator<string> begin(reader);
			istream_iterator<string> end;
			if(*begin != "Laser"){
				cerr << "not a laser reading ..." << endl;
				return;
			}
			else begin++;

			int size = stoi(*begin++);
			
			double num;
			for(int i = 0; i < 180; ++i){
				num = stod(*begin++);
				sense[i].distance = num * MAP_SCALE;
			}
		}
		else endLog = true;
	}
	else cout << "file is not open" << endl;

	return;
}


const TOdo rangeLogger::readOdo(){
	string line;
	TOdo odata;
	if(open){
		if(!getline(iFile, line, '\n').eof()){
			istringstream reader(line);
			
			istream_iterator<string> begin(reader);
			istream_iterator<string> end;
			if(*begin != "Odometry"){
				cerr << "not a odometry reading ..." << endl;
				return odata;
			}
			else begin++;
	
			double num;
			num = stod(*begin++);
			odata.x = num;
			num = stod(*begin++);
			odata.y = num;
			num = stod(*begin++);
			odata.theta = num;

		}
		else endLog = true;
	}
	else cout << "file is not open" << endl;
	
	return odata;
}

void rangeLogger::readOdo(TOdo& odo){
	string line;
	if(open){
		if(!getline(iFile, line, '\n').eof()){
			istringstream reader(line);

			istream_iterator<string> begin(reader);
			istream_iterator<string> end;
			if(*begin != "Odometry"){
				cerr << "not a odometry reading ..." << endl;
				return;
			}
			else begin++;


			double num;
			num = stod(*begin++);
			odo.x = num;
			num = stod(*begin++);
			odo.y = num;
			num = stod(*begin++);
			odo.theta = num;

			if(odo.theta > M_PI)
				odo.theta = odo.theta - 2 * M_PI;
			else if(odo.theta < -M_PI)
				odo.theta = odo.theta + 2 * M_PI;
			
		}
		else endLog = true;
	}
	else cout << "file is not open" << endl;

	return;
}

bool rangeLogger::eofLog() const{
	return endLog;
}
