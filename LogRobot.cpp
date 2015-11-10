#include "logger.h"
#include "robot.h"
#include "singleton.h"
#include <iostream>

using namespace std;

typedef singleton<rangeLogger> logSingleton;

template<class S, class O>
LogRobot<S, O>::LogRobot() {}

template<class S, class O>
LogRobot<S, O>::LogRobot(string name): logname(name) {
	logSingleton::instance()->openLog(logname);
}

template<class S, class O>
LogRobot<S, O>::~LogRobot(){
	logSingleton::instance()->closeLog();
	logSingleton::destroy();
}

template<class S, class O>
bool LogRobot<S, O>::connectSensor(){ 
	return logSingleton::instance()->is_open();
}

template<class S, class O>
bool LogRobot<S, O>::connectOdometry(){
	return logSingleton::instance()->is_open();
}

template<class S, class O>
bool LogRobot<S, O>::connectDrive(){
	return true;
}

template<class S, class O>
void LogRobot<S, O>::drive(){
	return;
}

template<class S, class O>
void LogRobot<S, O>::getSensor(S& sense){
	logSingleton::instance()->readLaser(sense);
}

template<class S, class O>
void LogRobot<S, O>::getOdometry(O& odo){
	logSingleton::instance()->readOdo(odo);
}

template<class S, class O>
bool LogRobot<S, O>::eofSense(){
	return logSingleton::instance()->eofLog();
}
