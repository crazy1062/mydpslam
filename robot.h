#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "logger.h"
#include "singleton.h"
#include <string>

template<class S, class O>
class robot{
public:
	robot() {};
	virtual ~robot() {};
	virtual bool connectSensor() = 0;
	virtual bool connectOdometry() = 0;
	virtual bool connectDrive() = 0;
	virtual void getSensor(S& sense) = 0;
	virtual void getOdometry(O& odo) = 0;
	virtual void drive() = 0;
};


template<class S, class O>
class LogRobot: public robot<S, O>{
private:
	std::string logname;
public:
	LogRobot();
	LogRobot(std::string);
	virtual ~LogRobot();
	virtual bool connectSensor();
	virtual bool connectOdometry();
	virtual bool connectDrive();
	virtual void getSensor(S& sense);
	virtual void getOdometry(O& odo);
	virtual void drive();
	bool eofSense();
};

#include "LogRobot.cpp"

#endif
