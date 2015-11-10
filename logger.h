#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iterator>
#include <vector>
#include "utils.h"

class logger{
public:
	logger() {};
	virtual ~logger() {};
	virtual bool openLog() = 0;
	virtual bool closeLog() = 0;
};


class rangeLogger: public logger{
private:
	std::string filename;
	std::ifstream iFile;
	TSense sensor_data;
	TOdo odometry;
	bool open;
	bool endLog;
public:
	rangeLogger();
	rangeLogger(std::string);
	virtual bool openLog(std::string name);
	virtual ~rangeLogger();
	virtual bool openLog();
	virtual bool closeLog();
	bool is_open();
	const std::vector<TSenseSample> readLaser();
	const TOdo readOdo();
	void readLaser(TSense& sense);
	void readOdo(TOdo& odo);
	bool eofLog() const;
	void writeLaser(TSense const&);
	void writeOdo(TOdo const&);
};


#endif
