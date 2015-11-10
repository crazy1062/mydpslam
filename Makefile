

CC = g++
CFLAGS = -g -I. -O3 -Wall -std=c++11 -I/usr/local/lib/g++-include

SRC = slam.o particle.o ancestry.o gridmap.o rangeLogger.o robot.h singleton.h

slamTest: $(SRC)
	$(CC) $(CFLAGS) slamTest.cpp -o slamTest $(SRC)

slam.o: slam.cpp slam.h ancestry.h gridmap.h particle.h utils.h frwds.h
	$(CC) $(CFLAGS) -c slam.cpp

particle.o: particle.cpp particle.h ancestry.h gridmap.h utils.h frwds.h
	$(CC) $(CFLAGS) -c particle.cpp

ancestry.o: ancestry.cpp ancestry.h particle.h gridmap.h utils.h frwds.h
	$(CC) $(CFLAGS) -c ancestry.cpp

gridmap.o: gridmap.cpp gridmap.h particle.h ancestry.h utils.h frwds.h
	$(CC) $(CFLAGS) -c gridmap.cpp

rangeLogger.o: rangeLogger.cpp logger.h utils.h
	$(CC) $(CFLAGS) -c rangeLogger.cpp

#LogRobot.o: robot.h logger.h singleton.h utils.h
#	$(CC) $(CFLAGS) robot.h
