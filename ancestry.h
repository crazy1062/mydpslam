#ifndef _ANCESTRY_H_
#define _ANCESTRY_H_

#include "utils.h"
#include "frwds.h"
#include <memory>
#include <vector>
#include <iostream>
#include <stack>

class Ancestry{
friend class GridMap;
friend class Particle;
private:
	int _id_num;
	std::vector<TAncestor> particleID; //ID_NUMBER
	std::vector<int> availableID; //ID_NUMBER
	int cleanID;
	std::weak_ptr<GridMap> gweak;
	std::weak_ptr<Particle> pweak;
protected:
	bool gptr_is_there();
	bool pptr_is_there();
public:
	Ancestry();
	Ancestry(int);
	virtual ~Ancestry();
	void setWPtr(std::shared_ptr<GridMap> gw, std::shared_ptr<Particle> pw);
	virtual void InitAncestry();
	virtual void DisposeAncestry();
	virtual void UpdateAncestry(TSense sense, int curGeneration, bool low);
	virtual void ResizeArray(TMapStarter* node, int deadID);
	TAncestor* getRootNode();
	TAncestor* popAvailableNode();
	TAncestor* getNode(int index);
};


#endif
