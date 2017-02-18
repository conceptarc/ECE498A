#ifndef A_STAR_H
#define A_STAR_H

#include <vector>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <ctime>
#include "Node.h"
#include "TreadmillMap.h"
//#include "TestResult.h"
#define nullptr 0

using namespace std;

class A_Star {
private:
	static Node* NextNode(unordered_map<Node*, float> &costList);
	static deque<Node*> GetPath(unordered_map<Node*, Node*> &pathList, Node* &end);
public:
	static bool FindPath(TreadmillMap* map);
};

#endif