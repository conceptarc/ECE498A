#ifndef GRADIENTDESCENT_H
#define GRADIENTDESCENT_H

#include <vector>
#include <deque>
#include <ctime>
#include <unordered_set>
#include "Node.h"
//#include "TestResult.h"
#include "TreadmillMap.h"

#define nullptr 0

using namespace std;

class GradientDescent {
private:
	static Node* NextNode(vector<Node*> neighbours);
public:
	static void FindPath(TreadmillMap* map);
};

#endif