#ifndef A_STAR_H
#define A_STAR_H

#include <vector>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <ctime>
#include "Node.h"
#include "Map.h"
#define nullptr 0

using namespace std;

class A_Star {
private:
	static Node* NextNode(unordered_map<Node*, float> &costList);
	static deque<Node*> GetPath(unordered_map<Node*, Node*> &pathList, Node* &end);
public:
	static void FindPath(Map* map);
};

void A_Star::FindPath(Map* map2d) {
	clock_t timer = clock(); // optional

	unordered_set<Node*> openList; // note that costList makes this redundant
	unordered_set<Node*> closedList;
	unordered_map<Node*, Node*> parentList;
	unordered_map<Node*, float> costList;

	Node* current = map2d->GetStart();
	openList.insert(current);
	parentList[current] = nullptr;
	costList[current] = current->GetHeuristicDist();
	int debugCounter = 0;
	int limit = 2 * map2d->GetResolution()* map2d->GetResolution();
	float deltaG = 100.0 / map2d->GetResolution();

	while (debugCounter++ < limit) {
		
		// find lowest F cost and assign that node as current
		current = NextNode(costList);

		openList.erase(current);
		closedList.insert(current);
		if (current == map2d->GetGoal()) {
			double duration = (clock() - timer) / (double)CLOCKS_PER_SEC;


			// print the final path and distance
			deque<Node*> finalPath = GetPath(parentList, current);
			for each (Node* node in finalPath) {
				node->SetPath(true);
			}
			for each (Node* node in closedList) {
				node->SetVisited(true);
			}
			//map2d->Print();
			cout << endl << map2d->GetResolution() << "*" << map2d->GetResolution() <<
				" done:\r\ndistance = " << finalPath.size() - 1 <<
				"\r\ntime = " << duration << "\r\nnodes visited = " << closedList.size() <<
				"/" << map2d->GetResolution() * map2d->GetResolution() << endl;
			break;
		}

		// note that paths contain the final node too
		vector<Node*> adjacent = current->GetAllAdjacent();
		for (int i = 0; i < adjacent.size(); i++) {
			Node* node = adjacent[i];
			if (node == nullptr || closedList.count(node) != 0) continue;
			if (node->IsOccupied()) {
				closedList.insert(node);
				continue;
			}
			// if this is not in OPEN or if its a shorter path than the one in OPEN
			// then add/replace in OPEN as needed
			float temp = current->GetHeuristicDist();
			float newPath = costList[current] - current->GetHeuristicDist() + deltaG + node->GetHeuristicDist();
			if (openList.count(node) == 0) {
				openList.insert(node);
				costList[node] = newPath;
				parentList[node] = current;
			}
			else {
				// check if this path is shorter
				float oldPath = costList[node];
				if (newPath < oldPath) {
					parentList[node] = current;
					costList[node] = newPath;
				}
			}
		}
		costList.erase(current); // because it is not in the open list anymore
	}
}

Node* A_Star::NextNode(unordered_map<Node*, float> &costList) {
	float cost = FLT_MAX;
	Node* selection = nullptr;
	for (unordered_map<Node*, float>::iterator i = costList.begin(); i != costList.end(); i++) {
		if (i->second < cost) {
			cost = i->second;
			selection = i->first;
		}
	}
	return selection;
}

deque<Node*> A_Star::GetPath(unordered_map<Node*, Node*> &parentList, Node* &end) {
	deque<Node*> path;
	while (end != nullptr) {
		path.push_front(end);
		end = parentList[end];
	}

	return path;
}

#endif
