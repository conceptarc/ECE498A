#ifndef A_STAR_H
#define A_STAR_H

#include <vector>
#include "Node.h"
#include "Map.h"
#define nullptr 0

using namespace std;

class A_Star {
private:
	static bool Contains(vector<Node*> list, Node* target);
	static vector<Node*> LowestCost(vector<vector<Node*>> &paths);
	static vector<Node*> GetPath(vector<vector<Node*>> &paths, Node* &end, bool remove);
	static vector<Node*> AppendToCopy(vector<Node*> existing, Node* target);
public:
	static void FindPath(Map* map);
};

void A_Star::FindPath(Map* map) {
	vector<Node*> openList;
	vector<Node*> closedList;
	vector<vector<Node*>> pathList; // list of parents for each node
	float minDist = FLT_MAX;

	Node* current = map->GetStart();
	openList.push_back(current);
	vector<Node*> initPath;
	initPath.push_back(current);
	pathList.push_back(initPath);
	int debugCounter = 0;
	int limit = 2 * map->GetResolution()* map->GetResolution();

	while (debugCounter++ < limit) { // carefully
		vector<Node*> lowest = LowestCost(pathList);
		current = lowest.back();
		int selectedIndex = -1;
		for (int i = 0; i < openList.size(); i++) {
			if (current == openList[i]) {
				selectedIndex = i;
				break;
			}
		}
		if (selectedIndex < 0) {
			cout << "WTF - it broke\r\n" << endl;
			break;
		}
		// find lowest F cost and assign to current
		openList.erase(openList.begin() + selectedIndex);
		closedList.push_back(current);
		if (current == map->GetGoal()) {
			// print the final path and distance
			cout << "done" << endl;
			vector<Node*> finalPath = GetPath(pathList, current, false);
			for each (Node* node in finalPath) {
				node->SetPath(true);
			}
			map->Print();
			break;
		}

		// note that paths contain the final node too
		vector<Node*> currentPath = GetPath(pathList, current, true); // remove this one from the list too
		vector<Node*> adjacent = current->GetAllAdjacent();
		for (int i = 0; i < adjacent.size(); i++) {
			if (Contains(closedList, adjacent[i])) continue;
			if (adjacent[i]->IsOccupied()) {
				closedList.push_back(adjacent[i]);
				continue;
			}
			// if this is not in OPEN or if its a shorter path than the one in OPEN
			// then add/replace in OPEN as needed

			if (!Contains(openList, adjacent[i])) {
				pathList.push_back(AppendToCopy(currentPath, adjacent[i]));
				openList.push_back(adjacent[i]);
			} else {
				vector<Node*> existingPath = GetPath(pathList, adjacent[i], false);
				vector<Node*> candidatePath = AppendToCopy(currentPath, adjacent[i]);
				// note that paths contain the final node too
				if (candidatePath.size() < existingPath.size()) {
					existingPath = candidatePath;
					// don't modify OPEN list
				}
			}
		}
	}
}

bool A_Star::Contains(vector<Node*> list, Node* target) {
	for (int i = 0; i < list.size(); i++) {
		if (list[i] == target) return true;
	}
	return false;
}

vector<Node*> A_Star::LowestCost(vector<vector<Node*>> &paths) {
	float dist = FLT_MAX;
	vector<Node*> cheapest;
	for each (vector<Node*> path in paths)
	{
		if (path.size() + path.back()->GetHeuristicDist() < dist) {
			dist = path.size() + path.back()->GetHeuristicDist();
			cheapest = path;
		}
	}
	return cheapest;
}

vector<Node*> A_Star::GetPath(vector<vector<Node*>> &paths, Node* &end, bool remove) {
	vector<Node*> path;
	for (int i = 0; i < paths.size(); i++)
	{
		if (!paths[i].empty() && paths[i].back() == end) {
			path = paths[i];
			if (remove) paths.erase(paths.begin() + i);
			break;
		}
	}
	return path;
}

vector<Node*> A_Star::AppendToCopy(vector<Node*> existing, Node* target) {
	vector<Node*> copy;
	for each (Node* node in existing) {
		copy.push_back(node);
	}
	copy.push_back(target);
	return copy;
}

#endif