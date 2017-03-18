#pragma once
#include <cfloat>
#include "GradientDescent.h"

void GradientDescent::FindPath(TreadmillMap* map) {
	// this is actually just hill climbing algorithm
	deque<Node*> pathList;
	unordered_set<int> visitedList;

	Node* current = map->GetStart();
	int maxLength = 4; // should be less than A star limit of 3
	int count = 0;
	bool alreadyVisited = visitedList.count(current->GetID()) != 0;
	float lowestCost = current->GetHeuristicDist();

	while (pathList.size() < maxLength && count++ < maxLength) {
		visitedList.insert(current->GetID());
		current = NextNode(current->GetAllAdjacent());
		if (current->GetHeuristicDist() < lowestCost) {
			lowestCost = current->GetHeuristicDist();
			pathList.push_back(current);
			alreadyVisited = visitedList.count(current->GetID()) != 0;
		}
		else {
			break;
		}
	}

	// store path into the map
	cout << "GD path size: " << pathList.size() << endl;
	for (int i = 0; i < pathList.size(); i++) {
		pathList[i]->SetPath(true);
		// cout << pathList[i]->GetID() << endl;
		map->PathNodeList.push_back(pathList[i]);
	}
}

Node* GradientDescent::NextNode(deque<Node*> neighbours) {
	float cost = FLT_MAX;
	Node* selection = nullptr;
	for (int i = 0; i < neighbours.size(); i++) {
		Node* neighbour = neighbours[i];

		if (neighbour->GetHeuristicDist() < cost) {
			cost = neighbour->GetHeuristicDist();
			//cout << "nearby cost: " << cost << endl;
			selection = neighbour;
		}
	}
	return selection;
}
