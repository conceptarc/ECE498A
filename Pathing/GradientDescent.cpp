#pragma once
#include <cfloat>
#include "GradientDescent.h"

void GradientDescent::FindPath(TreadmillMap* map) {
	// this is actually just hill climbing algorithm
	deque<Node*> pathList;
	unordered_set<int> visitedList;

	Node* current = map->GetStart();
	while (visitedList.count(current->GetID()) == 0 && current != map->GetGoal()) {
		visitedList.insert(current->GetID());
		pathList.push_back(current);
		current = NextNode(current->GetAllAdjacent());
		if (current == map->GetGoal()) {
			visitedList.insert(current->GetID());
			pathList.push_back(current); // breaks immediately after
		}
	}

	// store path into the map
	pathList[0]->SetPath(true);
	for (int i = 1; i < pathList.size(); i++) {
		pathList[i]->SetPath(true);
		cout << pathList[i]->GetID() << endl;
		map->PathNodeList.push_back(pathList[i]);
	}
}

Node* GradientDescent::NextNode(vector<Node*> neighbours) {
	float cost = FLT_MAX;
	Node* selection = nullptr;
	for (int i = 0; i < neighbours.size(); i++) {
		Node* neighbour = neighbours[i];
		if (neighbour->GetHeuristicDist() < cost) {
			cost = neighbour->GetHeuristicDist();
			selection = neighbour;
		}
	}
	return selection;
}
