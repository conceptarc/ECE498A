#pragma once
#include "GradientDescent.h"

void GradientDescent::FindPath(TreadmillMap* map) {
	//TestResult output;
	clock_t timer = clock(); // optional

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
	double duration = (clock() - timer) / (double)CLOCKS_PER_SEC;
	/*output.algorithmName = "Gradient Descent";
	output.solutionTime = duration;
	output.hasSolution = current == map->GetGoal();*/

	float distance = 0;
	pathList[0]->SetPath(true);
	for (int i = 1; i < pathList.size(); i++) {
		pathList[i]->SetPath(true);
		float tempDist = 100.0f / map->GetResolution();
		if (pathList[i]->GetDiagonals().count(pathList[i - 1]) != 0)
			tempDist = sqrtf(2 * tempDist*tempDist);
		distance += tempDist;
	}

	//output.solutionDistance = distance;
	//output.nodesVisited = (int)pathList.size();
	//output.widthResolution = map->GetResolution();
	//output.nodesTotal = map->GetResolution() * map->GetResolution();
	//output.percentVisited = output.nodesVisited / ((double)output.nodesTotal);

	//return output;
}

Node* GradientDescent::NextNode(vector<Node*> neighbours) {
	float cost = FLT_MAX;
	Node* selection = nullptr;
	for each (Node* neighbour in neighbours) {
		if (neighbour->GetHeuristicDist() < cost) {
			cost = neighbour->GetHeuristicDist();
			selection = neighbour;
		}
	}
	return selection;
}
