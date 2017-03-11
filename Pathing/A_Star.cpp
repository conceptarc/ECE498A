#pragma once
#include <cfloat>
#include "A_Star.h"

bool A_Star::FindPath(TreadmillMap* map2d) {
	unordered_set<Node*> openList; // note that costList makes this redundant
	unordered_set<Node*> closedList;
	unordered_map<Node*, Node*> parentList;
	unordered_map<Node*, float> costList;

	Node* current = map2d->GetStart();
	Node* currentBest = current;
	float bestHeuristic = FLT_MAX;
	openList.insert(current);
	parentList[current] = nullptr;
	costList[current] = map2d->CalcHeuristic(current); // ->GetHeuristicDist();
	int debugCounter = 0;
	int limit = 2 * map2d->GetMapWidthNodes() * map2d->GetMapLengthNodes();

	while (debugCounter++ < limit) {
		// find lowest F cost and assign that node as current
		if (map2d->CalcHeuristic(current) < bestHeuristic) { // July 14 update: must calculate heuristic before 
			bestHeuristic = current->GetHeuristicDist();
			currentBest = current;
		}

		current = NextNode(costList);

		openList.erase(current);
		closedList.insert(current);
		if (current == nullptr || current == map2d->GetGoal()) {
			if (current == nullptr)
				current = currentBest;

			// print the final path and distance
			deque<Node*> finalPath = GetPath(parentList, current);
			float distance = 0;
			finalPath[0]->SetPath(true);
			map2d->PathNodeList.push_back(finalPath[0]);
			for (int i = 1; i < finalPath.size(); i++) {
				finalPath[i]->SetPath(true);
				map2d->PathNodeList.push_back(finalPath[i]);
				float tempDist = map2d->CalcNodeWidthCm(); // dist between two nodes
				if (finalPath[i]->GetDiagonals().count(finalPath[i - 1]) != 0)
					tempDist = sqrtf(2 * tempDist*tempDist);
				distance += tempDist;
			}
			if (current == map2d->GetGoal())
				map2d->PathNodeList.push_back(current); // pad this list with another Goal so the dynamic mapping will move the S to G
														/*for each (Node* node in closedList) {
														node->SetVisited(true);
														}*/
			/*output.algorithmName = "A Star";
			output.hasSolution = current == map2d->GetGoal();
			output.nodesTotal = map2d->GetMapWidthNodes() * map2d->GetMapLengthNodes();
			output.nodesVisited = (int)openList.size() + (int)closedList.size();
			output.percentVisited = output.nodesVisited / ((double)output.nodesTotal);
			output.solutionDistance = distance;
			output.solutionTime = duration;
			output.widthResolution = map2d->GetMapWidthNodes();*/
			return current == map2d->GetGoal();

			//break;
		}

		// note that paths contain the final node too
		vector<Node*> adjacent = current->GetAllAdjacent();
		for (int i = 0; i < adjacent.size(); i++) {
			Node* node = adjacent[i];
			if (node == nullptr || closedList.count(node) != 0) continue;
			if (node->IsOccupied() || node->IsOccupationPredicted) {
				closedList.insert(node);
				continue;
			}
			// if this is not in OPEN or if its a shorter path than the one in OPEN
			// then add/replace in OPEN as needed
			float temp = map2d->CalcHeuristic(current); // July 14 change: Heuristics must be calculated the first time they are used
			float deltaG = map2d->CalcNodeWidthCm(); // dist between two nodes
			if (current->GetDiagonals().count(node) != 0)
				deltaG = sqrtf(2 * deltaG*deltaG); // get diagonal distance
			float newPath = costList[current] - current->GetHeuristicDist() + deltaG + map2d->CalcHeuristic(node);
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

	return false;
	//return output;
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
	Node* current = end;
	while (end != nullptr) {
		path.push_front(end);
		end = parentList[end];
	}
	end = current;
	return path;
}
