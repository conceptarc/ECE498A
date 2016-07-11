#ifndef UNIFORM_H
#define UNIFORM_H

#include <stdio.h>
#include "Map.h"
#include <tuple>
#include "TestResult.h"
#include <ctime>

#define nullptr 0

using namespace std;

class Uniform {
public:
	static TestResult FindPath(Map* map);
};

TestResult Uniform::FindPath(Map* wtf) {
	TestResult output; // adapter to the outside world
	clock_t startTime = clock();

	Node* currentNode =  wtf->GetStart();
	Node* goalNode = wtf->GetGoal();

	std::vector < std::tuple< Node*, Node* > > openSet; //vector of (Node, previousNode)
	std::vector < std::tuple< Node*, Node* > > closedSet;

	closedSet.push_back( std::tuple <Node*, Node*> (currentNode, nullptr));

	while (currentNode != goalNode)
	{
		double currentTime = (clock() - startTime) / (double)CLOCKS_PER_SEC;
		if (currentTime > 5) break;

		bool inClosed = false;
		bool inOpen = false;
		vector <Node*> adjacentNodes = currentNode->GetAllAdjacent();
		
		for (int i = 0; i < adjacentNodes.size(); i++){
			if (adjacentNodes[i] != nullptr && !adjacentNodes[i]->IsOccupied() && !adjacentNodes[i]->IsVisited()){
				//check for node in closedSet
				for (int j = 0; j < (signed int)closedSet.size(); j++){
					if (adjacentNodes[i] == std::get<0> (closedSet[j])) {
						inClosed = true;
						break;
					}
				}
				if (!inClosed){
					openSet.push_back (std::tuple <Node*, Node*> (adjacentNodes[i], currentNode));
					adjacentNodes[i]->SetVisited(true);
				}
			}
		}
		//cout << openSet.size() << endl;
		currentNode = get<0>(openSet.front());
		if (openSet.size() > 0){
			closedSet.push_back(openSet.front());
			//cout << openSet.size() << endl;
			openSet.erase(openSet.begin());
			//cout << openSet.size() << endl;
		}
		
		if (openSet.size() == 0) {
			//cout << "wtf?!@#!?@#" << endl;
			break;
		}
	}

	output.algorithmName = "Uniform Cost";
	output.hasSolution = currentNode == goalNode;
	output.nodesTotal = wtf->GetResolution() * wtf->GetResolution();
	output.nodesVisited = openSet.size() + closedSet.size();
	output.percentVisited = output.nodesVisited / (double)output.nodesVisited;
	output.solutionTime = (clock() - startTime) / (double)CLOCKS_PER_SEC;
	output.widthResolution = wtf->GetResolution();

	//cout << "done..." << endl;
	//cout << closedSet.size() << endl;
		//marking map with a path
	Node* startNode = wtf->Map::GetStart();
	Node* pathNode = wtf->Map::GetGoal();
	Node* previous = pathNode;

	float distance = 0;
	for (int i = closedSet.size() - 1; pathNode != startNode && i >=0; i --){
		if (std::get<0>(closedSet[i]) == pathNode){
			pathNode = std::get<1>(closedSet[i]);
			pathNode->SetPath(true);

			float tempDist = 100.0 / wtf->GetResolution();
			if (pathNode->GetDiagonals().count(previous) != 0)
				tempDist = sqrtf(2 * tempDist*tempDist);
			distance += tempDist;
			previous = pathNode;
		}
	}

	output.solutionDistance = distance;
	return output;
}

#endif