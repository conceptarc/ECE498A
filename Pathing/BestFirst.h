#ifndef BESTFIRST_H
#define BESTFIRST_H

#include <stdio.h>
#include "Map.h"
#include <tuple>
#include "TestResult.h"
#include <ctime>

#define nullptr 0

using namespace std;

class BestFirst {
public:
	static TestResult FindPath(Map* map);
};

TestResult BestFirst::FindPath(Map* wtf) {
	TestResult output; // adapter to the outside world
	clock_t startTime = clock();

	Node* currentNode = wtf->Map::GetStart();
	Node* goalNode = wtf->GetGoal();

	std::vector < std::tuple< Node*, Node*, float > > openSet; //vector of (Node, previousNode, heuristicDistance)
	std::vector < std::tuple< Node*, Node*, float > > closedSet;

	closedSet.push_back( std::tuple <Node*, Node*, float> (currentNode, nullptr, 0));

	while (currentNode != goalNode)
	{
		double currentTime = (clock() - startTime) / (double)CLOCKS_PER_SEC;
		if (currentTime > 5) break;
		vector <Node*> adjacentNodes = currentNode->GetAllAdjacent();
		
		//check for node in closedSet
		for (int i = 0; i < adjacentNodes.size(); i++){
			for (int j = 0; j < (signed int)closedSet.size(); j++){
				if (adjacentNodes[i] != nullptr && !adjacentNodes[i]->IsOccupied() && !adjacentNodes[i]->IsVisited())
				{
					if (adjacentNodes[i] == std::get<0> (closedSet[j])) {
						//cout << "closed "<< currentNode->GetID << "," << i << ","<<j << endl;
						break;
					}
					if (j == closedSet.size() - 1)
					{
						adjacentNodes[i]->SetVisited(true);
						openSet.push_back (std::tuple <Node*, Node*, float> (adjacentNodes[i], currentNode, adjacentNodes[i]->GetHeuristicDist()));
					}
				}
			}
		}

		//set new currentNode
		if (openSet.size() == 0) break;
		currentNode = std::get<0> (openSet.front());
		int position = 0;
		for (int i = 1; i < (signed int)openSet.size(); i++){
			if ( currentNode->GetHeuristicDist() >  (std::get<0> (openSet[i])->GetHeuristicDist() ) )
			{
				currentNode = std::get<0>(openSet[i]);
				position = i;
			}
		}

		//put currentNode into closedSet
		closedSet.push_back(openSet[position]);

		//remove node from openSet
		openSet.erase(openSet.begin() + position);

	}
	//cout << "done" << endl;

	output.algorithmName = "Best First";
	output.hasSolution = currentNode == goalNode;
	output.nodesTotal = wtf->GetResolution() * wtf->GetResolution();
	output.nodesVisited = openSet.size() + closedSet.size();
	output.percentVisited = output.nodesVisited / (double)output.nodesVisited;
	output.solutionTime = (clock() - startTime) / (double)CLOCKS_PER_SEC;
	output.widthResolution = wtf->GetResolution();

	//marking map with a path
	Node* startNode = wtf->Map::GetStart();
	Node* pathNode = wtf->Map::GetGoal();
	Node* previous = pathNode;
	float distance = 0;
	for (int i = closedSet.size() - 1; pathNode != startNode && i >= 0; i --){
		if (std::get<0>(closedSet[i]) == pathNode){
			float tempDist = 100.0 / wtf->GetResolution();
			pathNode = std::get<1>(closedSet[i]);
			pathNode->SetPath(true);

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