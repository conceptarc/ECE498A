#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "Map.h"
#include <tuple>
#include "TestResult.h"
#include <ctime>

#define nullptr 0

using namespace std;

class Dijkstra {
public:
	static TestResult FindPath(Map* map);
};

TestResult Dijkstra::FindPath(Map* wtf) {
	TestResult output; // adapter to the outside world
	clock_t startTime = clock();

	tuple <Node*, Node*, float>  current;
	current = tuple <Node*,Node*, float> (wtf->GetStart(), nullptr, 0);
	Node* goalNode = wtf->GetGoal();

	vector < tuple< Node*, Node*, float > > openSet; //vector of (Node, previousNode, distance to start)
	vector < tuple< Node*, Node*, float > > closedSet;

	closedSet.push_back( tuple <Node*, Node*, float> ((get<0>(current)), nullptr, 0));
	while (get<0>(current) != goalNode)
	{
		double currentTime = (clock() - startTime) / (double)CLOCKS_PER_SEC;
		if (currentTime > 5) break;
		vector <Node*> adjacentNodes = get<0>(current)->GetAllAdjacent();
		
		//check for node in closedSet
		for (int i = 0; i < 4; i++){
			for (int j = 0; j < closedSet.size(); j++){
				if (adjacentNodes[i] != nullptr && !adjacentNodes[i]->IsOccupied() && !adjacentNodes[i]->IsVisited())
				{
					if (adjacentNodes[i] == get<0> (closedSet[j])) {
						break;
					}
					if (j == closedSet.size() - 1)
					{
						adjacentNodes[i]->SetVisited(true);
						openSet.push_back (tuple <Node*, Node*, float> (adjacentNodes[i], get<0>(current), get<2> (current) + 100.0/wtf->GetResolution()));
					}
				}
			}
		}

		for (int i = 4; i < 8; i++){
			for (int j = 0; j < closedSet.size(); j++){
				if (adjacentNodes[i] != nullptr && !adjacentNodes[i]->IsOccupied() && !adjacentNodes[i]->IsVisited())
				{
					if (adjacentNodes[i] == get<0> (closedSet[j])) {
						//cout << "closed "<< currentNode->GetID << "," << i << ","<<j << endl;
						break;
					}
					if (j == closedSet.size() - 1)
					{
						adjacentNodes[i]->SetVisited(true);
						openSet.push_back (tuple <Node*, Node*, float> (adjacentNodes[i], get<0>(current), get<2>(current) + sqrt(2 * 100.0 / wtf->GetResolution() * 100.0 / wtf->GetResolution())));
					}
				}
				//cout << "tst" << endl;
			}
		}

		//set new currentNode
		if (openSet.size() == 0)
			break;
		current = openSet.front();
		int position = 0;
		for (int i = 1; i < openSet.size(); i++){
			if ( get<2>(current) >  get<2> (openSet[i]) )
			{
				current = openSet[i];
				position = i;
			}
		}

		//put currentNode into closedSet
		closedSet.push_back(openSet[position]);

		//remove node from openSet
		openSet.erase(openSet.begin() + position);
	}

	output.algorithmName = "Dijkstra's";
	output.hasSolution = get<0>(current) == goalNode;
	output.nodesTotal = wtf->GetResolution() * wtf->GetResolution();
	output.nodesVisited = openSet.size() + closedSet.size();
	output.percentVisited = output.nodesVisited / (double)output.nodesVisited;
	output.solutionTime = (clock() - startTime) / (double)CLOCKS_PER_SEC;

	output.solutionDistance = get<2>(closedSet[closedSet.size() - 1]);
	output.widthResolution = wtf->GetResolution();
	
	//marking map with a path
	Node* startNode = wtf->GetStart();
	Node* pathNode = wtf->GetGoal();

	for (int i = closedSet.size() - 1; pathNode != startNode && i > 0; i --){
		if (std::get<0>(closedSet[i]) == pathNode){

			float tempDist = 100.0 / wtf->GetResolution();
			if (get<1>(closedSet[i])->GetDiagonals().count(get<1>(closedSet[i - 1])) != 0)
				tempDist = sqrtf(2 * tempDist*tempDist);

			pathNode = std::get<1>(closedSet[i]);
			pathNode->SetPath(true);
		}
	}

	return output;
}

#endif