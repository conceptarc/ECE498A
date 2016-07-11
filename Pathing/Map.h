#ifndef MAP_H
#define MAP_H

#include <math.h>
#include <deque>
#include <iostream>
#include "Node.h"
#include "MobileObstacle.h"

#define nullptr 0

using namespace std;

enum MapOption {
	Occupation = 0,
	Gradient = 1
};

enum MapObstacleSet {
	NoSolution = -1,
	Clear = 0,
	Sparse = 1,
	Dense = 2,
	Wall = 3
};

class Map {

private:
	float MAP_SIZE;
	float START_X;
	float START_Y;
	float GOAL_X;
	float GOAL_Y;
	float PADDING;

	int _resolution;
	Node*** map2d;
	Node* _start;
	Node* _goal;
	Node* _topLeft;
	deque<MobileObstacle*> _obstacleList;
	
	float CalcDist(float x1, float x2, float y1, float y2, bool useManhattan);
	float CalcGradientObsHeight(float distance, float radius, float padding);

public:
	Map(int resolution, MapOption option, MapObstacleSet obsOption);
	~Map();
	float GetMapSize();
	void AddObstacle(MobileObstacle* obj);
	int GetResolution();
	Node* GetStart();
	Node* GetGoal();
	void Print();
	void UpdateMobileObstacles(float deltaTime);
	deque<Node*> PathNodesToClear;
	void ClearPath();
};

Map::Map(int resolution, MapOption option, MapObstacleSet obsOption) : _resolution(resolution) {
	// some arbitrary constants / constraints
	if (resolution < 10) {
		_resolution = 10;
	} else if (resolution > 2000) {
		_resolution = 2000;
	}
	MAP_SIZE = 100.0f;
	START_X = 15.0f;
	START_Y = 15.0f;
	GOAL_X = 85.0f;
	GOAL_Y = 85.0f;
	PADDING = 1; // n units away from radius of obstacle

	// generate a fixed map with variable resolution

	// create 2D array of pointers
	map2d = new Node**[_resolution];
	for (int i = 0; i < _resolution; i++) {
		map2d[i] = new Node*[_resolution];
	}

	// initialize
	for (int i = 0; i < _resolution; i++) {
		for (int j = 0; j < _resolution; j++) {
			map2d[i][j] = new Node();
		}
	}

	// now assign properties and references for each one
	float nodeWidth = MAP_SIZE / _resolution;
	float startProximity = nodeWidth * 2; // used for finding the start node
	float goalProximity = nodeWidth * 2; // used for finding the goal node

	// before iterating through each node, assign the obstacles (circular)
	deque<Obstacle> obsList;
	float scaling = 4;
	float wallThickness = 1.5;
	
	if (obsOption == MapObstacleSet::Wall || obsOption == MapObstacleSet::NoSolution) {
		obsList.push_back(Obstacle(18, 35, 85, wallThickness*scaling)); // id, x, y, r
		obsList.push_back(Obstacle(19, 40, 80, wallThickness*scaling));
		obsList.push_back(Obstacle(20, 45, 75, wallThickness*scaling));
		obsList.push_back(Obstacle(21, 50, 70, wallThickness*scaling));
		obsList.push_back(Obstacle(22, 55, 65, wallThickness*scaling));
		obsList.push_back(Obstacle(23, 60, 60, wallThickness*scaling));
		obsList.push_back(Obstacle(24, 65, 55, wallThickness*scaling));
		obsList.push_back(Obstacle(25, 70, 50, wallThickness*scaling));
		obsList.push_back(Obstacle(26, 75, 45, wallThickness*scaling));
		obsList.push_back(Obstacle(27, 80, 40, wallThickness*scaling));
		obsList.push_back(Obstacle(28, 85, 35, wallThickness*scaling));

		obsList.push_back(Obstacle(29, 40, 85, wallThickness*scaling));
		obsList.push_back(Obstacle(30, 45, 85, wallThickness*scaling));
		obsList.push_back(Obstacle(31, 50, 80, wallThickness*scaling));
		obsList.push_back(Obstacle(32, 55, 75, wallThickness*scaling));
		obsList.push_back(Obstacle(33, 60, 70, wallThickness*scaling));
		obsList.push_back(Obstacle(34, 65, 65, wallThickness*scaling));
		obsList.push_back(Obstacle(35, 70, 60, wallThickness*scaling));
		obsList.push_back(Obstacle(36, 75, 55, wallThickness*scaling));
		obsList.push_back(Obstacle(37, 80, 50, wallThickness*scaling));
		obsList.push_back(Obstacle(38, 85, 45, wallThickness*scaling));
		obsList.push_back(Obstacle(39, 90, 45, wallThickness*scaling));

		if (obsOption == NoSolution) {
			obsList.push_back(Obstacle(40, 40, 95, 2*wallThickness*scaling));
			obsList.push_back(Obstacle(41, 95, 40, 2*wallThickness*scaling));
		}
	} else if (obsOption != MapObstacleSet::Clear) {
		obsList.push_back(Obstacle(0, 35, 35, scaling)); // id, x, y, r
		obsList.push_back(Obstacle(1, 55, 35, scaling));
		obsList.push_back(Obstacle(2, 75, 35, scaling));
		obsList.push_back(Obstacle(3, 35, 55, scaling));
		obsList.push_back(Obstacle(4, 55, 55, scaling));
		obsList.push_back(Obstacle(5, 75, 55, scaling));
		obsList.push_back(Obstacle(6, 35, 75, scaling));
		obsList.push_back(Obstacle(7, 55, 75, scaling));
		obsList.push_back(Obstacle(8, 75, 75, scaling));

		if (obsOption != MapObstacleSet::Sparse) {
			obsList.push_back(Obstacle(9, 25, 25, scaling)); // id, x, y, r
			obsList.push_back(Obstacle(10, 45, 25, scaling));
			obsList.push_back(Obstacle(11, 65, 25, scaling));
			obsList.push_back(Obstacle(12, 25, 45, scaling));
			obsList.push_back(Obstacle(13, 45, 45, scaling));
			obsList.push_back(Obstacle(14, 65, 45, scaling));
			obsList.push_back(Obstacle(15, 25, 65, scaling));
			obsList.push_back(Obstacle(16, 45, 65, scaling));
			obsList.push_back(Obstacle(17, 65, 65, scaling));
		}
	}

	// now populate each node's properties with the information above
	for (int i = 0; i < _resolution; i++) {
		for (int j = 0; j < _resolution; j++) {
			Node* node = map2d[i][j];
			node->SetID(i*_resolution + j); // ascending integer for ID

			// set 4-primary links
			node->SetNorth(i == 0 ? NULL : map2d[i - 1][j]);
			node->SetSouth(i == _resolution - 1 ? NULL : map2d[i + 1][j]);
			node->SetWest(j == 0 ? NULL : map2d[i][j - 1]);
			node->SetEast(j == _resolution - 1 ? NULL : map2d[i][j + 1]);

			// set 4-diagonal links
			node->SetNorthEast(i == 0 || j == _resolution - 1? NULL : map2d[i - 1][j + 1]);
			node->SetNorthWest(i == 0 || j == 0 ? NULL : map2d[i - 1][j - 1]);
			node->SetSouthEast(i == _resolution - 1 || j == _resolution - 1 ? NULL : map2d[i + 1][j + 1]);
			node->SetSouthWest(i == _resolution - 1 || j == 0 ? NULL : map2d[i + 1][j - 1]);
			
			float x = nodeWidth * j + nodeWidth / 2;
			float y = nodeWidth * (_resolution - i - 1) + nodeWidth / 2;
			node->X = x;
			node->Y = y; // added for dynamic maps/obstacles

			float distToStart = CalcDist(x, START_X, y, START_Y, false);
			float distToGoal = CalcDist(x, GOAL_X, y, GOAL_Y, false); // true means use Manhattan
			node->SetHeuristic(distToGoal); // calculated above, might as well use it

			// find the start node
			if (distToStart < startProximity) {
				startProximity = distToStart;
				if (_start != nullptr) _start->SetStart(false);
				_start = node;
				_start->SetStart(true);
			}

			// find the goal node
			if (distToGoal < goalProximity) {
				goalProximity = distToGoal;
				if (_goal != nullptr) _goal->SetGoal(false);
				_goal = node;
				_goal->SetGoal(true);
			}
			
			// address the existing obstacles
			for each (Obstacle obs in obsList) {
				if (option == MapOption::Gradient) {
					float distance = CalcDist(x, obs.X, y, obs.Y, false);
					if (distance <= obs.Radius + PADDING) {
						node->SetHeuristic(powf(node->GetHeuristicDist(), 2) + CalcGradientObsHeight(distance, obs.Radius, PADDING));
						if (distance <= obs.Radius) {
							node->SetOccupied(true);
							node->SetHeuristic(FLT_MAX); // arbitrarly / sufficiently high
							break;
						}
					}
				}
				else {
					if (powf(x - obs.X, 2) + powf(y - obs.Y, 2) < powf(obs.Radius, 2)) {
						node->SetOccupied(true);
						break;
					}
				}
			}
		}
	}
	_topLeft = map2d[0][0];
}

Map::~Map() {
	for (int i = 0; i < _resolution; i++) {
		for (int j = 0; j < _resolution; j++) {
			delete map2d[i][j];
		}
		delete[] map2d[i];
	}
	delete[] map2d;
}

float Map::GetMapSize() {
	return MAP_SIZE; // read only value
}

void Map::AddObstacle(MobileObstacle* obj) {
	_obstacleList.push_back(obj);
}

int Map::GetResolution() {
	return _resolution;
}

Node* Map::GetStart() {
	return _start;
}

Node* Map::GetGoal() {
	return _goal;
}

void Map::Print() {

	//debug print nodes
	/*
	for (int i = -1; i <= _resolution; i++) {
		for (int j = -1; j <= _resolution; j++) {
			if (i == -1 || i == _resolution)
				cout << '-';
			else if (j == -1 || j == _resolution)
				cout << '|'; 
			else
				cout << map2d[i][j]->Print();
		}
		cout << endl;
	}

	cout << endl;
	*/
	Node* column = _topLeft;
	Node* row = _topLeft;
	
	for (int i = 0; i < _resolution + 2; i++) {
		cout << '-';
	} // top boundary

	cout << endl;
	do {
		cout << '|' << column->Print();
		row = column->GetEast();
		while (row != nullptr) {
			cout << row->Print();
			row = row->GetEast();
		}
		cout << '|' << endl;
		column = column->GetSouth();
	} while (column != nullptr);

	for (int i = 0; i < _resolution + 2; i++) { // bottom boundary
		cout << '-';
	}
}

void Map::UpdateMobileObstacles(float deltaTime) {
	// does not work for gradient descent!

	deque<MobileObstacle*> outOfBoundsList;

	for (int i = 0; i < _obstacleList.size(); i++) {
		MobileObstacle* obj = _obstacleList[i];

		obj->Move(deltaTime);

		// perform BFS on the nearby area starting at the centre node
		unordered_set<int> newNodeArea; // closed list of the search
		deque<Node*> openList;
		deque<Node*> closedList;

		int iApprox = (int)(roundf(_resolution - 0.5 - _resolution / MAP_SIZE*obj->Y)); // inverse function of y
		int jApprox = (int)(roundf(_resolution / MAP_SIZE*obj->X - 0.5)); // inverse function of x

		// restrict indices - just because the centre point is off the map does not mean the object is off the map (due to radius)
		// this point is used as a starting point for the search
		iApprox = (int)fmax(iApprox, 0);
		iApprox = (int)fmin(iApprox, _resolution - 1);
		jApprox = (int)fmax(jApprox, 0);
		jApprox = (int)fmin(jApprox, _resolution - 1);
		
		Node* rootNode = map2d[iApprox][jApprox];
		openList.push_back(rootNode);

		while (openList.size() > 0) {
			Node* current = openList[0];
			openList.pop_front();
			vector<Node*> neighbours = current->GetAllAdjacent();
			for each (Node* node in neighbours) {
				if (node == nullptr) continue;

				if (CalcDist(node->X, obj->X, node->Y, obj->Y, false) < obj->Radius) {
					if (newNodeArea.count(node->GetID()) == 0) {
						closedList.push_back(node);
						newNodeArea.insert(node->GetID());
						openList.push_back(node);
					}
				}
			}
		} // BFS completes
		
		// apply changes to the map

		for each (Node* node in obj->MapArea) {
			node->SetOccupied(false);
		}
		obj->MapArea.clear();
		for each (Node* node in closedList) {
			node->SetOccupied(true);
			obj->MapArea.push_back(node);
		}
		if (closedList.size() == 0)
			outOfBoundsList.push_back(obj); // mark for deletion
	}

	// delete the obstacles that moved off the map
	for each (MobileObstacle* obj in outOfBoundsList) {
		for (int i = 0; i < _obstacleList.size(); i++) {
			if (_obstacleList[i]->Id == obj->Id) {
				delete _obstacleList[i];
				_obstacleList.erase(_obstacleList.begin() + i);

				break;
			}
		}
	}
}

void Map::ClearPath() {
	for each (Node* node in PathNodesToClear) {
		node->SetPath(false);
	}
}

float Map::CalcDist(float x1, float x2, float y1, float y2, bool useManhattan) {
	if (useManhattan)
		return fabs(x1 - x2) + fabs(y1 - y2);
	return sqrtf(powf(x1 - x2, 2) + powf(y1 - y2, 2));
}

float Map::CalcGradientObsHeight(float distance, float radius, float padding) {
	if (distance <= radius) return FLT_MAX;
	if (distance < radius + padding)
		return MAP_SIZE / (distance - radius) - MAP_SIZE / (padding);
	return 0.0f;
}

#endif