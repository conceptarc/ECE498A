#pragma once
#include "TreadmillMap.h"

using namespace std;

TreadmillMap::TreadmillMap(int width, int height, float resolutionFactor, MapGridOption option) : _resolution(resolutionFactor) {
	// some arbitrary constants / constraints
	MAP_SIZE = 100.0f;
	START_X = 15.0f;
	START_Y = 15.0f;
	GOAL_X = 85.0f;
	GOAL_Y = 85.0f;
	PADDING = 1; // n units away from radius of obstacle

	// generate a fixed map with variable resolution

	// create 2D array of pointers
	map2d = new Node**[height/*resolutionFactor*/]; // rows
	for (int i = 0; i < _resolution; i++) {
		map2d[i] = new Node*[width/*resolutionFactor*/]; // columns
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
			node->SetNorthEast(i == 0 || j == _resolution - 1 ? NULL : map2d[i - 1][j + 1]);
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
			for (int i = 0; i < obsList.size(); i++) {
				Obstacle obs = obsList[i];
				if (option == MapGridOption::GridGradient) {
					float distance = CalcDist(x, obs.X, y, obs.Y, false);
					if (distance <= obs.Radius + PADDING) {
						node->SetHeuristic(powf(node->GetHeuristicDist(), 2) + CalcGradientObsHeight(distance, obs.Radius, PADDING));
						if (distance <= obs.Radius) {
							node->IsObjectPresent = true;//node->SetOccupied(true);
							node->SetHeuristic(FLT_MAX); // arbitrarly / sufficiently high
							break;
						}
					}
				}
				else {
					if (powf(x - obs.X, 2) + powf(y - obs.Y, 2) < powf(obs.Radius, 2)) {
						node->IsObjectPresent = true; //node->SetOccupied(true);
						break;
					}
				}
			}
		}
	}
	_topLeft = map2d[0][0];

	// init _thisCar
	_thisCar = new MobileObstacle(25, PI / 2.0, -1, _start->X, _start->Y, 0);
}

TreadmillMap::~TreadmillMap() {
	for (int i = 0; i < _resolution; i++) {
		for (int j = 0; j < _resolution; j++) {
			delete map2d[i][j];
		}
		delete[] map2d[i];
	}
	delete[] map2d;
	delete _thisCar;
	for (int i = 0; i < _obstacleList.size(); i++) {
		delete _obstacleList[i];
	}
	_obstacleList.clear();
}

float TreadmillMap::GetMapSize() {
	return MAP_SIZE; // read only value
}

void TreadmillMap::AddObstacle(MobileObstacle* obj) {
	_obstacleList.push_back(obj);
}

void TreadmillMap::ClearObstacles() {
	for (int i = 0; i < _obstacleList.size(); i++) {
		MobileObstacle* obj = _obstacleList[i];
		for (int j = 0; j < obj->ActualArea.size(); j++) {
			obj->ActualArea[j]->IsObjectPresent = false;
		}
		for (int j = 0; j < obj->ProjectionArea.size(); j++) {
			obj->ProjectionArea[j]->IsOccupationPredicted = false;
		}
		delete obj;
	}
	_obstacleList.clear();
}

int TreadmillMap::GetResolution() {
	return _resolution;
}

Node* TreadmillMap::GetStart() {
	return _start;
}

Node* TreadmillMap::GetGoal() {
	return _goal;
}

void TreadmillMap::SetStart(float x, float y) {
	Node* target = CalcNodeFromCoordinate(x, y);
	//if (target == nullptr) return; actually let it crash
	_start->SetStart(false);
	_start = target;
	_start->SetStart(true);
	_thisCar->X = x;
	_thisCar->Y = y;
}

void TreadmillMap::SetGoal(float x, float y) {
	Node* target = CalcNodeFromCoordinate(x, y);
	//if (target == nullptr) return; actually let it crash
	_goal->SetGoal(false);
	_goal = target;
	_goal->SetGoal(true);
}

float TreadmillMap::CalcHeuristic(Node* node) {
	float h = CalcDist(node->X, _goal->X, node->Y, _goal->Y, false);
	node->SetHeuristic(h);
	return h;
}

void TreadmillMap::Print() {

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

vector<Node*> TreadmillMap::CalcNewObjectArea(MobileObstacle obj) {
	vector<Node*> result;

	// perform BFS on the nearby area starting at the centre node
	unordered_set<int> newNodeArea; // closed list of the search
	deque<Node*> openList;

	Node* rootNode = CalcNodeFromCoordinate(obj.X, obj.Y);
	openList.push_back(rootNode);

	while (openList.size() > 0) {
		Node* current = openList[0];
		openList.pop_front();
		vector<Node*> neighbours = current->GetAllAdjacent();
		for (int i = 0; i < neighbours.size(); i++) {
			Node* node = neighbours[i];
			if (node == nullptr) continue;

			if (CalcDist(node->X, obj.X, node->Y, obj.Y, false) < obj.Radius) {
				if (newNodeArea.count(node->GetID()) == 0) {
					result.push_back(node);
					newNodeArea.insert(node->GetID());
					openList.push_back(node);
				}
			}
		}
	} // BFS completes

	return result;
}

void TreadmillMap::UpdateMobileObstacles(float deltaTime) {
	// does not work for gradient descent!

	deque<MobileObstacle*> outOfBoundsList;

	for (int i = 0; i < _obstacleList.size(); i++) {
		MobileObstacle* obj = _obstacleList[i];

		obj->Move(deltaTime);
		vector<Node*> newArea = CalcNewObjectArea(*obj);

		// apply changes to the map

		for (int j = 0; j < obj->ActualArea.size(); j++) {
			obj->ActualArea[j]->IsObjectPresent = false; // node->SetOccupied(false);
		}
		obj->ActualArea.clear();
		for (int j = 0; j < newArea.size(); j++) {
			Node* node = newArea[j];
			node->IsObjectPresent = true; // node->SetOccupied(true);
			obj->ActualArea.push_back(node);
		}
		if (newArea.size() == 0)
			outOfBoundsList.push_back(obj); // mark for deletion
	}

	// delete the obstacles that moved off the map
	for (int i = 0; i < outOfBoundsList.size(); i++) {
		MobileObstacle* obj = outOfBoundsList[i];
		cout << "cleared projection due to obstacle leaving map" << endl;
		for (int j = 0; j < obj->ProjectionArea.size(); j++) {
			obj->ProjectionArea[j]->IsOccupationPredicted = false;
		}
		obj->ProjectionArea.clear();
		for (int j = 0; j < _obstacleList.size(); j++) {
			if (_obstacleList[j]->Id == obj->Id) {
				delete _obstacleList[j];
				_obstacleList.erase(_obstacleList.begin() + j);

				break;
			}
		}
	}
}

void TreadmillMap::ClearPath() {
	for (int i = 0; i < PathNodeList.size(); i++) {
		PathNodeList[i]->SetPath(false);
	}
	PathNodeList.clear();
}

void TreadmillMap::UpdateCurrentLocation(float deltaTime) {
	if (PathNodeList.size() == 0) return;

	Node* nextNode = PathNodeList.size() > 1 ? PathNodeList[1] : PathNodeList[0];
	_thisCar->Heading = atan2f(nextNode->Y - _thisCar->Y, nextNode->X - _thisCar->X);

	float distance = _thisCar->Velocity * deltaTime;
	float deltaX = distance * cosf(_thisCar->Heading);
	float deltaY = distance * sinf(_thisCar->Heading);

	_thisCar->X += deltaX;
	_thisCar->Y += deltaY;

	// update the path list
	// find the closest node in the path
	int cutoffIndex = 0;
	float distToCar = MAP_SIZE * 2;
	for (int i = 0; i < PathNodeList.size() - 1; i++) {
		Node* current = PathNodeList[i];
		float distToNode = CalcDist(current->X, _thisCar->X, current->Y, _thisCar->Y, false);
		if (distToNode < distToCar) {
			distToCar = distToNode;
			cutoffIndex = i;
		}
	}
	// set new start point for the next iteration of the search

	for (int i = 0; i < cutoffIndex; i++) {
		Node* current = PathNodeList[i];
		current->SetPath(false);
	}
	//cout << "cutoff: " << cutoffIndex << endl;
	_start->SetPath(false);
	_start->SetStart(false);
	_start = PathNodeList[cutoffIndex];
	_start->SetStart(true);

	// update collision prediction
	tuple<Node*, MobileObstacle*> result = FindCollisionPoint();
	Node* centrePoint = get<0>(result);
	MobileObstacle* collider = get<1>(result);

	if (centrePoint != nullptr && collider != nullptr) {
		// project this collider on to the centre point
		float nodeWidth = MAP_SIZE / (float)_resolution;
		MobileObstacle projection = MobileObstacle(0, 0, 0, centrePoint->X, centrePoint->Y, collider->Radius + 2 * nodeWidth);
		cout << projection.X << ", " << projection.Y << endl;
		vector<Node*> newArea = CalcNewObjectArea(projection);
		for (int i = 0; i < collider->ProjectionArea.size(); i++) {
			//collider->ProjectionArea[i]->IsOccupationPredicted = false; // node->SetOccupied(false);
		}
		//collider->ProjectionArea.clear();
		cout << newArea.size() << endl;
		for (int i = 0; i < newArea.size(); i++) {
			Node* node = newArea[i];
			node->IsOccupationPredicted = true; // node->SetOccupied(true);
			collider->ProjectionArea.push_back(node);
		}
	}
}

// the purpose of this method is to simulate future collision points
// on the current path and return the point of collision
tuple<Node*, MobileObstacle*> TreadmillMap::FindCollisionPoint() {
	if (PathNodeList.size() == 0) return tuple<Node*, MobileObstacle*>(nullptr, nullptr);

	float totalDist = 0;
	for (int i = 1; i < PathNodeList.size(); i++) {
		Node* current = PathNodeList[i];
		Node* previous = PathNodeList[i - 1];
		totalDist += CalcDist(current->X, previous->X, current->Y, previous->Y, false);
		float time = totalDist / _thisCar->Velocity;

		for (int j = 0; j < _obstacleList.size(); j++) {
			MobileObstacle* obj = _obstacleList[j];
			MobileObstacle projection = obj->SimulateMove(time);
			vector<Node*> newArea = CalcNewObjectArea(projection);
			for (int k = 0; k < newArea.size(); k++) {
				Node* node = newArea[k];
				if (node == CalcNodeFromCoordinate(previous->X, previous->Y)) {
					cout << "predicted path node index: " << i << endl;
					cout << "predicted collision coord: " << projection.X << ", " << projection.Y << endl;
					cout << "predicted collision time: " << time << endl;
					return tuple<Node*, MobileObstacle*>(node, obj); // only find the first collision point
				}
			}
		}
	}

	return tuple<Node*, MobileObstacle*>(nullptr, nullptr);
}

float TreadmillMap::CalcDist(float x1, float x2, float y1, float y2, bool useManhattan) {
	if (useManhattan)
		return fabs(x1 - x2) + fabs(y1 - y2);
	return sqrtf(powf(x1 - x2, 2) + powf(y1 - y2, 2));
}

float TreadmillMap::CalcGradientObsHeight(float distance, float radius, float padding) {
	if (distance <= radius) return FLT_MAX;
	if (distance < radius + padding)
		return MAP_SIZE / (distance - radius) - MAP_SIZE / (padding);
	return 0.0f;
}

Node* TreadmillMap::CalcNodeFromCoordinate(float x, float y) {
	int iApprox = (int)(roundf(_resolution - 0.5f - _resolution / MAP_SIZE*y)); // inverse function of y
	int jApprox = (int)(roundf(_resolution / MAP_SIZE*x - 0.5f)); // inverse function of x

	// restrict indices - just because the centre point is off the map does not mean the object is off the map (due to radius)
	// this point is used as a starting point for the search
	iApprox = (int)fmax(iApprox, 0);
	iApprox = (int)fmin(iApprox, _resolution - 1);
	jApprox = (int)fmax(jApprox, 0);
	jApprox = (int)fmin(jApprox, _resolution - 1);

	return map2d[iApprox][jApprox];
}