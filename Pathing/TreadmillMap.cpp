#pragma once
#include <cfloat>
#include "TreadmillMap.h"

using namespace std;

TreadmillMap::TreadmillMap(int width, int length, float resolutionFactor) : _resolution(resolutionFactor) {
	// some constants / constraints
	_projectionHeight = (float)(width + length); // some constant that is approx eq to max heuristic value

	MAP_WIDTH_CM = width;
	MAP_LENGTH_CM = length;

	MAP_WIDTH_NODES = (int)round(width * resolutionFactor);
	MAP_LENGTH_NODES = (int)round(length * resolutionFactor);

	START_X = 0.0f; // init the values
	START_Y = 0.0f; // use void SetStart(float x, float y);	void SetGoal(float x, float y); to set these later
	GOAL_X = 0.0f;
	GOAL_Y = 0.0f;
	PADDING = 3; // n cm away from radius of obstacle


	// init _thisCar
	_thisCar = new MobileObstacle(0, 0, -1, 0, 0, 0);
	_start = NULL;
	_goal = NULL;
	
	
	// generate a fixed map with variable resolution
	// create 2D array of pointers
	map2d = new Node**[MAP_LENGTH_NODES]; // rows
	for (int i = 0; i < MAP_LENGTH_NODES; i++) {
		map2d[i] = new Node*[MAP_WIDTH_NODES]; // columns
	}

	// initialize 2D grid with empty nodes
	for (int i = 0; i < MAP_LENGTH_NODES; i++) {
		for (int j = 0; j < MAP_WIDTH_NODES; j++) {
			map2d[i][j] = new Node();
		}
	}

	// assign properties and references for each one
	float nodeWidth = 1 / resolutionFactor; // unit is cm (centimetres)
	//float startProximity = nodeWidth * 2; // used for finding the start node
	//float goalProximity = nodeWidth * 2; // used for finding the goal node

	// populate each node's properties with the information above
	for (int i = 0; i < MAP_LENGTH_NODES; i++) {
		for (int j = 0; j < MAP_WIDTH_NODES; j++) {
			Node* node = map2d[i][j];
			node->SetID(i*MAP_WIDTH_NODES + j); // ascending integer for ID

											// set 4-primary links
			node->SetNorth(i == 0 ? NULL : map2d[i - 1][j]);
			node->SetSouth(i == MAP_LENGTH_NODES - 1 ? NULL : map2d[i + 1][j]);
			node->SetWest(j == 0 ? NULL : map2d[i][j - 1]);
			node->SetEast(j == MAP_WIDTH_NODES - 1 ? NULL : map2d[i][j + 1]);

			// set 4-diagonal links
			node->SetNorthEast(i == 0 || j == MAP_WIDTH_NODES - 1 ? NULL : map2d[i - 1][j + 1]);
			node->SetNorthWest(i == 0 || j == 0 ? NULL : map2d[i - 1][j - 1]);
			node->SetSouthEast(i == MAP_LENGTH_NODES - 1 || j == MAP_WIDTH_NODES - 1 ? NULL : map2d[i + 1][j + 1]);
			node->SetSouthWest(i == MAP_LENGTH_NODES - 1 || j == 0 ? NULL : map2d[i + 1][j - 1]);

			// assign real world positions (cm) to each node
			float x = nodeWidth * j + nodeWidth / 2;
			float y = nodeWidth * (MAP_LENGTH_NODES - i - 1) + nodeWidth / 2;
			node->X = x;
			node->Y = y; // added for dynamic maps/obstacles
		}
	}
}

TreadmillMap::~TreadmillMap() {
	// clear obstacles before deleting the map
	for (int i = 0; i < _obstacleList.size(); i++) {
		delete _obstacleList[i];
	}
	_obstacleList.clear();

	for (int i = 0; i < MAP_LENGTH_NODES; i++) {
		for (int j = 0; j < MAP_WIDTH_NODES; j++) {
			delete map2d[i][j];
		}
		delete[] map2d[i];
	}
	delete[] map2d;
	delete _thisCar;
}

int TreadmillMap::GetMapWidthCm() {
	return MAP_WIDTH_CM; // read only value
}

int TreadmillMap::GetMapWidthNodes() {
	return MAP_WIDTH_NODES; // read only value
}

int TreadmillMap::GetMapLengthCm() {
	return MAP_LENGTH_CM; // read only value
}

int TreadmillMap::GetMapLengthNodes() {
	return MAP_LENGTH_NODES; // read only value
}

float TreadmillMap::CalcNodeWidthCm()
{
	return 1.0f / _resolution;
}

void TreadmillMap::AddObstacle(MobileObstacle* obj) {
	_obstacleList.push_back(obj);
}

deque<MobileObstacle*> TreadmillMap::GetObstacleList()
{
	return _obstacleList;
}

void TreadmillMap::ReplaceObstacleList(deque<MobileObstacle*> obsList) {
	for (int i = 0; i < _obstacleList.size(); i++) {
		bool hasFound = false;
		for (int j = 0; j < obsList.size(); j++) {
			if (_obstacleList[i]->Id == obsList[j]->Id) {
				hasFound = true;
				break;
			}
		}
		
		// clear the node state
		if (!hasFound) {
			delete _obstacleList[i];
		}
	}
	
	_obstacleList = obsList;
}

int TreadmillMap::GetObstacleCount()
{
	return (int)_obstacleList.size();
}

void TreadmillMap::ClearObstacles() {
	for (int i = 0; i < _obstacleList.size(); i++) {
		delete _obstacleList[i];
	}
	_obstacleList.clear();
}

void TreadmillMap::ClearProjection()
{
	for (int i = 0; i < _obstacleList.size(); i++) {
		_obstacleList[i]->ClearProjection();
	}
}

float TreadmillMap::GetResolution() {
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
	START_X = target->X;
	START_Y = target->Y;
	if (_start != NULL)
		_start->SetStart(false);
	_start = target;
	_start->SetStart(true);

	_thisCar->X = x; // move this elsewhere?
	_thisCar->Y = y;
}

void TreadmillMap::SetGoal(float x, float y) {
	Node* target = CalcNodeFromCoordinate(x, y);
	GOAL_X = target->X;
	GOAL_Y = target->Y;
	if (_goal != NULL) {
		_goal->SetGoal(false);
	}
	_goal = target;
	//printf("TreadmillMap.cpp - Set goal to Node ID: %d\n", _goal->GetID());
	_goal->SetGoal(true);
	
	// whenever the goal is moved, the map weighting must be recalculated
	for (int i = 0; i < MAP_LENGTH_NODES; i++) {
		for (int j = 0; j < MAP_WIDTH_NODES; j++) {
			// get heuristic based on distance to goal node
			Node* node = map2d[i][j];
			float distToStart = CalcDist(x, START_X, y, START_Y);
			float distToGoal = CalcDist(x, GOAL_X, y, GOAL_Y);

			node->SetHeuristic(distToGoal); // calculated above, might as well use it
			node->SetHeuristicToGoal(distToGoal); // calculated above, might as well use it
		}
	}
}

MobileObstacle * const TreadmillMap::GetThisCar()
{
	return _thisCar;
}

float TreadmillMap::CalcHeuristic(Node* node) {
	float h = CalcDist(node->X, _goal->X, node->Y, _goal->Y);
	node->SetHeuristic(h);
	node->SetHeuristicToGoal(h);
	//if (node->IsOccupationPredicted) return h + _projectionHeight;
	return h;
}

void TreadmillMap::Print() {

	//debug print nodes
	Node* column = map2d[0][0];
	Node* row = map2d[0][0];

	for (int i = 0; i < MAP_WIDTH_NODES + 2; i++) {
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

	for (int i = 0; i < MAP_WIDTH_NODES + 2; i++) { // bottom boundary
		cout << '-';
	}
}

deque<Node*> TreadmillMap::CalcNewObjectArea(MobileObstacle obj, float paddingCm) {
	deque<Node*> result;

	// perform BFS on the nearby area starting at the centre node
	unordered_set<int> newNodeArea; // closed list of the search
	deque<Node*> openList;

	Node* rootNode = CalcNodeFromCoordinate(obj.X, obj.Y);
	openList.push_back(rootNode);

	while (openList.size() > 0) {
		Node* current = openList[0];
		openList.pop_front();
		deque<Node*> neighbours = current->GetAllAdjacent();
		for (int i = 0; i < neighbours.size(); i++) {
			Node* node = neighbours[i];
			if (node == nullptr || node == NULL) continue;

			if (CalcDist(node->X, obj.X, node->Y, obj.Y) < obj.Radius + paddingCm) {
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

void TreadmillMap::UpdateMobileObstacles(float deltaTime, float currentTime) {

	deque<MobileObstacle*> outOfBoundsList;

	// simulate/project obstacle locations
	for (int i = 0; i < _obstacleList.size(); i++) {
		MobileObstacle* obj = _obstacleList[i];

		for (int j = 0; j < obj->ActualArea.size(); j++) {
			obj->ActualArea[j]->IsObjectPresent = false;
		}
		obj->ActualArea.clear();

		// move the object
		obj->Move(deltaTime, currentTime);
		// cout << "deltaT: " << deltaTime << endl;
		deque<Node*> newArea = CalcNewObjectArea(*obj, PADDING);

		// apply changes to the map
		for (int j = 0; j < newArea.size(); j++) {
			Node* node = newArea[j];
			node->IsObjectPresent = true;
			obj->ActualArea.push_back(node);
		}
		if (newArea.size() == 0)
			outOfBoundsList.push_back(obj); // mark for deletion
	}

	// delete the obstacles that moved off the map
	for (int i = 0; i < outOfBoundsList.size(); i++) {
		MobileObstacle* obj = outOfBoundsList[i];
		cout << "cleared projection due to obstacle leaving map" << endl;
		obj->ClearProjection();
		
		int idFound = -1;
		
		for (int j = 0; j < _obstacleList.size(); j++) {
			if (_obstacleList[j]->Id == obj->Id) {
				idFound = j;
				break;
			}
		}
		
		if (idFound != -1) {
			delete _obstacleList[idFound];
			_obstacleList.erase(_obstacleList.begin() + idFound);
		}
	}

	// perform collision prediction // do it once per obstacle
	for (int i = 0; i < _obstacleList.size(); i++) {
		tuple<Node*, MobileObstacle*> result = FindNextCollisionPoint(currentTime, _obstacleList[i]);
		Node* centrePoint = get<0>(result);
		MobileObstacle* collider = get<1>(result);

		if (centrePoint != nullptr && collider != nullptr) {
			// project this collider on to the centre point
			MobileObstacle projection = MobileObstacle(0, centrePoint->X, centrePoint->Y, 0, 0, collider->Radius);
			// pad by 5 cm
			deque<Node*> newProjectionArea = CalcNewObjectArea(projection, 3);

			//cout << newProjectionArea.size() << endl;
			for (int i = 0; i < newProjectionArea.size(); i++) {
				Node* node = newProjectionArea[i];
				node->IsOccupationPredicted = true; // node->SetOccupied(true);
				collider->ProjectionArea.push_back(node);
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

// the purpose of this method is to simulate future collision points
// on the current path and return the point of collision
tuple<Node*, MobileObstacle*> TreadmillMap::FindNextCollisionPoint(float currentTime, MobileObstacle* obst) {
	if (PathNodeList.size() == 0) return tuple<Node*, MobileObstacle*>(nullptr, nullptr);

	float totalDist = 0;
	for (int i = 1; i < PathNodeList.size(); i++) {
		Node* current = PathNodeList[i];
		Node* previous = PathNodeList[i - 1];
		totalDist += CalcDist(current->X, previous->X, current->Y, previous->Y);
		float speed = CalcDist(_thisCar->dX, 0, _thisCar->dY, 0); // reusing this equation
		//cout << "p Speed: " << speed << endl;
		//cout << "p Dist: " << totalDist << endl;

		float time = totalDist / speed; // distance is in cm but dx and dy are also cm/s
		//cout << "predict time: " << time << endl;

		MobileObstacle projection = obst->SimulateMove(time);
		deque<Node*> newArea = CalcNewObjectArea(projection, 0);

		// check if any of the projected area overlaps with the car path
		for (int k = 0; k < newArea.size(); k++) {
			Node* node = newArea[k];
			if (node == previous) { // collision detected
				//cout << "Obstacle " << obj->Id << " has projection expiry t=" << currentTime + time << endl;
				// we should set a max expiry time limit because future predictions are inaccurate
				// while the projections are absolute law.
				if (time > 2) time = 2; // hard cap to be no more than 5 sec in future
				obst->SetExpiryTime(currentTime + time + 0.0f); // linger for additional N seconds

				//cout << "predicted path node index: " << i << endl;
				//cout << "predicted collision coord: " << projection.X << ", " << projection.Y << endl;
				cout << "predicted collision time: " << (currentTime + time) << endl;
				Node* projectionCentre = CalcNodeFromCoordinate(projection.X, projection.Y);
				return tuple<Node*, MobileObstacle*>(projectionCentre, obst); // only find the first collision point
			}
		}
	}

	return tuple<Node*, MobileObstacle*>(nullptr, nullptr);
}

void TreadmillMap::ProjectAllObstaclesForGD(float deltaTime, float currentTime)
{
	float gradientRadiusInfluencePadding = 30; // N cm

	// clear the projections and arbitrarily project it at a certain time in the future
	for (int i = 0; i < _obstacleList.size(); i++) {
		MobileObstacle* obst = _obstacleList[i];
		obst->ClearProjection(); // this now clears the gradient heuristics

		MobileObstacle projection = obst->SimulateMove(deltaTime);
		deque<Node*> newArea = CalcNewObjectArea(projection, PADDING + gradientRadiusInfluencePadding); // also contains nodes surrounding the obstacle
		
		if (newArea.size() > 0) {
			// insert the new projection
			for (int j = 0; j < newArea.size(); j++) {
				Node* node = newArea[j];
				float distanceFromCentre = CalcDist(node->X, projection.X, node->Y, projection.Y);
				float gradientPenalty = CalcGradientObsHeight(distanceFromCentre, projection.Radius, PADDING + gradientRadiusInfluencePadding);

				node->SetHeuristic(node->GetHeuristicToGoal() + gradientPenalty);

				obst->GradientArea.push_back(node);
				if (distanceFromCentre <= projection.Radius + PADDING) {
					node->IsOccupationPredicted = true;
					obst->ProjectionArea.push_back(node);
				}
			}

			obst->SetExpiryTime(currentTime + deltaTime*0.5f);
		}
	}
}

float TreadmillMap::CalcDist(float x1, float x2, float y1, float y2) {
	return sqrtf(powf(x1 - x2, 2) + powf(y1 - y2, 2));
}

float TreadmillMap::CalcGradientObsHeight(float distance, float radius, float padding) {
	/*if (distance <= radius) return 10000;//FLT_MAX;
	if (distance < radius + padding) {
		float heightPenaltyFactor = 100;
		float gradient = MAP_WIDTH_CM / (distance - radius) - MAP_WIDTH_CM / (padding);
		//if (gradient < 10000)
			//cout << "GD penalty: dist = "  << distance << ", val: " << gradient*heightPenaltyFactor << endl;
		return gradient * heightPenaltyFactor; // heightPenaltyFactor increases the gradient
	}
	return 0.0f;*/

	// new gradient function: no plateau in centre

	float heightPenaltyFactor = 20;
	float gradient = MAP_WIDTH_CM / (distance) - MAP_WIDTH_CM / (radius + padding);
	//if (gradient < 10000)
	//cout << "GD penalty: dist = "  << distance << ", val: " << gradient*heightPenaltyFactor << endl;
	return gradient * heightPenaltyFactor; // heightPenaltyFactor increases the gradient
}

Node* TreadmillMap::CalcNodeFromCoordinate(float x, float y) {
	int iApprox = (int)(roundf(MAP_LENGTH_NODES - 0.5f - y / CalcNodeWidthCm())); // inverse function of vertical coord
	int jApprox = (int)(roundf(x/CalcNodeWidthCm() - 0.5f)); // inverse function of horizontal coord

	// restrict indices - just because the centre point is off the map does not mean the object is off the map (due to radius)
	// this point is used as a starting point for the search
	iApprox = (int)fmax(iApprox, 0);
	iApprox = (int)fmin(iApprox, MAP_LENGTH_NODES - 1);
	jApprox = (int)fmax(jApprox, 0);
	jApprox = (int)fmin(jApprox, MAP_WIDTH_NODES - 1);

	return map2d[iApprox][jApprox];
}
