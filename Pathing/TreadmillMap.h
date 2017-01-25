#pragma once

#include <math.h>
#include <deque>
#include <vector>
#include <iostream>
#include <tuple>
#include "Node.h"
#include "MobileObstacle.h"
//#include "Map.h"

#define nullptr 0
#define PI 3.14159f

using namespace std;

enum class MapGridOption {
	GridOccupation,
	GridGradient
};

class TreadmillMap {

private:
	int MAP_WIDTH_CM;
	int MAP_WIDTH_NODES;

	int MAP_LENGTH_CM;
	int MAP_LENGTH_NODES;

	float START_X;
	float START_Y;
	float GOAL_X;
	float GOAL_Y;
	float PADDING;

	float _projectionHeight;

	float _resolution;
	Node*** map2d;
	Node* _start;
	Node* _goal;
	Node* _topLeft;
	deque<MobileObstacle*> _obstacleList;
	MobileObstacle* _thisCar;

	float CalcDist(float x1, float x2, float y1, float y2, bool useManhattan);
	float CalcGradientObsHeight(float distance, float radius, float padding);
	Node* CalcNodeFromCoordinate(float x, float y);
	vector<Node*> CalcNewObjectArea(MobileObstacle obj);

public:
	// new constructor for a custom map size that fits the treadmill
	// resolution factor = number of tiles : number of cm -> e.g. 5 cm * 5 cm with factor 2 = 10 tile * 10 tile grid
	TreadmillMap(int width, int length, float resolutionFactor, MapGridOption option); // width & length are in cm
	~TreadmillMap();

	int GetMapWidthCm();
	int GetMapWidthNodes();
	int GetMapLengthCm();
	int GetMapLengthNodes();
	float CalcNodeWidthCm();

	void AddObstacle(MobileObstacle* obj);
	void ClearObstacles();
	void ClearProjection();
	float GetResolution();
	Node* GetStart();
	Node* GetGoal();
	void SetStart(float x, float y);
	void SetGoal(float x, float y);
	float CalcHeuristic(Node* node);

	void Print();
	void UpdateMobileObstacles(float deltaTime); // moving obstacles without generating the path
	deque<Node*> PathNodeList;
	void ClearPath();
	void UpdateCurrentLocation(float deltaTime); // update the start location
	tuple<Node*, MobileObstacle*> FindCollisionPoint(); // costly to check + simulate the future collisions
};
