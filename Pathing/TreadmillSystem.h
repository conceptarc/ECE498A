#pragma once
#include <deque>
#include <chrono>
#include "TreadmillMap.h"
#include "MobileObstacle.h"
#include "A_Star.h"
#include "GradientDescent.h"

using namespace std;

// The purpose of this class is to combine the functionality of TreadmillMap
// with the two pathing algorithms: A* and Gradient. This will be the main
// interface with the ROS side of the FYDP.
class TreadmillSystem {

private:
	TreadmillMap* map;
	clock_t initTime;

	float CurrentTime();

public:
	TreadmillSystem();
	~TreadmillSystem();

	void SetTreadmillDimensions(int width, int length, float resolution);
	void SetGoal(float x, float y); // x -> width, y -> height
	void UpdateCar(float x, float y, float dx, float dy); // dx, dy are velocity components
	pair<float, float> GetNextWaypoint();

	void UpdateObstacle(int id, float x, float y, float dx, float dy, float radius);
	void RemoveObstacle(int id);
	void RemoveObstaclesExcept(vector<int> idList);

	void DebugPrint();
};