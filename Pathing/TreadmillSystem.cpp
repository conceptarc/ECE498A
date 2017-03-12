#pragma once
#include "TreadmillSystem.h"

using namespace std;
using namespace std::chrono;

float TreadmillSystem::CurrentTime()
{
	return duration_cast<duration<float>>(high_resolution_clock::now() - initTime).count();;
}

TreadmillSystem::TreadmillSystem() {
	map = NULL;
	initTime = high_resolution_clock::now();
	prevTime = 0;
}

TreadmillSystem::~TreadmillSystem() {
	if (map != NULL) {
		delete map;
	}
}

void TreadmillSystem::SetTreadmillDimensions(int width, int length, float resolution)
{
	if (map != NULL) {
		delete map;
	}
	map = new TreadmillMap(width, length, resolution);
}

void TreadmillSystem::SetGoal(float x, float y)
{
	map->SetGoal(x, y);
}

void TreadmillSystem::UpdateCar(float x, float y, float dx, float dy)
{
	MobileObstacle* car = map->GetThisCar();
	car->X = x;
	car->Y = y;
	car->dX = dx;
	car->dY = dy;
}

pair<float, float> TreadmillSystem::GetNextWaypoint()
{
	float nextX = map->GetThisCar()->X;
	float nextY = map->GetThisCar()->Y;
	map->SetStart(nextX, nextY);
	// use a combination of A* and Gradient to output the next waypoint

	// update the obstacles before calculating the path
	float timeSnapshot = CurrentTime();
	cout << "Time since last waypoint request: " << timeSnapshot - prevTime << endl;
	cout << "Current time: " << timeSnapshot << endl;
	
	// track and predict obstacles based on current information
	map->UpdateMobileObstacles(prevTime, timeSnapshot);
	// ^ very important to do before the path is cleared

	prevTime = timeSnapshot;

	map->ClearPath(); // only clear the path just before recalculation
	bool aStarSuccess = A_Star::FindPath(map);
	// Due to the (lack of) sensitivity of the car controller, if a given waypoint is
	// too near the current car location, the car simply does not move. Therefore
	// we should pass a waypoint that is 2 nodes distance away.
	if (aStarSuccess && map->PathNodeList.size() > 2) {
		nextX = map->PathNodeList[2]->X;
		nextY = map->PathNodeList[2]->Y;
		cout << "A* has found a path." << endl;
	} else if (aStarSuccess && map->PathNodeList.size() > 1) {
		nextX = map->PathNodeList[1]->X;
		nextY = map->PathNodeList[1]->Y;
		cout << "A* has found a path." << endl;
	} else {
		cout << "A* found no path, switching to gradient descent." << endl;
		map->ClearPath();
		//map->ClearProjection();
		/*// try again with GradientDescent
		// WIP
		GradientDescent::FindPath(map);
		if (map->PathNodeList.size() > 1) {
			nextX = map->PathNodeList[0]->X;
			nextY = map->PathNodeList[0]->Y;
		}*/
	}

	return pair<float, float>(nextX, nextY);
}

void TreadmillSystem::UpdateObstacle(int id, float x, float y, float dx, float dy, float radius)
{
	deque<MobileObstacle*> existingObstacles = map->GetObstacleList();
	bool foundObstacle = false;
	for (int i = 0; i < existingObstacles.size(); i++) {
		if (existingObstacles[i]->Id == id) {
			foundObstacle = true;
			existingObstacles[i]->X = x;
			existingObstacles[i]->Y = y;
			existingObstacles[i]->dX = dx;
			existingObstacles[i]->dY = dy;
			existingObstacles[i]->Radius = radius;
		}
	}

	// otherwise add this new obstacle
	if (!foundObstacle) {
		printf("new obstacle ID: %d\n", id);
		MobileObstacle* additionalObstacle = new MobileObstacle(id, x, y, dx, dy, radius);
		map->AddObstacle(additionalObstacle);
	}
}

void TreadmillSystem::RemoveObstaclesExcept(vector<int> idList)
{
	deque<MobileObstacle*> existingObstacles = map->GetObstacleList();
	
	// create a new deque and replace the old one with the new one
	deque<MobileObstacle*> newObstacleSet;

	for (int i = 0; i < existingObstacles.size(); i++) {

		// check for ID match and then populate the new collection
		for (int j = 0; j < idList.size(); j++) {
			if (existingObstacles[i]->Id == idList[j]) {
				newObstacleSet.push_back(existingObstacles[i]);
				break;
			}
		}
	}

	map->ReplaceObstacleList(newObstacleSet);
}

void TreadmillSystem::DebugPrint()
{
	map->Print();
}
