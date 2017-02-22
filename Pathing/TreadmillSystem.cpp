#pragma once
#include "TreadmillSystem.h"

using namespace std;

float TreadmillSystem::CurrentTime()
{
	return (clock() - initTime) / (float)CLOCKS_PER_SEC;
}

TreadmillSystem::TreadmillSystem() {
	map = NULL;
	initTime = clock();
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
	if (A_Star::FindPath(map) && map->PathNodeList.size() > 1) {
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
		MobileObstacle* additionalObstacle = new MobileObstacle(id, x, y, dx, dy, radius);
		map->AddObstacle(additionalObstacle);
	}
}

void TreadmillSystem::DebugPrint()
{
	map->Print();
}
