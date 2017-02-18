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
}

pair<float, float> TreadmillSystem::GetNextWaypoint()
{
	float nextX = map->GetThisCar()->X;
	float nextY = map->GetThisCar()->Y;
	map->SetStart(nextX, nextY);
	// use a combination of A* and Gradient to output the next waypoint
	map->ClearPath();
	if (A_Star::FindPath(map) && map->PathNodeList.size() > 1) {
		nextX = map->PathNodeList[1]->X;
		nextY = map->PathNodeList[1]->Y;
	} else {
		map->ClearPath();
		// try again with GradientDescent
	}

	return pair<float, float>(nextX, nextY);
}

void TreadmillSystem::DebugPrint()
{
	map->Print();
}
