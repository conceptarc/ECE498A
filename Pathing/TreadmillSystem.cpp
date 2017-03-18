#pragma once
#include "TreadmillSystem.h"

using namespace std;
using namespace std::chrono;

float TreadmillSystem::CurrentTime()
{
	return duration_cast<duration<float>>(high_resolution_clock::now() - initTime).count();
}

TreadmillSystem::TreadmillSystem() {
	map = NULL;
	initTime = high_resolution_clock::now();
	prevTime = 0;
	prevCarTime = 0;
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

void TreadmillSystem::UpdateCar(float x, float y)
{
	MobileObstacle* car = map->GetThisCar();

	// we need to estimate the average car velocity based on the last time the car was updated
	float deltaX = x - car->X;
	float deltaY = y - car->Y;
	float deltaT = CurrentTime() - prevCarTime;// cout << "delta T car: " << deltaT << endl;
	prevCarTime = CurrentTime();

	car->X = x;
	car->Y = y;
	car->dX = deltaX / deltaT;// cout << "car dx: " << car->dX << endl;
	car->dY = deltaY / deltaT;// cout << "car dy: " << car->dX << endl;
}

pair<float, float> TreadmillSystem::GetNextWaypoint()
{
	float nextX = map->GetThisCar()->X;
	float nextY = map->GetThisCar()->Y;
//	cout << "Prev X " << nextX << " \r\n";
//	cout << "Prev Y " << nextY << " \r\n";

	map->SetStart(nextX, nextY);
	// use a combination of A* and Gradient to output the next waypoint

	// update the obstacles before calculating the path
	float timeSnapshot = CurrentTime();
	float deltaTime = timeSnapshot - prevTime;
	cout << "Time since last waypoint request: " << deltaTime << endl;
	cout << "Current time: " << timeSnapshot << endl;
	
	// track and predict obstacles based on current information
	map->UpdateMobileObstacles(deltaTime, timeSnapshot);
	// ^ very important to do before the path is cleared

	prevTime = timeSnapshot;

	//if (!savePath) // save the last good path if we fail to calculate a new one
	map->ClearPath();

	bool aStarSuccess = A_Star::FindPath(map);
	// Due to the (lack of) sensitivity of the car controller, if a given waypoint is
	// too near the current car location, the car simply does not move. Therefore
	// we should pass a waypoint that is 2 nodes distance away.
	int minAStarPathSize = 7; // HARD CODED MAGIC NUMBER 7 = limit at which gradient descent triggers
	if (aStarSuccess && map->PathNodeList.size() >= minAStarPathSize) {
		nextX = map->PathNodeList[3]->X;
		nextY = map->PathNodeList[3]->Y;
		cout << "A* has found a path." << endl;
	} else {
		// else the path is too short
		// A* is not reliable in avoiding future collisions when the car is already near the destination

		cout << "A* found no path, switching to gradient descent." << endl;
		// time for GD
		map->ClearPath();
		map->ClearProjection();
		//map->UpdateMobileObstacles(deltaTime, timeSnapshot); // gotta do this again

		float gradientFuturePrediction = 40; // should be seconds but really is not for some reason
		map->ProjectAllObstaclesForGD(gradientFuturePrediction, timeSnapshot);
		GradientDescent::FindPath(map);

		if (map->PathNodeList.size() > 2) {
			cout << "Grad Desc has found a path \r\n";
			nextX = map->PathNodeList[2]->X;
			nextY = map->PathNodeList[2]->Y;
		} else if (map->PathNodeList.size() > 1) {
			cout << "Grad Desc has found a path \r\n";
			nextX = map->PathNodeList[1]->X;
			nextY = map->PathNodeList[1]->Y;
		} else if (map->PathNodeList.size() > 0) {
			cout << "Grad Desc has found a path \r\n";
			nextX = map->PathNodeList[0]->X;
			nextY = map->PathNodeList[0]->Y;
		}
	}

	return pair<float, float>(nextX, nextY);
}

void TreadmillSystem::UpdateOtherCar(int id, float x, float y, float timeout)
{
	float carRadius = 10; // N cm buffer
	deque<MobileObstacle*> existingObstacles = map->GetObstacleList();
	bool foundCar = false;
	for (int i = 0; i < existingObstacles.size(); i++) {
		if (existingObstacles[i]->Id == id) {
			foundCar = true;
			existingObstacles[i]->X = x;
			existingObstacles[i]->Y = y;
			existingObstacles[i]->dX = 0;
			existingObstacles[i]->dY = 0;
			existingObstacles[i]->Radius = carRadius; // hard coded car radius
		}
	}

	// otherwise add this new other car
	if (!foundCar) {
		printf("other car ID: %d\n", id);
		MobileObstacle* additionalObstacle = new MobileObstacle(id, x, y, 0, 0, carRadius);
		additionalObstacle->IsCar = true;
		additionalObstacle->SetExpiryTime(timeout);
		map->AddObstacle(additionalObstacle);
	}
	existingObstacles = map->GetObstacleList();
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
	deque<MobileObstacle*> deleteObstacleSet;

	for (int i = 0; i < existingObstacles.size(); i++) {

		// check for ID match and then populate the new collection
		bool hasMatch = false;
		if (existingObstacles[i]->IsCar) {
			hasMatch = true;
			newObstacleSet.push_back(existingObstacles[i]);
			
		} else {
			for (int j = 0; j < idList.size(); j++) {
				if (existingObstacles[i]->Id == idList[j]) {
					hasMatch = true;
					newObstacleSet.push_back(existingObstacles[i]);
					break;
				}
			}
		}
		
		if (!hasMatch) {
			deleteObstacleSet.push_back(existingObstacles[i]);
		}
	}
	
	// we should manually clear the projections for the obstacles that we delete here
	for (int i = 0; i < deleteObstacleSet.size(); i++) {
		deleteObstacleSet[i]->ClearProjection();
	}

	map->ReplaceObstacleList(newObstacleSet);
}

void TreadmillSystem::DebugPrint()
{
	map->Print();
}
