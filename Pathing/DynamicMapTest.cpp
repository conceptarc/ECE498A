#pragma once

#include "TreadmillSystem.h"
#include "MobileObstacle.h"
#include <ctime>
#include <string>
#include <thread>
#include <chrono>
#include <math.h>

using namespace std;

#define nullptr 0
//#define PI 3.14159f

void setup1(TreadmillMap* currentMap) {

	currentMap->SetStart(0, 0);
	currentMap->SetGoal(50, 90);

	// TESTING
	currentMap->AddObstacle(new MobileObstacle(18, -PI / 2.0, 111, 10, 150, 15));
	currentMap->AddObstacle(new MobileObstacle(22, -PI / 2.0, 222, 30, 150, 15));
	currentMap->AddObstacle(new MobileObstacle(26, -PI / 2.0, 333, 50, 150, 15));
	currentMap->AddObstacle(new MobileObstacle(45, -PI / 2.0f - 0.1f, 444, 100, 130, 15));
	//currentMap->AddObstacle(new MobileObstacle(15, PI, 123, 100, 90, 15));
	//currentMap->AddObstacle(new MobileObstacle(15, 0, 222, 0, 50, 15));
}

int main() {
	char pause; // for final key input
	clock_t start = clock();
	float time = (clock() - start) / (float)CLOCKS_PER_SEC;

	while (true) {

		TreadmillSystem* mapSystem = new TreadmillSystem();
		mapSystem->SetTreadmillDimensions(100, 150, 0.2f);
		mapSystem->SetGoal(50, 90);

		float carX = 5;
		float carY = 5;
		mapSystem->UpdateCar(carX, carY, 0, 0);

		while (time < 7) {
			float currentTime = (clock() - start) / (float)CLOCKS_PER_SEC;
			cout << endl;
			int msDelayForTesting = 110;
			this_thread::sleep_for(chrono::milliseconds(msDelayForTesting)); // add artifical delay for testing

			pair<float, float> waypoint = mapSystem->GetNextWaypoint();
			mapSystem->DebugPrint();

			float apparentSpeedX = (waypoint.first - carX) / msDelayForTesting * 1000.0f;
			float apparentSpeedY = (waypoint.second - carY) / msDelayForTesting * 1000.0f;
			// assume the car arrives at the exact point specified (for this test)
			float carNewX = waypoint.first;
			float carNewY = waypoint.second;
			mapSystem->UpdateCar(carNewX, carNewY, apparentSpeedX, apparentSpeedY);

			// update obstacles at some point

			/*
			// check N times for collision (N = number of obstacles) and recalc the path up to N-1 times
			int recalculateCount = 0;

			TestResult debug = A_Star::FindPath(temp);
			bool hasCollision = temp->UpdateCurrentLocation(deltaTime, currentTime);
			while (++recalculateCount < temp->GetObstacleCount() && hasCollision) {
				//temp->Print();
				temp->ClearPath();
				debug = A_Star::FindPath(temp);
				hasCollision = temp->UpdateCurrentLocation(deltaTime, currentTime);
			}
			//temp->ClearProjection();
			*/
			time = currentTime;
			//cout << currentTime/*debug.solutionTime*/ << endl;
			//temp->Print();
			//temp->ClearPath();

		}

		//temp->Print();
		cout << endl << "restart? " << endl;
		cin >> pause;
		delete mapSystem;

		//temp->ClearObstacles();
		//setup1(temp);


		start = clock();
		time = 0;
	}


	while (true) {
		cin >> pause;
	}

	
	return 0;
}