#pragma once

#include "TreadmillSystem.h"
#include "MobileObstacle.h"
#include <ctime>
#include <string>
#include <thread>
#include <chrono>
#include <math.h>

using namespace std;
using namespace std::chrono;

#define nullptr 0
//#define PI 3.14159f

int main() {
	char pause; // for final key input
	high_resolution_clock::time_point start = high_resolution_clock::now();
	float time = 0;

	while (true) {

		TreadmillSystem* mapSystem = new TreadmillSystem();
		mapSystem->SetTreadmillDimensions(100, 150, 0.2f);
		mapSystem->SetGoal(50, 60);
		//mapSystem->UpdateObstacle(10, 25, 55, 0, 0, 15); // test stationary obstacle
		//mapSystem->UpdateObstacle(101, 64, 45, -5, 5, 10); // test mobile obstacle
		//mapSystem->UpdateObstacle(102, 80, 100, -11.5, 0.1f, 20); // case 2 blocks goal temporarily
		mapSystem->UpdateObstacle(201, 50, 160, 0, -30, 20); // vertical blocking obstacle

		float carX = 5;
		float carY = 5;
		mapSystem->UpdateCar(carX, carY);
		//high_resolution_clock::time_point prevTime = high_resolution_clock::now();
		int msDelayForTesting = 211;

		float prevTime = 0;

		while (time < 6) {
			cout << endl;

			pair<float, float> waypoint = mapSystem->GetNextWaypoint();
			mapSystem->DebugPrint();

			float currentTime = duration_cast<duration<float>>(high_resolution_clock::now() - start).count();
			float deltaT = currentTime - prevTime;
			prevTime = currentTime;

			// assume the car arrives at the exact point specified (for this test)
			carX = waypoint.first;
			carY = waypoint.second;
			mapSystem->UpdateCar(carX, carY);
			this_thread::sleep_for(chrono::milliseconds(msDelayForTesting)); // add artifical delay for testing

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

		start = high_resolution_clock::now();
		time = 0;
	}


	while (true) {
		cin >> pause;
	}

	
	return 0;
}