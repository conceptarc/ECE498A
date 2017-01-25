#pragma once

#include "TreadmillMap.h"
#include "MobileObstacle.h"
#include "A_Star.h"
#include "GradientDescent.h"
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
	TreadmillMap* temp = new TreadmillMap(100, 150, 0.2f, MapGridOption::GridOccupation);
	setup1(temp);


	clock_t start = clock();
	float time = (clock() - start) / (float)CLOCKS_PER_SEC;
	Node* originalStart = temp->GetStart();
	Node* originalGoal = temp->GetGoal();
	while (true) {
		while (time < 7) {
			float currentTime = (clock() - start) / (float)CLOCKS_PER_SEC;
			cout << endl;
			this_thread::sleep_for(chrono::milliseconds(105)); // add artifical delay for testing

			float deltaTime = currentTime - time;

			temp->UpdateMobileObstacles(deltaTime, currentTime);

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

			time = currentTime;
			cout << currentTime/*debug.solutionTime*/ << endl;
			temp->Print();
			temp->ClearPath();

		}

		//temp->Print();
		cout << "restart? "<< endl;
		cin >> pause;

		temp->ClearObstacles();
		setup1(temp);


		start = clock();
		time = 0;
	}
	delete temp;

	cin >> pause;
	
	return 0;
}