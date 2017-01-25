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

void setup(TreadmillMap* currentMap) {

	currentMap->SetStart(0, 0);
	currentMap->SetGoal(15, 15);
	currentMap->AddObstacle(new MobileObstacle(2, -PI/2.0, 111, 9, 15, 2)); // TESTING
	currentMap->AddObstacle(new MobileObstacle(2, PI, 123, 10, 9, 2));
	currentMap->AddObstacle(new MobileObstacle(2, 0, 222, 0, 5, 2));
}

int main() {
	
	char pause; // for final key input
	TreadmillMap* temp = new TreadmillMap(10, 15, 2, MapGridOption::GridOccupation);
	setup(temp);


	clock_t start = clock();
	float time = (clock() - start) / (float)CLOCKS_PER_SEC;
	Node* originalStart = temp->GetStart();
	Node* originalGoal = temp->GetGoal();
	while (true) {
		while (time < 6) {
			float newTime = (clock() - start) / (float)CLOCKS_PER_SEC;
			cout << endl;
			this_thread::sleep_for(chrono::milliseconds(35)); // add artifical delay for testing

			float deltaTime = newTime - time;

			temp->UpdateMobileObstacles(deltaTime);
			TestResult debug = A_Star::FindPath(temp);
			temp->UpdateCurrentLocation(deltaTime);
			//temp->ClearProjection();

			cout << debug.solutionTime << endl;
			temp->Print();
			temp->ClearPath();

			time = newTime;
		}

		//temp->Print();
		cout << "restart? "<< endl;
		cin >> pause;

		temp->ClearObstacles();
		setup(temp);


		start = clock();
		time = 0;
	}
	delete temp;

	cin >> pause;
	
	return 0;
}