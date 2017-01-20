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

int main() {
	
	char pause; // for final key input
	TreadmillMap* temp = new TreadmillMap(120, 220, 1, MapGridOption::GridOccupation);
	//temp->SetGoal(15, 15);
	temp->AddObstacle(new MobileObstacle(25, -2 / 4.0 * PI, 123, 40, 90, 10)); // TESTING
	temp->AddObstacle(new MobileObstacle(25, PI, 123, 100, 70, 10));
	//temp->AddObstacle(new MobileObstacle(25, -2.5 / 4.0 * PI, 123, 90, 90, 10));
	//temp->AddObstacle(new MobileObstacle(20, -3 / 4.0 * PI, 123, 90, 90, 10));
	clock_t start = clock();
	float time = (clock() - start) / (float)CLOCKS_PER_SEC;
	Node* originalStart = temp->GetStart();
	Node* originalGoal = temp->GetGoal();
	while (true) {
		while (time < 6) {
			float newTime = (clock() - start) / (float)CLOCKS_PER_SEC;
			cout << endl;
			this_thread::sleep_for(chrono::milliseconds(100)); // add artifical delay for testing

			float deltaTime = newTime - time;

			temp->UpdateMobileObstacles(deltaTime);
			TestResult debug = A_Star::FindPath(temp);
			temp->UpdateCurrentLocation(deltaTime);

			cout << debug.solutionTime << endl;
			temp->Print();
			temp->ClearPath();

			time = newTime;
		}

		//temp->Print();
		cout << "restart? "<< endl;

		temp->ClearObstacles();
		temp->SetStart(15, 15);
		//temp->SetGoal(15, 15);
		cin >> pause;
		temp->AddObstacle(new MobileObstacle(25, -2 / 4.0 * PI, 123, 40, 90, 10));
		temp->AddObstacle(new MobileObstacle(25, PI, 123, 100, 70, 10));
		//temp->AddObstacle(new MobileObstacle(25, -2.5 / 4.0 * PI, 123, 90, 90, 10));
		//temp->AddObstacle(new MobileObstacle(20, -3 / 4.0 * PI, 123, 90, 90, 10));
		start = clock();
		time = 0;
	}
	delete temp;

	cin >> pause;
	
	return 0;
}