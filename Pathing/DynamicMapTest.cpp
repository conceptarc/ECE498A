#include "Node.h"
#include "Map.h"
#include "A_Star.h"
#include "GradientDescent.h"
#include <ctime>
#include <string>
#include <thread>
#include <chrono>
#include "MobileObstacle.h"

using namespace std;


int main() {
	char pause; // for final key input

	MobileObstacle* obst = new MobileObstacle(25, 0, 123, -5, 50, 10);

	Map* temp = new Map(25, Occupation, Clear);
	temp->AddObstacle(obst);
	clock_t start = clock();
	float time = (clock() - start) / (float)CLOCKS_PER_SEC;
	while (true) {
		while (time < 5) {
			float newTime = (clock() - start) / (float)CLOCKS_PER_SEC;
			cout << endl;
			this_thread::sleep_for(chrono::milliseconds(500)); // add artifical delay for testing

			float deltaTime = newTime - time;

			temp->UpdateMobileObstacles(deltaTime);
			temp->ClearPath();
			A_Star::FindPath(temp);
			temp->Print();

			time = newTime;
		}

		//temp->Print();
		cout << "restart? "<< endl;

		cin >> pause;
		temp->AddObstacle(new MobileObstacle(25, 0, 123, -5, 50, 10));
		start = clock();
		time = 0;
	}
	delete temp;


	cin >> pause;
	return 0;
}