#pragma once
#include "MobileObstacle.h"
#include <iostream>

MobileObstacle::MobileObstacle() : Obstacle(0, 0, 0, 0) {
	ExpiryTime = -1;
	IsCar = false;
	dX = 0;
	dY = 0;
	// current deque is blank
}

MobileObstacle::MobileObstacle(int id, float x, float y, float dx, float dy, float radius) :
	dX(dx), dY(dy), Obstacle(id, x, y, radius) {
	// current deque is blank
	ExpiryTime = -1;
	IsCar = false;
}

MobileObstacle::~MobileObstacle() {
	// do not delete the map Nodes!
	for (int j = 0; j < ActualArea.size(); j++) {
		ActualArea[j]->IsObjectPresent = false;
	}
	//ClearProjection();
}

void MobileObstacle::Move(float deltaTime, float globalTime) {
	X = X + dX * deltaTime;
	Y = Y + dY * deltaTime;

	if (ExpiryTime > -1 && ExpiryTime < globalTime) {
		if (IsCar) {
			// in this case, we just delete the whole MobileObstacle
			// rely on the higher level to constantly increase the expiry time
			ClearProjection();
			ExpiryTime = -1;
			cout << "Other car " << Id << " has expired from lack of updates.\r\n";
		}
		else {
			cout << "Projection of Obst " << Id << " has expired.\r\n";
			ClearProjection();
			ExpiryTime = -1;
		}
	}
}

void MobileObstacle::SetExpiryTime(float expiryTime)
{
	ExpiryTime = expiryTime;
}

void MobileObstacle::ClearProjection()
{
	for (int j = 0; j < ProjectionArea.size(); j++) {
		ProjectionArea[j]->IsOccupationPredicted = false;
	}
	ProjectionArea.clear();
	
	// do similar protocol for gradient area
	// reset the heuristics that were modified by gradient descent
	if (GradientArea.size() > 0)
		cout << "Clearing gradient area sized: " << GradientArea.size() << endl;
	for (int j = 0; j < GradientArea.size(); j++) {
		GradientArea[j]->SetHeuristic(GradientArea[j]->GetHeuristicToGoal());
	}
	GradientArea.clear();
}

MobileObstacle MobileObstacle::SimulateMove(float deltaTime) {
	float newX = X + dX * deltaTime;
	float newY = Y + dY * deltaTime;

	return MobileObstacle(Id, newX, newY, dX, dY, Radius);
}
