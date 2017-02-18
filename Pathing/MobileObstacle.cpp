#pragma once
#include "MobileObstacle.h"

MobileObstacle::MobileObstacle() : Obstacle(0, 0, 0, 0) {
	// current deque is blank
}

MobileObstacle::MobileObstacle(int id, float x, float y, float dx, float dy, float radius) :
	dX(dx), dY(dy), Obstacle(id, x, y, radius) {
	// current deque is blank
	ExpiryTime = -1;
}

MobileObstacle::~MobileObstacle() {
	// do not delete the map Nodes!
}

void MobileObstacle::Move(float deltaTime, float globalTime) {
	X = X + dX * deltaTime;
	Y = Y + dY * deltaTime;

	if (ExpiryTime > -1 && ExpiryTime < globalTime) {
		ClearProjection();
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
}

MobileObstacle MobileObstacle::SimulateMove(float deltaTime) {
	float newX = X + dX * deltaTime;
	float newY = Y + dY * deltaTime;

	return MobileObstacle(Id, newX, newY, dX, dY, Radius);
}