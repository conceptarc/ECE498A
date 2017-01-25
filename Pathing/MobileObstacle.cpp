#pragma once
#include "MobileObstacle.h"

MobileObstacle::MobileObstacle() : Obstacle(0, 0, 0, 0) {
	// current deque is blank
}

MobileObstacle::MobileObstacle(float velocity, float heading, int id, float x, float y, float radius) :
	Velocity(velocity), Heading(heading), Obstacle(id, x, y, radius) {
	// current deque is blank
	ExpiryTime = -1;
}

MobileObstacle::~MobileObstacle() {
	// do not delete the map Nodes!
}

void MobileObstacle::Move(float deltaTime, float globalTime) {
	float distance = Velocity * deltaTime;
	float deltaX = distance * cosf(Heading);
	float deltaY = distance * sinf(Heading);

	X = X + deltaX;
	Y = Y + deltaY;

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
	float distance = Velocity * deltaTime;
	float deltaX = distance * cosf(Heading);
	float deltaY = distance * sinf(Heading);

	return MobileObstacle(Velocity, Heading, Id, X + deltaX, Y + deltaY, Radius);
}