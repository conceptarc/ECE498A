#pragma once
#include "MobileObstacle.h"

MobileObstacle::MobileObstacle() : Obstacle(0, 0, 0, 0) {
	// current deque is blank
}

MobileObstacle::MobileObstacle(float velocity, float heading, int id, float x, float y, float radius) :
	Velocity(velocity), Heading(heading), Obstacle(id, x, y, radius) {
	// current deque is blank
}

MobileObstacle::~MobileObstacle() {
	// do not delete the map Nodes!
}

void MobileObstacle::Move(float deltaTime) {
	float distance = Velocity * deltaTime;
	float deltaX = distance * cosf(Heading);
	float deltaY = distance * sinf(Heading);

	X = X + deltaX;
	Y = Y + deltaY;
}

MobileObstacle MobileObstacle::SimulateMove(float deltaTime) {
	float distance = Velocity * deltaTime;
	float deltaX = distance * cosf(Heading);
	float deltaY = distance * sinf(Heading);

	return MobileObstacle(Velocity, Heading, Id, X + deltaX, Y + deltaY, Radius);
}