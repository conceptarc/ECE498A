#ifndef MOBILEOBSTACLE_H
#define MOBILEOBSTACLE_H

#include "Node.h"
#include <deque>
#include "Obstacle.h"

using namespace std;

class MobileObstacle : public Obstacle {
public:
	float Velocity; // in ms^-1
	float Heading; // in radians
	deque<Node*> MapArea;

	MobileObstacle(float velocity, float heading, int id, float x, float y, float radius);
	~MobileObstacle();
	void Move(float deltaTime);
	MobileObstacle MoveReplace(float deltaTime);
};

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

MobileObstacle MobileObstacle::MoveReplace(float deltaTime) {
	float distance = Velocity * deltaTime;
	float deltaX = distance * cosf(Heading);
	float deltaY = distance * sinf(Heading);
	X = X + deltaX;
	Y = Y + deltaY;

	return MobileObstacle(Velocity, Heading, Id, X + deltaX, Y + deltaY, Radius);
}
#endif