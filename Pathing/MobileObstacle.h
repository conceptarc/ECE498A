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
	deque<Node*> ActualArea;
	deque<Node*> ProjectionArea;

	MobileObstacle();
	MobileObstacle(float velocity, float heading, int id, float x, float y, float radius);
	~MobileObstacle();
	void Move(float deltaTime);
	MobileObstacle SimulateMove(float deltaTime);
};

#endif