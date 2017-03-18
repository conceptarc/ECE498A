#ifndef MOBILEOBSTACLE_H
#define MOBILEOBSTACLE_H

#include "Node.h"
#include <deque>
#include "Obstacle.h"

using namespace std;

class MobileObstacle : public Obstacle {
public:
	float dX; // velocity X component
	float dY; // velocity Y component
	deque<Node*> ActualArea;
	deque<Node*> ProjectionArea;
	float ExpiryTime;
	bool IsCar;

	MobileObstacle();
	MobileObstacle(int id, float x, float y, float dx, float dy, float radius);
	~MobileObstacle();
	void Move(float deltaTime, float globalTime);
	void SetExpiryTime(float expiryTime);
	void ClearProjection();
	MobileObstacle SimulateMove(float deltaTime);
};

#endif