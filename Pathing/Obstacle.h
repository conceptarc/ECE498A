#ifndef OBSTACLE_H
#define OBSTACLE_H

#define nullptr 0

class Obstacle {
public:
	Obstacle(int id, float x, float y, float radius);
	~Obstacle();
	int Id;
	float X;
	float Y;
	float Radius;
};

#endif