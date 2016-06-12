#ifndef MAP_H
#define MAP_H

#include <math.h>
#include <deque>
#include <iostream>
#include "Node.h"

#define nullptr 0

using namespace std;

class Map {

private:
	float MAP_SIZE;
	float START_X;
	float START_Y;
	float GOAL_X;
	float GOAL_Y;

	int _resolution;
	Node*** map2d;
	Node* _start;
	Node* _goal;
	Node* _topLeft;
	
	float CalcDist(float x1, float x2, float y1, float y2, bool useManhattan);

public:
	Map(int resolution);
	~Map();
	int GetResolution();
	Node* GetStart();
	Node* GetGoal();
	void Print();
};

Map::Map(int resolution) : _resolution(resolution) {
	// some arbitrary constants / constraints
	if (resolution < 10) {
		_resolution = 10;
	} else if (resolution > 2000) {
		_resolution = 2000;
	}
	MAP_SIZE = 100.0f;
	START_X = 15.0f;
	START_Y = 15.0f;
	GOAL_X = 85.0f;
	GOAL_Y = 85.0f;

	// generate a fixed map with variable resolution

	// create 2D array of pointers
	map2d = new Node**[resolution];
	for (int i = 0; i < _resolution; i++) {
		map2d[i] = new Node*[_resolution];
	}

	// initialize
	for (int i = 0; i < _resolution; i++) {
		for (int j = 0; j < _resolution; j++) {
			map2d[i][j] = new Node();
		}
	}

	// now assign properties and references for each one
	float nodeWidth = MAP_SIZE / _resolution;
	float startProximity = nodeWidth * 2; // used for finding the start node
	float goalProximity = nodeWidth * 2; // used for finding the goal node

	for (int i = 0; i < _resolution; i++) {
		for (int j = 0; j < _resolution; j++) {
			Node* node = map2d[i][j];
			node->SetID(i*_resolution + j); // ascending integer for ID

			// set 4-directional links
			node->SetNorth(i == 0 ? NULL : map2d[i - 1][j]);
			node->SetSouth(i == _resolution - 1 ? NULL : map2d[i + 1][j]);
			node->SetWest(j == 0 ? NULL : map2d[i][j - 1]);
			node->SetEast(j == _resolution - 1 ? NULL : map2d[i][j + 1]);
			
			float x = nodeWidth * j + nodeWidth / 2;
			float y = nodeWidth * (_resolution - i - 1) + nodeWidth / 2;
			float distToStart = CalcDist(x, START_X, y, START_Y, false);
			float distToGoal = CalcDist(x, GOAL_X, y, GOAL_Y, false);
			node->SetHeuristic(distToGoal); // calculated above, might as well use it

			// find the start node
			if (distToStart < startProximity) {
				startProximity = distToStart;
				if (_start != nullptr) _start->SetStart(false);
				_start = node;
				_start->SetStart(true);
			}

			// find the goal node
			if (distToGoal < goalProximity) {
				goalProximity = distToGoal;
				if (_goal != nullptr) _goal->SetGoal(false);
				_goal = node;
				_goal->SetGoal(true);
			}

			// circular obstacle at (x = 30, y = 30, r = 10)
			if (powf(x - 30, 2) + powf(y - 30, 2) < 100) node->SetOccupied(true);

			// circular obstacle at (x = 55, y = 35, r = 10)
			if (powf(x - 55, 2) + powf(y - 35, 2) < 100) node->SetOccupied(true);

			// circular obstacle at (x = 40, y = 60, r = 10)
			if (powf(x - 40, 2) + powf(y - 60, 2) < 100) node->SetOccupied(true);

			// circular obstacle at (x = 83, y = 50, r = 10)
			if (powf(x - 83, 2) + powf(y - 50, 2) < 100) node->SetOccupied(true);

			// circular obstacle at (x = 75, y = 65, r = 10)
			if (powf(x - 75, 2) + powf(y - 65, 2) <= 100) node->SetOccupied(true);

			// circular obstacle at (x = 65, y = 82, r = 11)
			if (powf(x - 65, 2) + powf(y - 82, 2) <= 121) node->SetOccupied(true);
		}
	}
	_topLeft = map2d[0][0];

}

Map::~Map() {
	for (int i = 0; i < _resolution; i++) {
		for (int j = 0; j < _resolution; j++) {
			delete map2d[i][j];
		}
		delete[] map2d[i];
	}
	delete[] map2d;
}

int Map::GetResolution() {
	return _resolution;
}

Node* Map::GetStart() {
	return _start;
}

Node* Map::GetGoal() {
	return _goal;
}

void Map::Print() {

	//debug print nodes
	/*
	for (int i = -1; i <= _resolution; i++) {
		for (int j = -1; j <= _resolution; j++) {
			if (i == -1 || i == _resolution)
				cout << '-';
			else if (j == -1 || j == _resolution)
				cout << '|'; 
			else
				cout << map2d[i][j]->Print();
		}
		cout << endl;
	}

	cout << endl;
	*/
	Node* column = _topLeft;
	Node* row = _topLeft;
	
	for (int i = 0; i < _resolution + 2; i++) {
		cout << '-';
	} // top boundary

	cout << endl;
	do {
		cout << '|' << column->Print();
		row = column->GetEast();
		while (row != nullptr) {
			cout << row->Print();
			row = row->GetEast();
		}
		cout << '|' << endl;
		column = column->GetSouth();
	} while (column != nullptr);

	for (int i = 0; i < _resolution + 2; i++) { // bottom boundary
		cout << '-';
	}
}

float Map::CalcDist(float x1, float x2, float y1, float y2, bool useManhattan) {
	if (useManhattan)
		return sqrtf(powf(x1 - x2, 2) + powf(y1 - y2, 2));
	return fabs(x1 - x2) + fabs(y1 - y2);
}
#endif
