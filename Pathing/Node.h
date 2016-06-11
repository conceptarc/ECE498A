#ifndef NODE_H
#define NODE_H

#include <vector>
#define nullptr 0

using namespace std;

class Node {
private:
	int _id;
	bool _isPath;
	bool _isOccupied;
	bool _isStart;
	bool _isGoal;
	float _heuristicDist;
	Node* _north;
	Node* _east;
	Node* _south;
	Node* _west;

public:
	Node();
	~Node();
	// accessors
	int GetID();
	bool IsPath();
	bool IsOccupied();
	bool IsStart();
	bool IsGoal();
	float GetHeuristicDist();
	Node* GetNorth();
	Node* GetEast();
	Node* GetSouth();
	Node* GetWest();
	vector<Node*> GetAllAdjacent();
	char Print();
	
	// mutators
	void SetID(int id);
	void SetPath(bool isPath);
	void SetOccupied(bool isOccupied);
	void SetStart(bool isStart);
	void SetGoal(bool isGoal);
	void SetHeuristic(float dist);
	void SetNorth(Node* north);
	void SetEast(Node* east);
	void SetSouth(Node* south);
	void SetWest(Node* west);
};

Node::Node() {
}

Node::~Node() {
}

int Node::GetID() {
	return _id;
}

bool Node::IsPath() {
	return _isPath;
}

bool Node::IsOccupied() {
	return _isOccupied;
}

bool Node::IsStart() {
	return _isStart;
}

bool Node::IsGoal() {
	return _isGoal;
}

float Node::GetHeuristicDist() {
	// ensure admissibility using floor
	return floorf(_heuristicDist);
}

Node* Node::GetNorth() {
	return _north;
}

Node* Node::GetEast() {
	return _east;
}

Node* Node::GetSouth() {
	return _south;
}

Node* Node::GetWest() {
	return _west;
}

vector<Node*> Node::GetAllAdjacent() {
	vector<Node*> children;
	children.push_back(_north);
	children.push_back(_east);
	children.push_back(_south);
	children.push_back(_west);
	return children;
}

char Node::Print() {
	if (_isOccupied) return 'X';
	if (_isGoal) return 'G';
	if (_isStart) return 'S';
	if (_isPath) return '.';
	return ' ';
}

void Node::SetID(int id) {
	_id = id;
}

void Node::SetPath(bool isPath) {
	_isPath = isPath;
}

void Node::SetOccupied(bool isOccupied) {
	_isOccupied = isOccupied;
}

void Node::SetStart(bool isStart) {
	_isStart = isStart;
}

void Node::SetGoal(bool isGoal) {
	_isGoal = isGoal;
}

void Node::SetHeuristic(float dist) {
	_heuristicDist = dist;
}

void Node::SetNorth(Node* north) {
	_north = north;
}

void Node::SetEast(Node* east) {
	_east = east;
}

void Node::SetSouth(Node* south) {
	_south = south;
}

void Node::SetWest(Node* west) {
	_west = west;
}

#endif