#ifndef NODE_H
#define NODE_H

#include <vector>
#include <unordered_set>
#define nullptr 0

using namespace std;

class Node {
private:
	int _id;
	bool _isPath;
	bool _isVisited;
	bool _isOccupied;
	bool _isStart;
	bool _isGoal;
	float _heuristicDist;
	Node* _north;
	Node* _east;
	Node* _south;
	Node* _west;
	Node* _northEast;
	Node* _northWest;
	Node* _southEast;
	Node* _southWest;

public:
	Node();
	~Node();
	// accessors
	int GetID();
	bool IsPath();
	bool IsVisited();
	bool IsOccupied();
	bool IsStart();
	bool IsGoal();
	bool IsObjectPresent;
	bool IsOccupationPredicted;
	float X;
	float Y;
	float GetHeuristicDist();
	Node* GetNorth();
	Node* GetEast();
	Node* GetSouth();
	Node* GetWest();
	Node* GetNorthEast();
	Node* GetNorthWest();
	Node* GetSouthEast();
	Node* GetSouthWest();
	vector<Node*> GetAllAdjacent();
	unordered_set<Node*> GetDiagonals();
	char Print();
	
	// mutators
	void SetID(int id);
	void SetPath(bool isPath);
	void SetVisited(bool isVisited);
	void SetOccupied(bool isOccupied);
	void SetStart(bool isStart);
	void SetGoal(bool isGoal);
	void SetHeuristic(float dist);
	void SetNorth(Node* north);
	void SetEast(Node* east);
	void SetSouth(Node* south);
	void SetWest(Node* west);
	void SetNorthEast(Node* northEast);
	void SetNorthWest(Node* northWest);
	void SetSouthEast(Node* southEast);
	void SetSouthWest(Node* southWest);
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

bool Node::IsVisited() {
	return _isVisited;
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
	return _heuristicDist;
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

Node* Node::GetNorthEast() {
	return _northEast;
}

Node* Node::GetNorthWest() {
	return _northWest;
}

Node* Node::GetSouthEast() {
	return _southEast;
}

Node* Node::GetSouthWest() {
	return _southWest;
}

vector<Node*> Node::GetAllAdjacent() {
	vector<Node*> children;
	if (_north != nullptr) children.push_back(_north);
	if (_east != nullptr) children.push_back(_east);
	if (_south != nullptr) children.push_back(_south);
	if (_west != nullptr) children.push_back(_west);
	if (_northEast != nullptr) children.push_back(_northEast);
	if (_northWest != nullptr) children.push_back(_northWest);
	if (_southEast != nullptr) children.push_back(_southEast);
	if (_southWest != nullptr) children.push_back(_southWest);
	return children;
}

unordered_set<Node*> Node::GetDiagonals() {
	unordered_set<Node*> children;
	children.insert(_northEast);
	children.insert(_northWest);
	children.insert(_southEast);
	children.insert(_southWest);
	return children;
}

char Node::Print() {
	if (_isGoal) return 'G';
	if (_isStart) return 'S';
	if (_isOccupied) return 'X';
	if (IsOccupationPredicted) return 'X';
	if (_isPath) return '.';
	if (_isVisited) return 'o';
	if (IsObjectPresent) return 'o';
	return ' ';
}

void Node::SetID(int id) {
	_id = id;
}

void Node::SetPath(bool isPath) {
	_isPath = isPath;
}

void Node::SetVisited(bool isVisited) {
	_isVisited = isVisited;
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

void Node::SetNorthEast(Node* northEast) {
	_northEast = northEast;
}

void Node::SetNorthWest(Node* northWest) {
	_northWest = northWest;
}

void Node::SetSouthEast(Node* southEast) {
	_southEast = southEast;
}

void Node::SetSouthWest(Node* southWest) {
	_southWest = southWest;
}

#endif