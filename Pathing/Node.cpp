#pragma once
#include "Node.h"

Node::Node() {
	_id = 0;
	_isPath = false;
	_isVisited = false;
	_isOccupied = false;
	_isStart = false;
	_isGoal = false;
	_heuristicDist = 0;
	_north = NULL;
	_east = NULL;
	_south = NULL;
	_west = NULL;
	_northEast = NULL;
	_northWest = NULL;
	_southEast = NULL;
	_southWest = NULL;
	
	IsObjectPresent = false;
	IsOccupationPredicted = false;
	X = 0;
	Y = 0;
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
	if (_isStart) return 'S';
	if (_isGoal) return 'G';
	if (IsOccupationPredicted) return '-';
	if (IsObjectPresent) return 'o';
	if (_isOccupied) return 'X';
	if (_isVisited) return 'o';
	if (_isPath) return '.';
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
