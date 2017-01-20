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

#endif