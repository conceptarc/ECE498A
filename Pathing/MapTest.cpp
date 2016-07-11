#include "Node.h"
#include "Map.h"
#include "A_Star.h"
#include "GradientDescent.h"
#include <ctime>
#include "Dijkstra.h"
#include "BestFirst.h"
#include "NonCyclicUniform.h"
#include <fstream>
#include <string>

using namespace std;

enum Algorithm {
	AStar = 0,
	GraDesc = 1,
	Uniform = 2,
	Dijkstra = 3,
	BestFirst = 4
};

string ExecutePathTest(Algorithm type, int resolution, MapObstacleSet obstacleSet) {
	string csvOutput = "";
	TestResult result;
	Map* testMap = nullptr;

	clock_t startTime = clock();
	switch (type) {
	case AStar:
		testMap = new Map(resolution, Occupation, obstacleSet);
		result = A_Star::FindPath(testMap);
		delete testMap;
		break;
	case GraDesc:
		testMap = new Map(resolution, Gradient, obstacleSet);
		result = GradientDescent::FindPath(testMap);
		delete testMap;
		break;
	case Uniform:
		testMap = new Map(resolution, Occupation, obstacleSet);
		result = Uniform::FindPath(testMap);
		delete testMap;
		break;
	case Dijkstra:
		testMap = new Map(resolution, Occupation, obstacleSet);
		result = Dijkstra::FindPath(testMap);
		delete testMap;
		break;
	case BestFirst:
		testMap = new Map(resolution, Occupation, obstacleSet);
		result = BestFirst::FindPath(testMap);
		delete testMap;
		break;
	default:
		return "error - invalid Path Algorithm Type";
	}

	switch (obstacleSet) {
	case Clear:
		result.obstacleDensity = "Clear";
		break;
	case Sparse:
		result.obstacleDensity = "Sparse";
		break;
	case Dense:
		result.obstacleDensity = "Dense";
		break;
	case Wall:
		result.obstacleDensity = "Wall";
		break;
	default:
		result.obstacleDensity = "NoSolution";
		break;
	}

	double totalTime = (clock() - startTime) / (double)CLOCKS_PER_SEC;
	double mapGenTime = totalTime - result.solutionTime;

	csvOutput = result.algorithmName + "," + result.obstacleDensity +"," + (result.hasSolution ? "true" : "false") + "," +
		to_string(result.nodesTotal) + "," + to_string(result.nodesVisited) + "," + to_string(result.percentVisited) + "," +
		to_string(result.solutionDistance) + "," + to_string(result.solutionTime) + "," + to_string(result.widthResolution) +
		"," + to_string(mapGenTime);

	cout << resolution << endl;

	return csvOutput;
}

void debug() {
	Map* temp = new Map(10, Occupation, Clear);
	temp->Print();
	delete temp;
	cout << endl;
	temp = new Map(10, Occupation, Sparse);
	temp->Print();
	delete temp;
	cout << endl;
	temp = new Map(10, Occupation, Dense);
	temp->Print();
	delete temp;
	cout << endl;
	temp = new Map(10, Occupation, Wall);
	temp->Print();
	delete temp;
}

int main() {
	char pause; // for final key input
	string csvHeader = "Name,ObstacleDensity,FoundSolution,TotalNodes,NodesVisited,PercentVisited,SolutionDistance,SolutionTime,MapWidthResolution,MapGenTime";
	string csvBody = "";
	debug();
	cin >> pause;
	//return 0;

	ofstream writer;

	writer.open("FYDP Pathfinding Simulation Results.csv");
	writer << csvHeader << endl;

	int testLimit = 16; // 5 < testLimit < 18, 1.5^17 = roughly 1000

	//A Star
	for (int i = 5; i <= testLimit; i++) { // no obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(AStar, res, Clear);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // sparse obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(AStar, res, Sparse);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // dense obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(AStar, res, Dense);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // wall of obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(AStar, res, Wall);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // no solution
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(AStar, res, NoSolution);
		writer << output << endl;
	}
	cout << "Done A* testing." << endl;

	//Gradient Descent
	for (int i = 5; i <= testLimit; i++) { // no obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(GraDesc, res, Clear);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // sparse obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(GraDesc, res, Sparse);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // dense obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(GraDesc, res, Dense);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // wall of obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(GraDesc, res, Wall);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // no solution
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(GraDesc, res, NoSolution);
		writer << output << endl;
	}
	cout << "Done Gradient Descent testing." << endl;

	//Dijkstra's
	for (int i = 5; i <= testLimit; i++) { // no obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(Dijkstra, res, Clear);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // sparse obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(Dijkstra, res, Sparse);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // dense obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(Dijkstra, res, Dense);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // wall of obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(Dijkstra, res, Wall);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // no solution
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(Dijkstra, res, NoSolution);
		writer << output << endl;
	}
	cout << "Done Dijkstra's testing." << endl;

	//Best First
	for (int i = 5; i <= testLimit; i++) { // no obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(BestFirst, res, Clear);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // sparse obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(BestFirst, res, Sparse);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // dense obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(BestFirst, res, Dense);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // wall of obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(BestFirst, res, Wall);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // no solution
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(BestFirst, res, NoSolution);
		writer << output << endl;
	}
	cout << "Done Best First testing." << endl;

	//Uniform Cost
	for (int i = 5; i <= testLimit; i++) { // no obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(Uniform, res, Clear);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // sparse obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(Uniform, res, Sparse);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // dense obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(Uniform, res, Dense);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // wall of obstacles
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(Uniform, res, Wall);
		writer << output << endl;
	}
	for (int i = 5; i <= testLimit; i++) { // no solution
		int res = (int)powl(1.5, i);
		string output = ExecutePathTest(Uniform, res, NoSolution);
		writer << output << endl;
	}
	cout << "Done Uniform Cost testing." << endl;

	writer.close();

	//cout << csvHeader << endl << csvBody << endl;
	cout << endl << "Input char to exit." << endl;
	cin >> pause;
	return 0;
}