#ifndef TESTRESULT_H
#define TESTRESULT_H
#include <string>

using namespace std;

// just a container for the test results
class TestResult {
public:
	TestResult();
	~TestResult();
	string algorithmName;
	int widthResolution;
	string obstacleDensity;
	int nodesVisited;
	int nodesTotal;
	bool hasSolution;
	double solutionTime;
	double solutionDistance;
	double percentVisited;
};

TestResult::TestResult() {}

TestResult::~TestResult() {}

#endif