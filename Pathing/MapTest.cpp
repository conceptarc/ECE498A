

#include "Node.h"
#include "Map.h"
#include "A_Star.h"
#define nullptr 0

int main() {

	char end;

	Map* test = new Map(40);
	//test->Print();
	//cin >> end;
	A_Star::FindPath(test);
	delete test;

	cin >> end;
	return 0;
}