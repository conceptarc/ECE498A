

#include "Node.h"
#include "Map.h"
#include "A_Star.h"
#define nullptr 0

int main() {

	char end;
	int res = 15;
	Map* test = new Map(res);

	A_Star::FindPath(test);
	//test->Print();
	delete test;

	//test = new Map(res);
	//A_Star::FindPath(test, false);
	//test->Print();

	//delete test;
	cin >> end;
	return 0;
}
