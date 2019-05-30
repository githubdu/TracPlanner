



#pragma once

#include "Test.h"

int main(int argc, char* argv[])
{	
	test_pinv();

	testIntp();

	testTrac();

	testPlanner6D();

	testPlannerAgv();

	testPlannerOpt();

	printf("Press enter to quit \r\n");

	getchar();

	return 0;
}