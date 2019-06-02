



#pragma once

#include "Test.h"

int main(int argc, char* argv[])
{	

	test_linprog();

	test_inv(10000);

	test_pinv(10000);

	test_ikine(10000);


	testIntp();

	testTrac();

	testPlanner6D();

	testPlannerAgv();

	testPlannerOpt();

	printf("Press enter to quit \r\n");

	getchar();

	return 0;
}