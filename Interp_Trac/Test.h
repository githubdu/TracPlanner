

#ifndef TEST_H_
#define TEST_H_

#pragma once
#include <windows.h> // for timing
#include "trajectory.h"






void test_pinv();








void testIntp();
void test_interp_U();
void test_interp_T();
void test_interp_S();
void test_interp_M();



void testTrac();
void test_trac_line();
void test_trac_circle();
void test_trac_bspline();
void test_trac_blender();



#include "Planner.h"
void testPlanner6D();
void test_planner6D_ptp();
void test_planner6D_line();
void test_planner6D_circle();
void test_planner6D_bspline();
void test_planner6D_rotation();


void testPlannerAgv();
void test_plannerAgv_line();
void test_plannerAgv_circle();
void test_plannerAgv_bspline();
void test_plannerAgv_turning();
void test_plannerAgv_rotation();








void testPlannerOpt();

#endif // !TEST_H_