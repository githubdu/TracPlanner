
#pragma once
#include "Test.h"


void test_pinv()
{
	double timing[100000];
	for (int k=0; k<100000; k++)
	{
		//printf("k:%d\n\n",k);

		double data[6*6];
		double pinvData[6*6];

		for (int i=0; i<6; i++)
		{
			for (int j=0; j<6; j++)
			{
				data[i*6+j] = rand() / double(RAND_MAX)*10000.0 - 5000.0;
			}
		}

		double J[6][6],pinvJ[6][6],ret[6][6];

		for (int i=0; i<6; i++)
		{
			for (int j=0; j<6; j++)
			{
				J[i][j] = data[i*6+j];
				//printf("%.7f, ",data[i*6+j]);
			}
			//printf("\n");
		}
		//printf("\n\n");

		//-----------------------------------------------
		LARGE_INTEGER nFreq;					
		LARGE_INTEGER nBeginTime,nEndTime;
		QueryPerformanceFrequency(&nFreq);
		QueryPerformanceCounter(&(nBeginTime));

		pinv(pinvData,data,6,6);

		QueryPerformanceCounter(&(nEndTime));
		timing[k] = 1000.0*(nEndTime.QuadPart - nBeginTime.QuadPart)/(double)(nFreq.QuadPart);
		printf("the pinv time is: %.10f\n",timing[k]);
		//-----------------------------------------------

		for (int i=0; i<6; i++)
		{
			for (int j=0; j<6; j++)
			{
				pinvJ[i][j] = pinvData[i*6+j];
				//printf("%.7f, ",pinvData[i*6+j]);
			}
			//printf("\n");
		}
		//printf("\n\n");


		// check result
		bool flag = false;
		M6p6(ret,J,pinvJ);
		for (int i=0; i<6; i++)
		{
			for (int j=0; j<6; j++)
			{
				if (i==j)
				{
					if (ABS(ret[i][j] - 1.0) > ACCURACY_FACTOR)
					{
						flag = true;
					}
				}
				else
				{
					if (ABS(ret[i][j]) > ACCURACY_FACTOR)
					{
						flag = true;
					}
				}
				//printf("%.6f, ",ret[i][j]);
			}
			//printf("\n");
		}
		//printf("\n\n");

		if (flag)
		{
			for (int i=0; i<6; i++)
			{
				for (int j=0; j<6; j++)
				{
					data[i*6+j] = J[i][j];
				}
			}
			printf("ERR:%d\n",k);
			pinv(pinvData,data,6,6);
		}
	}
}





void testIntp()
{
	test_interp_U();
	test_interp_T();
	test_interp_S();
	test_interp_M();
}

void test_interp_U()
{
	printf("testing 'U' profile interpolation ... \n");

	DS_or_Trap U;
	double t=0,tT;
	U.setLimit(1.8,3.0,10.0);
	U.planProfile(t,0,1,0,0);

	if (U.isValid())
	{
		FILE* file = nullptr;
		fopen_s(&file, "../data/U.txt","w+");

		tT = U.getDuration() + t;

		while(t <= tT && file != nullptr)
		{
			fprintf(file,"%.10f,%.10f,%.10f,%.10f,%.10f\n",
				t,U.pos(t),U.vel(t),U.acc(t),U.jerk(t));

			t += PLAN_CYCLE/1000.0;
		}
		
		if (file!=nullptr)
		{
			fclose(file);
		}
	}
	printf("Done! The result is in 'U.txt' .\n\n\n");
}

void test_interp_T()
{
	printf("testing 'T' profile interpolation ... \n");

	double t=0,tT;
	Trapezoid Trape;
	Trape.setLimit(3,1.8,10);
	Trape.planProfile(t,0,-6,-1,-2);

	if (Trape.isValid())
	{
		FILE* file = nullptr;
		fopen_s(&file, "../data/T.txt","w+");

		tT = Trape.getDuration() + t;

		while(t <= tT && file != nullptr)
		{
			fprintf(file,"%.10f,%.10f,%.10f,%.10f,%.10f\n",
				t,Trape.pos(t),Trape.vel(t),Trape.acc(t),Trape.jerk(t));

			t += PLAN_CYCLE/1000.0;
		}

		if (file!=nullptr)
		{
			fclose(file);
		}
	}
	printf("Done! The result is in 'T.txt' .\n\n\n");

}

void test_interp_S()
{
	printf("testing 'S' profile interpolation ... \n");

	DoubleS Ds;
	double t=0,tT;
	Ds.setLimit(1.8,3,10);
	Ds.planProfile(t,0,-5,2);

	if (Ds.isValid())
	{
		FILE* file = nullptr;
		fopen_s(&file, "../data/S.txt","w+");

		tT = Ds.getDuration() + t;

		while(t <= tT && file != nullptr)
		{
			fprintf(file,"%.10f,%.10f,%.10f,%.10f,%.10f\n",
				t,Ds.pos(t),Ds.vel(t),Ds.acc(t),Ds.jerk(t));

			t += PLAN_CYCLE/1000.0;
		}

		if (file!=nullptr)
		{
			fclose(file);
		}
	}
	printf("Done! The result is in 'S.txt' .\n\n\n");

}

void test_interp_M()
{
	printf("testing multi-points profile interpolation ... \n");

	double t = 0, tT = 10;

	double p[11] = {0,1,-1.3,1,3,-6,2,-0.3,-2,-1,0};

	Multi_ST_1D Mst;
	Mst.setLimit(2.0,5.0,15.0);
	Mst.planProfile(t,tT,11,p);

	if (Mst.isValid())
	{
		double T[11] = {0.0};
		Mst.getTimeSequence(T,11);

		FILE* file = nullptr;
		fopen_s(&file, "../data/M.txt","w+");

		tT = Mst.getDuration() + t;

		while(t <= tT && file != nullptr)
		{
			fprintf(file,"%.10f,%.10f,%.10f,%.10f,%.10f\n",
				t,Mst.pos(t),Mst.vel(t),Mst.acc(t),Mst.jerk(t));

			t += PLAN_CYCLE/1000.0;
		}

		if (file!=nullptr)
		{
			fclose(file);
		}
	}
	printf("Done! The result is in 'M.txt'. \n\n\n");
}




void testTrac()
{
	test_trac_line();
	test_trac_circle();
	test_trac_bspline();
	test_trac_blender();
}

void test_trac_line()
{
	printf("testing 3D line trajectory...\n");


	double t=0.5,tT;
	double p[TRANS_D];
	double v[TRANS_D];
	double a[TRANS_D];
	double p0[3], p1[3];
	p0[0] = 0; p0[1] = 0; p0[2] = 0;
	p1[0] = 1; p1[1] = -3; p1[2] = 2;

	LINE trac;
	trac.setLimit(2,5,10);
	trac.planTrajectory(t,p0,p1);

	if (trac.isValid())
	{
		FILE* file;
		fopen_s(&file, "../data/line.txt","w+");

		tT = t + trac.getDuration();

		double trigger = tT/2.0; // stop or re-plan

		while(t<=tT && file!=nullptr)
		{
			trac.move(t,p,v,a);
			fprintf(file,"%.10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f\n",
				t,p[0],p[1],p[2],v[0],v[1],v[2],a[0],a[1],a[2]);

			t += PLAN_CYCLE/1000.0;

			if (ABS(t-trigger) < PLAN_CYCLE/1000.0)
			{
				trac.stopTrajectory(trigger);
				//trac.rePlanTrajectory(t,1);
				tT = trac.getDuration() + trac.getStartTime();
			}
		}

		if (file!=nullptr)
		{
			fclose(file);
		}
	}

	printf("Done! The result is in 'line.txt'\n\n\n");
}

void test_trac_circle()
{
	printf("testing 3D circle trajectory ... \n");

	double refV = 1;
	double t=1.0,tT;

	double p[TRANS_D];
	double v[TRANS_D];
	double a[TRANS_D];
	double p0[3], p1[3], p2[3];
	p0[0] = 0; p0[1] = 0; p0[2] = 0;
	p1[0] = 1; p1[1] = 3; p1[2] = 2;
	p2[0] = 2; p2[1] = 1; p2[2] = 0;

	CIRCLE trac;
	trac.setLimit(2,5,10);
	trac.planTrajectory(t,p0,p1,p2);	
	
	// or :
	//double cA = circleAngle(p0,p1,p2);
	//double cP[3]; circleCenter(cP,p0,p1,p2);
	//double nV[3]; circleVector(nV,p0,p1,p2);
	//trac.planTrajectory(t,p0,cP,nV,cA);

	if (trac.isValid())
	{
		FILE* file;
		fopen_s(&file, "../data/circle.txt","w+");

		tT = t + trac.getDuration();

		double trigger = tT/2.0; // stop or re-plan

		while(t<=tT && file!=nullptr)
		{
			trac.move(t,p,v,a);

			fprintf(file,"%.10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f\n",
				t,p[0],p[1],p[2],v[0],v[1],v[2],a[0],a[1],a[2]);

			t += PLAN_CYCLE/1000.0;

			if (ABS(t-trigger) < PLAN_CYCLE/1000.0)
			{
				//trac.stopTrajectory(t);

				if(0 == trac.rePlanTrajectory(t,refV))
				{
					refV = 2.0;
					trigger = t + (tT - t)/2.0;
				}

				tT = trac.getDuration() + trac.getStartTime();
			}
		}

		if (file!=nullptr)
		{
			fclose(file);
		}
	}

	printf("Done! The result is in 'circle.txt'\n\n\n");
}

void test_trac_bspline()
{
	printf("test 3D B-spline trajectory ... \n");

	double points[100][3];
	points[0][0] = 0.83;	points[0][1] = -0.54;	points[0][2] = 1.19;
	points[1][0] = 0.64;	points[1][1] = 0.10;	points[1][2] = -1.24;
	points[2][0] = 0.42;	points[2][1] = 0.79;	points[2][2] = 2.26;
	points[3][0] = -0.98;	points[3][1] = 0.23;	points[3][2] = 2.22;
	points[4][0] = -0.13;	points[4][1] = 1.25;	points[4][2] = 1.02;
	points[5][0] = 1.40;	points[5][1] = 0.81;	points[5][2] = 0.92;
	points[6][0] = 0.43;	points[6][1] = 0.32;	points[6][2] = 0.92;
	points[7][0] = -0.65;	points[7][1] = -0.17;	points[7][2] = 1.34;
	points[8][0] = -0.45;	points[8][1] = -0.89;	points[8][2] = 1.82;
	points[9][0] = 0.71;	points[9][1] = 0.90;	points[9][2] = 1.92;

	// repeat 10 times
	for (int i=10; i<100; i++)
	{
		for (int j=0; j<3; j++)
		{
			points[i][j] = points[i-10][j];
		}
	}

	BSPLINE trac;
	trac.setLimit(5,10,20);
	double t0 = 0.0, tf = 100.0;
	{
		LARGE_INTEGER nFreq;					
		LARGE_INTEGER nBeginTime,nEndTime;
		QueryPerformanceFrequency(&nFreq);
		QueryPerformanceCounter(&(nBeginTime));

		trac.planTrajectory(t0,tf,points,100);

		QueryPerformanceCounter(&(nEndTime));
		double temp = 1000.0*(nEndTime.QuadPart - nBeginTime.QuadPart)/(double)(nFreq.QuadPart);
		printf("the b-spline planning time is: %.5f\n",temp);

		QueryPerformanceCounter(&(nBeginTime));
		double arcLength = trac.getProfileLength();
		QueryPerformanceCounter(&(nEndTime));
		temp = 1000.0*(nEndTime.QuadPart - nBeginTime.QuadPart)/(double)(nFreq.QuadPart);
		printf("the getting arc length(%.3f) time is: %.5fms\n",arcLength,temp);
	}

	if (trac.isValid())
	{
		FILE* file;
		fopen_s(&file, "../data/bspline.txt","w+");

		double p[TRANS_D],v[TRANS_D],a[TRANS_D];
		double t=0,tT;tT = t + trac.getDuration();

		srand(10);
		double trigger = 10.0*(rand() + 0.0)/RAND_MAX *tT + t;

		while(t<=tT && file!=nullptr)
		{
			trac.move(t,p,v,a);
			fprintf(file,"%.10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f,%10f\n",
				t,p[0],p[1],p[2],v[0],v[1],v[2],a[0],a[1],a[2]);

			t += PLAN_CYCLE/1000.0;

			if (ABS(t-trigger) < PLAN_CYCLE/1000.0)
			{
				trac.stopTrajectory(t);
				tT = trac.getDuration() + trac.getStartTime();
			}
		}

		if (file!=nullptr)
		{
			fclose(file);
		}
	}

	printf("Done! the result is in 'bspline.txt'\n\n\n");
}

void test_trac_blender()
{
	printf("testing 3D blender trajectory...\n");

	// limits
	double vl=0.5,al=1.0,jl=2.0;

	// points
	double p0[TRANS_D] = {0};
	double p1[TRANS_D] = {0.5,0.5,0};
	double p2[TRANS_D] = {-1.2,2.6,0};
	double p3[TRANS_D] = {-1,5,0};

	// circle
	CIRCLE trac1;
	trac1.setLimit(vl,al,jl);
	double cA = circleAngle(p0,p1,p2);
	double nV[3]; circleVector(nV,p0,p1,p2);
	double cP[3]; circleCenter(cP,p0,p1,p2);
	trac1.planTrajectory(0,p0,cP,nV,cA);
	if (!trac1.isValid())
	{
		printf("wrong circle trajectory!\n");
		return;
	}

	// line
	LINE line;
	line.setLimit(vl,al,jl);
	line.planTrajectory(0,p2,p3);
	if (!line.isValid())
	{
		printf("wrong line trajectory!\n");
		return;
	}

	// transition zone

	BLENDER blender;
	blender.setLimit(vl,al,jl);
	blender.setPrevTrac(&trac1);
	blender.setPostTrac(&line);
	blender.setBlendZone(0,0.3);

	// timing
	{
		LARGE_INTEGER nFreq;
		LARGE_INTEGER nBeginTime,nEndTime;
		QueryPerformanceFrequency(&nFreq);
		QueryPerformanceCounter(&(nBeginTime));

		blender.planTrajectory(); // timing planTracjectory

		QueryPerformanceCounter(&(nEndTime));
		double temp = 1000.0*(nEndTime.QuadPart - nBeginTime.QuadPart)/(double)(nFreq.QuadPart);
		printf("the blender planning time is: %.5f ms\n",temp);

		QueryPerformanceCounter(&(nBeginTime));
		double arcLength = blender.getProfileLength(); // timing arcLength
		QueryPerformanceCounter(&(nEndTime));
		temp = 1000.0*(nEndTime.QuadPart - nBeginTime.QuadPart)/(double)(nFreq.QuadPart);
		printf("the getting arc length(%.3f) time is: %.5fms\n",arcLength,temp);
	}

	if (!blender.isValid())
	{
		printf("wrong blender trajectory!\n");
		return ;
	}

	FILE* file;
	fopen_s(&file,"../data/blender.txt","w+");

	double t = 0.0;
	double indexT = 0.0;
	double p[TRANS_D] = {0.0};
	double v[TRANS_D] = {0.0};
	double a[TRANS_D] = {0.0};
	double t1 = blender.getPrevTime();
	double t2 = blender.getPostTime();
	double totalTime = t1 + blender.getDuration()
		+ (blender.getPostTrac())->getDuration() - t2;

	TRAC* tracHandl = nullptr;

	srand(10);
	bool   stoped = false;
	double trigger = 80.2;

	while(t <= totalTime)
	{
		if (t <= t1)
		{
			indexT = t;
			tracHandl = blender.getPrevTrac();
		}
		else
		{
			if (t <= blender.getStartTime()+blender.getDuration())
			{
				indexT = t;
				tracHandl = &blender;

				if (ABS(t-trigger) < PLAN_CYCLE/1000.0)
				{
					if (tracHandl->stopTrajectory(t))
					{
						stoped = true;
					}
				}
			}
			else
			{
				if (!stoped)
				{
					indexT = t - t1 - blender.getDuration() + t2;
					tracHandl = blender.getPostTrac();
				}
				else
				{
					break;
				}
			}
		}

		tracHandl->move(indexT,p,v,a);

		if (file!=nullptr)
		{
			fprintf(file,"%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n",
				t,p[0],p[1],p[2],v[0],v[1],v[2],a[0],a[1],a[2]);
		}

		t = t + PLAN_CYCLE/1000.0;
	}

	if (file!=nullptr)
	{
		fclose(file);
	}

	printf("Done! The result is in 'blender.txt'\n\n\n");
}





void testPlanner6D()
{
	test_planner6D_ptp();
	test_planner6D_line();
	test_planner6D_circle();
	test_planner6D_bspline();
	test_planner6D_rotation();
}

void test_planner6D_ptp()
{
	printf("test planner->ptp ...\n");

	// prepare
	FILE* file;
	fopen_s(&file,"../data/pj.txt","w+");

	double t0 = 0.0;
	double refV = 0.5;
	double refA = 1.0;
	double refJ = 1.0;

	int dof = 6;
	int index = 0;
	int tracN = 2;
	double tcp[6] = {0};
	double start[6] = {0};
	double pos[6],vel[6],acc[6];
	double targets[2][6] = {{0,PI/4,-PI/4,0,0,0},
							{0,PI/4,0,0,PI/4,0}};

	// planner
	bool newM = true;
	bool quit = false;
	Planner6D planner;
	planner.updateCurrentJoint(dof,start);
	double jvl[6], jal[6],jjl[6];
	for (int i=0; i<dof; i++)
	{
		jvl[i] = 110.0/180.0*PI;
		jal[i] = 300.0/180.0*PI;
		jjl[i] = 900.0/180.0*PI;
	}
	planner.setJointLimits(dof,jvl,jal,jjl);

	// simulated cyclic real-time task
	while(!quit)
	{
		// if receive new line motion
		if (newM)
		{
			switch(planner.moveJ(targets[index],refV,refA,refJ))
			{
			case PLANNER_SUCCEED:
				index += 1;
				if (index >= tracN)
				{
					newM = false;
				}
				break;
			case ERR_PLAN_FAILED:
				printf("moveJ failed!\n");
				quit = true;
				break;
			}
		}

		// get target
		switch(planner.move(PLAN_CYCLE,pos,vel,acc))
		{
		case PLANNER_DONE:
			if (!newM)
			{
				quit = true;
			}
			break;

		case ERR_PLAN_FAILED:
			printf("moveJ failed!\n");
			quit = true;
			break;
		}

		// save target
		if (file!=nullptr)
		{
			fprintf(file,"%.20f",t0);
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",pos[i]);
			}
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",vel[i]);
			}
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",acc[i]);
			}
			fprintf(file,"\n");
		}

		t0 += PLAN_CYCLE/1000.0;
	}

	// quit the task
	if (file!=nullptr)
	{
		fclose(file);
	}
	printf("Done! the results are saved in 'pj.txt'!\n\n\n");
}

void test_planner6D_line()
{
	printf("test planner->line ...\n");

	// prepare
	FILE* file;
	fopen_s(&file,"../data/pl.txt","w+");

	double t0 = 0.0;
	double refV = 0.5;
	double refA = 1.0;
	double refJ = 1.0;
	double refB = 0.1;

	int index = 0;
	int tracN = 3;
	double start[6] = {0};	
	double pos[6],vel[6],acc[6];
	double tcp[6] = {1,0,2,PI/2,0,0};
	double targets[3][6] = {{0,5,0,0,0,PI},
							{5,10,0,0,0,0},
							{0,-5,0,0,0,-PI}};

	// planner
	bool newM = true;
	bool quit = false;
	Planner6D planner;
	double tvl = 1.5, tal = 5, tjl = 15;
	double rvl = 1.0, ral = 3, rjl = 10;
	planner.updateCurrentPos(start,tcp);
	planner.setCartesianLimits(tvl,tal,tjl,rvl,ral,rjl);

	// simulated cyclic real-time task
	while(!quit)
	{
		if (newM)
		{
			switch(planner.moveL(targets[index],refV,refA,refJ,refB))
			{
			case PLANNER_SUCCEED:
				index += 1;
				if (index >= tracN)
				{
					newM = false;
				}
				break;
			case ERR_PLAN_FAILED:
				printf("moveL failed!\n");
				quit = true;
				break;
			}
		}

		// get target
		switch(planner.move(PLAN_CYCLE,pos,vel,acc))
		{
		case PLANNER_DONE:
			if (!newM)
			{
				quit = true;
			}
			break;

		case ERR_PLAN_FAILED:
			printf("moveL failed!\n");
			quit = true;
			break;
		}

		// save target
		if (file!=nullptr)
		{
			fprintf(file,"%.20f",t0);
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",pos[i]);
			}
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",vel[i]);
			}
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",acc[i]);
			}
			fprintf(file,"\n");
		}

		t0 += PLAN_CYCLE/1000.0;
	}

	// quit the task
	if (file!=nullptr)
	{
		fclose(file);
	}
	printf("Done! the results are saved in 'pl.txt'!\n\n\n");
}

void test_planner6D_circle()
{
	printf("test planner->circle ...\n");

	// prepare
	FILE* file;
	fopen_s(&file,"../data/pc.txt","w+");

	double t0 = 0.0;
	double refV = 0.5;
	double refA = 1.0;
	double refJ = 1.0;
	double refB = 0.2;

	int index = 0;
	int tracN = 2;
	double start[6] = {0};	
	double pos[6],vel[6],acc[6];
	double tcp[6] = {0.5,0,0,0,0,0};
	double targets[2][6] = {{0,10,0,0,0,PI},
							{0,0,0,0,0,0}};
	double middles[2][6] = {{5,5,0,0,0,0},
							{0,5,5,0,0,0}};

	// planner
	bool newM = true;
	bool quit = false;
	Planner6D planner;
	planner.updateCurrentPos(start,tcp);
	double tvl = 1.5, tal = 5, tjl = 15;
	double rvl = 1.0, ral = 3, rjl = 10;
	planner.setCartesianLimits(tvl,tal,tjl,rvl,ral,rjl);

	// simulated cyclic real-time task
	while(!quit)
	{
		if (newM)
		{
			// or:
			//double cA = circleAngle(start,middle,target);
			//double cP[3];circleCenter(cP,start,middle,target);
			//double nV[3];circleVector(nV,start,middle,target);
			//switch(planner.moveC(cP,cA,nV,target+3,refV,refA,refJ,refB))

			switch(planner.moveC(targets[index],middles[index],refV,refA,refJ,refB))
			{
			case PLANNER_SUCCEED:
				index += 1;
				if (index >= tracN)
				{
					newM = false;
				}
				break;
			case ERR_PLAN_FAILED:
				printf("moveC failed!\n");
				quit = true;
				break;
			}
		}

		// get target
		switch(planner.move(PLAN_CYCLE,pos,vel,acc))
		{
		case PLANNER_DONE:
			if (!newM)
			{
				quit = true;
			}
			break;

		case ERR_PLAN_FAILED:
			printf("moveC failed!\n");
			quit = true;
			break;
		}

		// save target
		if (file!=nullptr)
		{
			fprintf(file,"%.20f",t0);
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",pos[i]);
			}
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",vel[i]);
			}
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",acc[i]);
			}
			fprintf(file,"\n");
		}

		t0 += PLAN_CYCLE/1000.0;
	}

	// quit the task
	if (file!=nullptr)
	{
		fclose(file);
	}
	printf("Done! the results are saved in 'pc.txt'!\n\n\n");
}

void test_planner6D_bspline()
{
	printf("test planner->bspline ...\n");

	double p[11][6] = {{0}};
	p[0][0] = 2.0,	p[0][1] = 0.0,	p[0][5] = PI/2;
	p[1][0] = 2.0,  p[1][1] = 2.0,  p[1][5] = 0;
	p[2][0] = 2.0,	p[2][1] = 4.0,	p[2][5] = -PI/2;
	p[3][0] = 0.0,	p[3][1] = 4.0,	p[3][5] = 0.0;
	p[4][0] = 0.0,	p[4][1] = 2.0,	p[4][5] = PI/2;
	p[5][0] = 0.0,	p[5][1] = 0.0,	p[5][5] = PI;
	p[6][0] = 1.0,	p[6][1] = 2.0,	p[6][5] = PI;
	p[7][0] = 2.0,	p[7][1] = 4.0,	p[7][5] = PI;
	p[8][0] = 0.0,	p[8][1] = 4.0,	p[8][5] = PI;
	p[9][0] = 2.0,	p[9][1] = 0.0,	p[9][5] = PI;
	p[10][0] = 0.0,	p[10][1] = 0.0;	p[10][5] = PI;

	// prepare
	FILE* file;
	fopen_s(&file,"../data/ps.txt","w+");

	int index = 0;
	int tracN = 1;
	double t0 = 0.0;
	double refT = 100;
	double refB = 0.05;
	double start[6] = {0};
	double pos[6],vel[6],acc[6];
	double tcp[6] = {1,0,2,PI/2,0,0};
	
	// planner
	bool newM = true;
	bool quit = false;
	Planner6D planner;
	planner.updateCurrentPos(start,tcp);
	double tvl = 1.5, tal = 5, tjl = 15;
	double rvl = 1.0, ral = 3, rjl = 10;
	planner.setCartesianLimits(tvl,tal,tjl,rvl,ral,rjl);

	// simulated cyclic real-time task
	while(!quit)
	{
		if (newM)
		{
			switch(planner.moveB(p,11,0.5,refB,0,0,refT))
			{
			case PLANNER_SUCCEED:
				index += 1;
				if (index >= tracN)
				{
					newM = false;
				}
				break;
			case ERR_PLAN_FAILED:
				printf("moveB failed!\n");
				quit = true;
				break;
			}
		}

		// get target
		switch(planner.move(PLAN_CYCLE,pos,vel,acc))
		{
		case PLANNER_DONE:
			if (!newM)
			{
				quit = true;
			}
			break;

		case ERR_PLAN_FAILED:
			printf("moveB failed!\n");
			quit = true;
			break;
		}

		// save target
		if (file!=nullptr)
		{
			fprintf(file,"%.20f",t0);
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",pos[i]);
			}
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",vel[i]);
			}
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",acc[i]);
			}
			fprintf(file,"\n");
		}

		t0 += PLAN_CYCLE/1000.0;
	}

	// quit the task
	if (file!=nullptr)
	{
		fclose(file);
	}

	printf("Done! the results are saved in 'ps.txt'!\n\n\n");
}

void test_planner6D_rotation()
{
	printf("test planner->rotation ...\n");

	// prepare
	FILE* file;
	fopen_s(&file,"../data/pr.txt","w+");

	double t0 = 0.0;
	double refV = 0.5;
	double refA = 1.0;
	double refJ = 1.0;

	int index = 0;
	int tracN = 2;
	double start[6] = {0};	
	double tcp[6] = {2,0,0,0,0,0};
	double pos[6],vel[6],acc[6];
	double targets[2][6] = {{0,0,0,0,0,PI},
							{0,0,0,0,0,0}};

	// planner
	bool newM = true;
	bool quit = false;
	Planner6D planner;
	planner.updateCurrentPos(start,tcp);
	double tvl = 1.5, tal = 5, tjl = 15;
	double rvl = 1.0, ral = 3, rjl = 10;
	planner.setCartesianLimits(tvl,tal,tjl,rvl,ral,rjl);

	// simulated cyclic real-time task
	while(!quit)
	{
		if (newM)
		{
			switch(planner.moveR(targets[index],refV,refA,refJ))
			{
			case PLANNER_SUCCEED:
				index += 1;
				if (index >= tracN)
				{
					newM = false;
				}
				break;
			case ERR_PLAN_FAILED:
				printf("moveR failed!\n");
				quit = true;
				break;
			}
		}

		// get target
		switch(planner.move(PLAN_CYCLE,pos,vel,acc))
		{
		case PLANNER_DONE:
			if (!newM)
			{
				quit = true;
			}
			break;

		case ERR_PLAN_FAILED:
			printf("moveR failed!\n");
			quit = true;
			break;
		}

		// save target
		if (file!=nullptr)
		{
			fprintf(file,"%.20f",t0);
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",pos[i]);
			}
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",vel[i]);
			}
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",acc[i]);
			}
			fprintf(file,"\n");
		}

		t0 += PLAN_CYCLE/1000.0;
	}

	// quit the task
	if (file!=nullptr)
	{
		fclose(file);
	}

	printf("Done! the results are saved in 'pr.txt'!\n\n\n");
}




#define DOF 6

void testPlannerOpt()
{
	printf("test plannerOpt ...\n");

	// prepare
	FILE* file;
	fopen_s(&file,"../data/opt.txt","w+");

	double start[6] = {0};
	double joint[] = {0,-45.0/180.0*PI,45.0/180.0*PI,0,0,0};

	double t0 = 0.0;
	double refV = 0.3;
	double refA = 1.0;
	double refJ = 1.0;
	double refB = 0.1;

	int index = 0;
	int tracN = 5;
	double pos[6],vel[6],jpose[6],jvel[6];
	double targets[][6] = {{0.2,0,0,0,0,0},
							{0.0,0.2,0,0,0,0},
							{0.0,0.0,0.2,0,0,0},
							{0,-0.2,-0.2,0,0,0},
							{-0.2,0,0,0,0,0}};

	// planner
	Kine kine;
	bool newM = true;
	bool quit = false;
	PlannerOpt planner;
	kine.Fkine(start,joint);
	double tvl=1,tal=2,tjl=5;
	double rvl=1,ral=2,rjl=5;
	planner.updateCurrentPos(start);
	planner.updateCurrentJoint(DOF,joint);
	double jointVelLimit[DOF] = {2,2,2,2,2,2};
	double jointAccLimit[DOF] = {5,5,5,5,5,5};
	double jointUpLimit[DOF] = {PI,PI,PI,PI,PI,PI};
	planner.setCartesianLimits(tvl,tal,tjl,rvl,ral,rjl);
	double jointLowLimit[DOF] = {-PI,-PI,-PI,-PI,-PI,-PI};
	planner.setJointRange(DOF,jointUpLimit,jointLowLimit);
	planner.setJointLimits(DOF,jointVelLimit,jointAccLimit,jointUpLimit);

	// simulated cyclic real-time task
	while(!quit)
	{
		if (newM)
		{
			switch(planner.moveL(targets[index],refV,refA,refJ,refB,0,true))
			{
			case PLANNER_SUCCEED:
				index += 1;
				if (index >= tracN)
				{
					newM = false;
				}
				break;
			case ERR_PLAN_FAILED:
				printf("moveL failed!\n");
				quit = true;
				break;
			}
		}

		// get target
		switch(planner.move(PLAN_CYCLE,pos,vel,jpose,jvel))
		{
		case PLANNER_DONE:
			if (!newM)
			{
				quit = true;
			}
			break;

		case ERR_PLAN_FAILED:
			printf("moveL failed!\n");
			quit = true;
			break;
		}

		// save target
		if (file!=nullptr)
		{
			fprintf(file,"%.20f",t0);
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",pos[i]);
			}
			for (int i=0; i<6; i++)
			{
				fprintf(file,",%.20f",vel[i]);
			}
			for (int i=0; i<DOF; i++)
			{
				fprintf(file,",%.20f",jpose[i]);
			}
			for (int i=0; i<DOF; i++)
			{
				fprintf(file,",%.20f",jvel[i]);
			}
			fprintf(file,"\n");
		}

		t0 += PLAN_CYCLE/1000.0;
	}

	// quit the task
	if (file!=nullptr)
	{
		fclose(file);
	}
	printf("Done! the results are saved in 'opt.txt'!\n\n\n");
}




void testPlannerAgv()
{
	test_plannerAgv_line();
	test_plannerAgv_circle();
	test_plannerAgv_bspline();
	test_plannerAgv_rotation();
}

void test_plannerAgv_line()
{
	printf("test plannerAgv->line ...\n");

	// prepare
	FILE* file;
	fopen_s(&file,"../data/al.txt","w+");

	double t0 = 0.0;
	double refV = 0.5;
	double refA = 1.0;
	double refJ = 1.0;
	double refB = 0.1;

	// x y a w	
	int index = 0;
	int tracN = 4;
	double start[4] = {0};
	double tcp[3] = {0,0,0};
	double pos[4],vel[4],acc[4];
	double targets[4][3] = {{5,0,1.0*PI},
							{5,5,0.0*PI},
							{0,5,-1.0*PI},
							{0,0,0.0*PI}};

	// planner
	bool newM = true;
	bool quit = false;
	PlannerAgv planner;
	double tvl = 1.5, tal = 5, tjl = 15;
	double rvl = 1.0, ral = 3, rjl = 10;
	planner.updateCurrentPos(start,tcp);
	planner.setCartesianLimits(tvl,tal,tjl,rvl,ral,rjl);

	// simulated cyclic real-time task
	while(!quit)
	{
		if (newM)
		{		
			switch(planner.moveL(targets[index],refV,refA,refJ,refB,0,-1))
			{
			case PLANNER_SUCCEED:
				index += 1;
				if (index >= tracN)
				{
					newM = false;
				}
				break;
			case ERR_PLAN_FAILED:
				printf("moveL failed!\n");
				quit = true;
				break;
			}
		}

		// get target
		switch(planner.move(PLAN_CYCLE,pos,vel,acc))
		{
		case PLANNER_DONE:
			if (!newM)
			{
				quit = true;
			}
			break;

		case ERR_PLAN_FAILED:
			printf("moveL failed!\n");
			quit = true;
			break;
		}

		// save target
		if (file!=nullptr)
		{
			fprintf(file,"%.20f",t0);
			for (int i=0; i<4; i++)
			{
				fprintf(file,",%.20f",pos[i]);
			}
			for (int i=0; i<4; i++)
			{
				fprintf(file,",%.20f",vel[i]);
			}
			for (int i=0; i<4; i++)
			{
				fprintf(file,",%.20f",acc[i]);
			}
			fprintf(file,"\n");
		}
		t0 += PLAN_CYCLE/1000.0;
	}

	// quit the task
	if (file!=nullptr)
	{
		fclose(file);
	}
	printf("Done! the results are saved in 'al.txt'!\n\n\n");
}

void test_plannerAgv_circle()
{
	printf("test plannerAgv->circle ...\n");

	// prepare
	FILE* file;
	fopen_s(&file,"../data/ac.txt","w+");

	double t0 = 0.0;
	double refV = 0.5;
	double refA = 1.0;
	double refJ = 1.0;
	double refB = 0.0;

	int index = 0;
	int tracN = 4;
	double start[4] = {0};
	double tcp[6] = {0,0,0};
	double pos[4],vel[4],acc[4];

	double targets[4][3] = {{5,0,1.0*PI},
							{5,5,0.0*PI},
							{0,5,-1.0*PI},
							{0,0,0.0*PI}};
	double middles[4][3] = {{2.5,2.5,0},
							{2.5,2.5,0},
							{2.5,2.5,0},
							{2.5,2.5,0}};

	// planner
	bool newM = true;
	bool quit = false;
	PlannerAgv planner;
	planner.updateCurrentPos(start,tcp);
	double tvl = 1.5, tal = 5, tjl = 15;
	double rvl = 1.0, ral = 3, rjl = 10;
	planner.setCartesianLimits(tvl,tal,tjl,rvl,ral,rjl);

	// simulated cyclic real-time task
	while(!quit)
	{
		if (newM)
		{
			switch(planner.moveC(targets[index],middles[index],refV,refA,refJ,refB,0,0,1))
			{
			case PLANNER_SUCCEED:
				index += 1;
				if (tracN==index)
				{
					newM = false;
				}
				break;
			case ERR_PLAN_FAILED:
				printf("moveC failed!\n");
				quit = true;
				break;
			}
		}

		// get next target
		switch(planner.move(PLAN_CYCLE,pos,vel,acc))
		{
		case PLANNER_DONE:
			if (!newM)
			{
				quit = true;
			}
			break;

		case ERR_PLAN_FAILED:
			quit = true;
			break;
		}

		// save target
		if (file!=nullptr)
		{
			fprintf(file,"%.20f",t0);
			for (int i=0; i<4; i++)
			{
				fprintf(file,",%.20f",pos[i]);
			}
			for (int i=0; i<4; i++)
			{
				fprintf(file,",%.20f",vel[i]);
			}
			for (int i=0; i<4; i++)
			{
				fprintf(file,",%.20f",acc[i]);
			}
			fprintf(file,"\n");
		}
		t0 += PLAN_CYCLE/1000.0;
	}

	// quit the task
	if (file!=nullptr)
	{
		fclose(file);
	}
	printf("Done! the results are saved in 'ac.txt'!\n\n\n");
}

void test_plannerAgv_bspline()
{
	printf("test plannerAgv->bspline ...\n");

	double p[11][3] = {{0}};
	p[0][0] = 2.0,	p[0][1] = 0.0,	p[0][2] = PI/2;
	p[1][0] = 2.0,  p[1][1] = 2.0,  p[1][2] = 0;
	p[2][0] = 2.0,	p[2][1] = 4.0,	p[2][2] = -PI/2;
	p[3][0] = 0.0,	p[3][1] = 4.0,	p[3][2] = 0.0;
	p[4][0] = 0.0,	p[4][1] = 2.0,	p[4][2] = PI/2;
	p[5][0] = 0.0,	p[5][1] = 0.0,	p[5][2] = PI;
	p[6][0] = 1.0,	p[6][1] = 2.0,	p[6][2] = PI;
	p[7][0] = 2.0,	p[7][1] = 4.0,	p[7][2] = PI;
	p[8][0] = 0.0,	p[8][1] = 4.0,	p[8][2] = PI;
	p[9][0] = 2.0,	p[9][1] = 0.0,	p[9][2] = PI;
	p[10][0] = 0.0,	p[10][1] = 0.0;	p[10][2] = PI;

	// prepare
	FILE* file;
	fopen_s(&file,"../data/as.txt","w+");

	int index = 0;
	int tracN = 1;
	double t0 = 0.0;
	double refT = 100;
	double refB = 0.05;
	double start[4] = {0};
	double pos[4],vel[4],acc[4];

	// planner
	bool newM = true;
	bool quit = false;
	PlannerAgv planner;
	planner.updateCurrentPos(start);
	double tvl = 1.5, tal = 5, tjl = 15;
	double rvl = 1.0, ral = 3, rjl = 10;
	planner.setCartesianLimits(tvl,tal,tjl,rvl,ral,rjl);

	// simulated cyclic real-time task
	while(!quit)
	{
		if (newM)
		{
			switch(planner.moveB(p,11,0.5,refB,0,0,refT))
			{
			case PLANNER_SUCCEED:
				index += 1;
				if (index >= tracN)
				{
					newM = false;
				}
				break;
			case ERR_PLAN_FAILED:
				printf("moveB failed!\n");
				quit = true;
				break;
			}
		}

		// get target
		switch(planner.move(PLAN_CYCLE,pos,vel,acc))
		{
		case PLANNER_DONE:
			if (!newM)
			{
				quit = true;
			}
			break;

		case ERR_PLAN_FAILED:
			printf("moveB failed!\n");
			quit = true;
			break;
		}

		// save target
		if (file!=nullptr)
		{
			fprintf(file,"%.20f",t0);
			for (int i=0; i<4; i++)
			{
				fprintf(file,",%.20f",pos[i]);
			}
			for (int i=0; i<4; i++)
			{
				fprintf(file,",%.20f",vel[i]);
			}
			for (int i=0; i<4; i++)
			{
				fprintf(file,",%.20f",acc[i]);
			}
			fprintf(file,"\n");
		}

		t0 += PLAN_CYCLE/1000.0;
	}

	// quit the task
	if (file!=nullptr)
	{
		fclose(file);
	}

	printf("Done! the results are saved in 'as.txt'!\n\n\n");
}

void test_plannerAgv_rotation()
{
	printf("test plannerAgv->rotation ...\n");

	// prepare
	FILE* file;
	fopen_s(&file,"../data/ar.txt","w+");

	double t0 = 0.0;
	double refV = 0.5;
	double refA = 1.0;
	double refJ = 1.0;

	int index = 0;
	int tracN = 2;
	double start[4] = {0};	
	double pos[4],vel[4],acc[4];
	double tcp[4] = {2,0,0,0};
	double targets[2][3] = {{0,0,PI},
							{0,0,0}};

	// planner
	bool newM = true;
	bool quit = false;
	PlannerAgv planner;
	planner.updateCurrentPos(start,tcp);
	double tvl = 1.5, tal = 5, tjl = 15;
	double rvl = 1.0, ral = 3, rjl = 10;
	planner.setCartesianLimits(tvl,tal,tjl,rvl,ral,rjl);

	// simulated cyclic real-time task
	while(!quit)
	{
		if (newM)
		{
			switch(planner.moveR(targets[index],refV,refA,refJ))
			{
			case PLANNER_SUCCEED:
				index += 1;
				if (index >= tracN)
				{
					newM = false;
				}
				break;
			case ERR_PLAN_FAILED:
				printf("moveR failed!\n");
				quit = true;
				break;
			}
		}

		// get target
		switch(planner.move(PLAN_CYCLE,pos,vel,acc))
		{
		case PLANNER_DONE:
			if (!newM)
			{
				quit = true;
			}
			break;

		case ERR_PLAN_FAILED:
			printf("moveR failed!\n");
			quit = true;
			break;
		}

		// save target
		if (file!=nullptr)
		{
			fprintf(file,"%.20f",t0);
			for (int i=0; i<4; i++)
			{
				fprintf(file,",%.20f",pos[i]);
			}
			for (int i=0; i<4; i++)
			{
				fprintf(file,",%.20f",vel[i]);
			}
			for (int i=0; i<4; i++)
			{
				fprintf(file,",%.20f",acc[i]);
			}
			fprintf(file,"\n");
		}

		t0 += PLAN_CYCLE/1000.0;
	}

	// quit the task
	if (file!=nullptr)
	{
		fclose(file);
	}

	printf("Done! the results are saved in 'ar.txt'!\n\n\n");
}

