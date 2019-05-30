
void test_ikine(int cycle)
{
	Kine kine;
	// t d a alp sig mdh off qlim
	double DH[][9] = {
		{0,     0,     0.3,  -PI/2.0,  0,  0,  0,       -PI,PI},
		{0,     0,     0.7,  0,        0,  0,  -PI/2.0, -PI,PI},
		{0,     0,     0.11,  -PI/2.0, 0,  0,  0,       -PI,PI},
		{0,     0.725, 0,     PI/2.0,  0,  0,  0,       -PI,PI},
		{0,     0,     0,     -PI/2.0, 0,  0,  0,       -PI,PI},
		{0,     0,     0,     0,       0,  0,  PI,      -PI,PI}};

	kine.Initiate(DOF,DH);

	double pose[6] = {0};
	double angle[DOF] = {0};

	// zero position
	printf("Test Kinematics ... \n\n");
	printf("Zero Position:\n");
	kine.Fkine(pose,angle);
	for (int i=0; i<6; i++)
	{
		printf("%.5f  ",pose[i]);
	}
	printf("\n");

	kine.Ikine(angle,pose);

	for (int i=0; i<cycle; i++)
	{
		// random joint angle
		for (int j=0; j<DOF; j++)
		{
			angle[j] = (rand() + 0.0)/RAND_MAX * (160.0/180.*PI) * 2.0 - (160.0/180.*PI);
		}

		// forward kinematics
		kine.Fkine(pose,angle);

		// Jacobi of base coordinates
		double J0[6][MAX_DOF];
		kine.Jacob0(J0,angle);

		// Jacobi of flange coordinates
		double Jn[6][MAX_DOF];
		kine.Jacobn(Jn,angle);

		// check Fkine and Jacobi
		double T60[4][4];
		pose2homogeneous(T60,pose);
		double TJ0[6][6] = {{0}};
		double FJ0[6][MAX_DOF] = {{0}};
		for (int j=0; j<3; j++)
		{
			for (int k=0; k<3; k++)
			{
				TJ0[j+3][k] = 0;
				TJ0[j][k+3] = 0;
				TJ0[j][k] = T60[j][k];
				TJ0[j+3][k+3] = T60[j][k];
			}
		}
		M6pN(FJ0,TJ0,Jn,DOF);

		for (int j=0; j<6; j++)
		{
			for (int k=0; k<DOF; k++)
			{
				if (ABS(FJ0[j][k] - J0[j][k]) > ACCURACY_FACTOR)
				{
					printf("\nWrong Fkine and Jacobi for Joint angle:\n");
					for (int m=0; m<DOF; m++)
					{
						printf("%.5f  ",angle[m]);
					}
					printf("\n\n");
					break;
				}
			}
		}

		// random joint angle
		for (int j=0; j<DOF; j++)
		{
			if ((rand() + 0.0)/RAND_MAX*2.0 - 1.0 > 0)
			{
				angle[j] += (rand() + 0.0)/RAND_MAX * (4.0/180.*PI) * 2.0 - (2.0/180.*PI);
			}
		}

		// inverse kinematics
		double tmpPos[6];
		double tmp[MAX_DOF];
		kine.Ikine(tmp,pose,angle);
		kine.Fkine(tmpPos,tmp);
		for (int i=0; i<6; i++)
		{					
			if (ABS(tmpPos[i] - pose[i]) > ACCURACY_FACTOR)
			{
				kine.Ikine(tmp,pose,angle);
				printf("\nWrong Ikine for Joint angle:\n");
				for (int m=0; m<DOF; m++)
				{
					printf("%.5f  ",angle[m]);
				}
				printf("\n\n");
				break;
			}
		}
	}
}
