


#pragma once

#include "Kinematics.h"

Kine::Kine()
{
	_dof = 6;
	_r_dh.d4 = R_DH_D4;
	_r_dh.a[0] = R_DH_A1;
	_r_dh.a[1] = R_DH_A2;
	_r_dh.a[2] = R_DH_A3;

	_r_dh.theta[0] = R_DH_T1;
	_r_dh.theta[1] = R_DH_T2;
	_r_dh.theta[2] = R_DH_T3;
	_r_dh.theta[3] = R_DH_T4;
	_r_dh.theta[4] = R_DH_T5;
	_r_dh.theta[5] = R_DH_T6;
}

Kine::~Kine(){}

bool Kine::Fkine(double pose[6],double angle[])
{
	double T[4][4];

	for (int i = 0; i < _dof; i++)
	{
		_angle[i] = this->_r_dh.theta[i] + angle[i];
	}

	bool ret = _innerKine(T,_angle);

	homogeneous2pose(pose,T);

	return ret;
}

bool Kine::Jacob0(double Jacob[6][MAX_DOF],double angle[])
{
	for (int i = 0; i < _dof; i++)
	{
		_angle[i] = this->_r_dh.theta[i] + angle[i];
	}

	double d4 = this->_r_dh.d4;
	double a1 = this->_r_dh.a[0];
	double a2 = this->_r_dh.a[1];
	double a3 = this->_r_dh.a[2];

	double c1 = cos(_angle[0]), s1 = sin(_angle[0]);
	double c2 = cos(_angle[1]), s2 = sin(_angle[1]);
	double c3 = cos(_angle[2]), s3 = sin(_angle[2]);
	double c4 = cos(_angle[3]), s4 = sin(_angle[3]);
	double c5 = cos(_angle[4]), s5 = sin(_angle[4]);
	double c6 = cos(_angle[5]), s6 = sin(_angle[5]);

	// J11 - J16
	Jacob[0][0] = d4*(c2*s1*s3 + c3*s1*s2) - a1*s1 -  
					a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3;

	Jacob[0][1] = -c1*(d4*(c2*c3 - s2*s3) + a2*s2 + a3*c2*s3 + a3*c3*s2);

	Jacob[0][2] = -c1*(c2*(d4*c3 + a3*s3) + s2*(a3*c3 - d4*s3));
	Jacob[0][3] = 0;
	Jacob[0][4] = 0;
	Jacob[0][5] = 0;

	// J21 - J26
	Jacob[1][0] = a1*c1 - d4*(c1*c2*s3 + c1*c3*s2) +  
					a2*c1*c2 + a3*c1*c2*c3 - a3*c1*s2*s3;

	Jacob[1][1] = -s1*(d4*(c2*c3 - s2*s3) + a2*s2 + a3*c2*s3 + a3*c3*s2);

	Jacob[1][2] = -s1*(c2*(d4*c3 + a3*s3) + s2*(a3*c3 - d4*s3));
	Jacob[1][3] = 0;
	Jacob[1][4] = 0;
	Jacob[1][5] = 0;

	// J31 - J36
	Jacob[2][0] = 0;
	Jacob[2][1] = c1*c1 * (d4*(c2*s3 + c3*s2) - a2*c2 - a3*c2*c3 + a3*s2*s3) +  
					s1*s1 * (d4*(c2*s3 + c3*s2) - a2*c2 - a3*c2*c3 + a3*s2*s3);

	Jacob[2][2] = -c1*(c1*c2*(a3*c3 - d4*s3) - c1*s2*(d4*c3 + a3*s3)) -  
					s1*(c2*s1*(a3*c3 - d4*s3) - s1*s2*(d4*c3 + a3*s3));

	Jacob[2][3] = 0;
	Jacob[2][4] = 0;
	Jacob[2][5] = 0;

	//J41-J46
	Jacob[3][0] = 0;
	Jacob[3][1] = -s1;
	Jacob[3][2] = -s1;
	Jacob[3][3] = -c1*c2*s3 - c1*c3*s2;
	Jacob[3][4] = -c4*s1 - s4*(c1*s2*s3 - c1*c2*c3);
	Jacob[3][5] = -s5*(s1*s4 - c4*(c1*s2*s3 - c1*c2*c3)) - 
					c5*(c1*c2*s3 + c1*c3*s2);

	//J51-J56
	Jacob[4][0] = 0;
	Jacob[4][1] = c1;
	Jacob[4][2] = c1;
	Jacob[4][3] = -c2*s1*s3 - c3*s1*s2;
	Jacob[4][4] = c1*c4 - s4*(s1*s2*s3 - c2*c3*s1);
	Jacob[4][5] = s5*(c1*s4 + c4*(s1*s2*s3 - c2*c3*s1)) -  
					c5*(c2*s1*s3 + c3*s1*s2);

	//J61-J66
	Jacob[5][0] = 1;
	Jacob[5][1] = 0;
	Jacob[5][2] = 0;
	Jacob[5][3] = s2*s3 - c2*c3;
	Jacob[5][4] = -s4*(c2*s3 + c3*s2);
	Jacob[5][5] = c4*s5*(c2*s3 + c3*s2) - c5*(c2*c3 - s2*s3);

	return true;
}

bool Kine::_innerKine(double T60[4][4], double angle[]) const
{
	double d4 = this->_r_dh.d4;
	double a1 = this->_r_dh.a[0];
	double a2 = this->_r_dh.a[1];
	double a3 = this->_r_dh.a[2];

	double c1 = cos(angle[0]), s1 = sin(angle[0]);
	double c2 = cos(angle[1]), s2 = sin(angle[1]);
	double c3 = cos(angle[2]), s3 = sin(angle[2]);
	double c4 = cos(angle[3]), s4 = sin(angle[3]);
	double c5 = cos(angle[4]), s5 = sin(angle[4]);
	double c6 = cos(angle[5]), s6 = sin(angle[5]);

	double nx = s6*(c4*s1 + s4*(c1*s2*s3 - c1*c2*c3)) + c6*(c5*(s1*s4 - 
		        c4*(c1*s2*s3 - c1*c2*c3)) - s5*(c1*c2*s3 + c1*c3*s2));

	double ny = -s6*(c1*c4 - s4*(s1*s2*s3 - c2*c3*s1)) - c6*(c5*(c1*s4 +  
		        c4*(s1*s2*s3 - c2*c3*s1)) + s5*(c2*s1*s3 + c3*s1*s2));

	double nz = s4*s6*(c2*s3 + c3*s2) - c6*(s5*(c2*c3 - s2*s3) +  
		        c4*c5*(c2*s3 + c3*s2));

	double ox = c6*(c4*s1 + s4*(c1*s2*s3 - c1*c2*c3)) - s6*(c5*(s1*s4 -  
		        c4*(c1*s2*s3 - c1*c2*c3)) - s5*(c1*c2*s3 + c1*c3*s2));

	double oy = s6*(c5*(c1*s4 + c4*(s1*s2*s3 - c2*c3*s1)) + s5*(c2*s1*s3 +  
		        c3*s1*s2)) - c6*(c1*c4 - s4*(s1*s2*s3 - c2*c3*s1));

	double oz = s6*(s5*(c2*c3 - s2*s3) + c4*c5*(c2*s3 + c3*s2)) +  
		        c6*s4*(c2*s3 + c3*s2);

	double ax = -s5*(s1*s4 - c4*(c1*s2*s3 - c1*c2*c3)) - c5*(c1*c2*s3 + c1*c3*s2);

	double ay = s5*(c1*s4 + c4*(s1*s2*s3 - c2*c3*s1)) - c5*(c2*s1*s3 + c3*s1*s2);

	double az = c4*s5*(c2*s3 + c3*s2) - c5*(c2*c3 - s2*s3);

	double px = a1*c1 - d4*(c1*c2*s3 + c1*c3*s2) + a2*c1*c2 +  
		        a3*c1*c2*c3 - a3*c1*s2*s3;

	double py = a1*s1 - d4*(c2*s1*s3 + c3*s1*s2) + a2*c2*s1 +  
		        a3*c2*c3*s1 - a3*s1*s2*s3;

	double pz = -d4*(c2*c3 - s2*s3) - a2*s2 - a3*c2*s3 - a3*c3*s2;

	T60[0][0] = nx; T60[0][1] = ox; T60[0][2] = ax; T60[0][3] = px;
	T60[1][0] = ny; T60[1][1] = oy; T60[1][2] = ay; T60[1][3] = py;
	T60[2][0] = nz; T60[2][1] = oz; T60[2][2] = az; T60[2][3] = pz;
	T60[3][0] = 0;  T60[3][1] = 0;  T60[3][2] = 0;  T60[3][3] = 1.0;

	return true;
}

bool Kine::Ikine(double solutions[KINE_SIZE][MAX_DOF], double pose[6])
{
	double T60[4][4] = {{0}};
	pose2homogeneous(T60,pose);

	double px = T60[0][3];
	double py = T60[1][3];
	double pz = T60[2][3];

	double ax = T60[0][2];
	double ay = T60[1][2];
	double az = T60[2][2];

	double nx = T60[0][0];
	double ny = T60[1][0];
	double nz = T60[2][0];
	
	double d4 = this->_r_dh.d4;
	double a1 = this->_r_dh.a[0];
	double a2 = this->_r_dh.a[1];
	double a3 = this->_r_dh.a[2];

	double Q123[KINE_SIZE][MAX_DOF] = {{0}};
	for (int i=0; i<KINE_SIZE; i++)
	{
		for (int j=0; j<_dof; j++)
		{
			Q123[i][j] = INVALID;
			solutions[i][j] = INVALID;
			_solutions[i][j] = INVALID;
		}
	}

	//-------------------------the first three joints-----------------------
	// the 1st joint
	double q1[2] = {0.0};
	q1[0] = atan2(py, px); 
	q1[1] = q1[0] - sign(q1[0])*PI;
	if (ABS(px) > ACCURACY_FACTOR)
	{
		q1[0] = atan(py/px);
		q1[1] = q1[0] -sign(q1[0])*PI;
	}
	else
	{
		q1[0] = PI/2.0;
		q1[1] = -PI/2.0;
	}

	if (0 == q1[0])
	{
		q1[1] = PI;
	}

	// the 2nd joint
	int index = 0;
	for (int i = 0; i < 2; i++)
	{
		double c1 = cos(q1[i]);
		double s1 = sin(q1[i]);

		double z26 = pz - 0;
		double x26 = px - c1*a1;
		double y26 = py - s1*a1;
		double d36 = sqrt(a3*a3 + d4*d4);
		double d26 = sqrt(x26*x26 + y26*y26 + z26*z26);
		double a326= acos((d26*d26 + a2*a2 - d36*d36)/(2.0*d26*a2));
		double a26 = -asin(z26/d26);

		double q2[4] = {0};
		q2[0] =  a326 + a26;
		q2[1] = -a326 + a26;
		q2[2] = radDistance(0,-a326 - a26 + PI) ;
		q2[3] = radDistance(0, a326 - a26 + PI) ;

		// the 3rd joint
		for (int j = 0; j < 4; j++)
		{
			double c2 = cos(q2[j]);
			double s2 = sin(q2[j]);
			double z6 = -d4*c2 - a2*s2 - a3*1*s2;
			double x6 = a1*c1 - d4*c1*s2 + a2*c1*c2 + a3*c1*c2;
			double y6 = a1*s1 - d4*s1*s2 + a2*c2*s1 + a3*c2*s1;
			double d66 = sqrt((px-x6)*(px-x6) + (py-y6)*(py-y6) + (pz-z6)*(pz-z6));
			double a636 = acos((d36*d36 +d36*d36 - d66*d66)/(2.0*d36*d36));
			double q3[2] = {a636,-a636};

			// check 
			for (int k = 0; k < 2; k++)
			{
				double temp[6] = { q1[i], q2[j], q3[k],INVALID,INVALID,INVALID};
				if (_checkTranslation(temp, Q123, T60))
				{
					Q123[index][0] = temp[0];
					Q123[index][1] = temp[1];
					Q123[index][2] = temp[2];
					index = index + 1;
				}
			}
		}
	}

	//-------------------------the last three joints-----------------------
	int Q123Num = index; index = 0;
	for (int i = 0; i < Q123Num; i++)
	{
		double c1 = cos(Q123[i][0]), s1 = sin(Q123[i][0]);
		double c2 = cos(Q123[i][1]), s2 = sin(Q123[i][1]);
		double c3 = cos(Q123[i][2]), s3 = sin(Q123[i][2]);
		double c5 = az*s2*s3 - az*c2*c3 - ax*c1*c2*s3 - ax*c1*c3*s2 - ay*c2*s1*s3 - ay*c3*s1*s2;
		double q5[2] = {acos(c5),-acos(c5)}, q4[2] = {0}, q6[2] = {0};
		for (int j=0; j<2; j++)
		{
			double s5 = sin(q5[j]);

			if (ABS(s5) < SIGULARITY)
			{
				// singularity, let q4 = 0
			}
			else
			{
				double c4 = ax*c1*c2*c3 - az*c3*s2 - az*c2*s3;
				c4 = c4 + ay*c2*c3*s1 - ax*c1*s2*s3 - ay*s1*s2*s3;
				c4 = -c4/s5; 
				if (c4 > 1.0)
				{
					c4 = 1.0;
				}
				if (c4 < -1.0)
				{
					c4 = -1.0;
				}
				q4[0] = acos(c4);q4[1] = -q4[0];
			}

			// the 6th joint
			for (int k = 0; k < 2; k++)
			{
				double R[3][3] = {{0}}, deltaR[3][3] = {{0}}, tempR[3][3] = {{0}}, tempT[4][4] = {{0.0}};
				double angle[6] = {Q123[i][0],Q123[i][1],Q123[i][2],q4[k],q5[j],0}, tempV[3] = {0,0,1};
				_innerKine(tempT,angle); homogeneous2rot(tempR,tempT); Trp3(deltaR,tempR);
				homogeneous2rot(tempR,T60); M3p3(R, deltaR,tempR);
				matrix2rot(tempV,q6,R);q6[1] = q6[0] - 2.0*PI;

				for (int l=0; l<2; l++)
				{
					angle[5] = q6[l];
					_innerKine(tempT,angle);
					bool flag = true;
					for (int m = 0; m<3; m++)
					{
						for (int n = 0; n<3; n++)
						{
							if (ABS(tempT[m][n] - T60[m][n]) > ACCURACY_FACTOR)
							{
								flag = false;
							}
						}
					}
					if (flag && !_checkRepetition(angle,_solutions))
					{
						if (index > KINE_SIZE-1)
						{
							DUMP_ERROR("ERR: too many inverse kinematics solutions!\n\n");
							return false;
						}
						memcpy(_solutions[index],angle,sizeof(double)*_dof);
						index = index + 1;
					}
				}
			}
		}
	}

	for (int i = 0; i < KINE_SIZE; i++)
	{
		for (int j = 0; j < _dof; j++)
		{
			if (i < index)
			{
				solutions[i][j] = radDistance(0,_solutions[i][j] - _r_dh.theta[j]);
			}
			else
			{
				solutions[i][j] = INVALID;
			}
		}
	}

	if (index < KINE_SIZE)
	{
		solutions[index][0] = INVALID;
	}

	return true;
}

bool Kine::Ikine(double angle[], double pose[6], double ref[])
{
	/* NOT GOOD !!! */

	double solution[KINE_SIZE][MAX_DOF] = {{0}};
	Ikine(solution,pose);

	int index = 0;
	double sum = 1e20;
	for (int i=0; i<KINE_SIZE; i++)
	{
		if (solution[i][0] == INVALID)
		{
			break;
		}

		if (ABS(sin(solution[i][4] + this->_r_dh.theta[4])) < SIGULARITY)
		{
			solution[i][5] = solution[i][5] + solution[i][3];
			solution[i][3] = ref[3];
			solution[i][5] = solution[i][5] - ref[3];
		}

		double temp = 0;
		for (int j=0; j<_dof; j++)
		{
			temp += ABS(radDistance(ref[j],solution[i][j]));
		}
		if (temp < sum)
		{
			sum = temp;
			index = i;
		}
	}

	memcpy(angle,solution[index],sizeof(double)*_dof);

	return true;

}

bool Kine::_checkRepetition(double angle[], double Q[KINE_SIZE][MAX_DOF]) const
{
	bool flag = false;
	for (int i = 1; i < KINE_SIZE; i++)
	{
		bool inner_flag = true;
		for (int j = 0; j < _dof; j++)
		{
			if (ABS(radDistance(angle[j],Q[i][j])) > ACCURACY_FACTOR )
			{
				inner_flag = false;
			}
		}

		if (inner_flag) 
		{
			flag = true;
			break;
		}
	}

	return flag;
}

bool Kine::_checkTranslation(double angle[], double Q123[KINE_SIZE][MAX_DOF], double T60[4][4]) const
{
	// using px py and pz to remove the wrong solution of the first 3 joints

	bool flag = false;

	double px = T60[0][3];
	double py = T60[1][3];
	double pz = T60[2][3];

	double d4 = this->_r_dh.d4;
	double a1 = this->_r_dh.a[0];
	double a2 = this->_r_dh.a[1];
	double a3 = this->_r_dh.a[2];

	double c1 = cos(angle[0]), s1 = sin(angle[0]);
	double c2 = cos(angle[1]), s2 = sin(angle[1]);
	double c3 = cos(angle[2]), s3 = sin(angle[2]);

	double Pz = -d4*(c2*c3 - s2*s3) - a2*s2 - a3*c2*s3 -  a3*c3*s2;
	double Px = a1*c1 - d4*(c1*c2*s3 + c1*c3*s2) + a2*c1*c2 + a3*c1*c2*c3 - a3*c1*s2*s3;
	double Py = a1*s1 - d4*(c2*s1*s3 + c3*s1*s2) + a2*c2*s1 + a3*c2*c3*s1 - a3*s1*s2*s3;

	if ((abs(Px - px) < ACCURACY_FACTOR) && (abs(Py - py) < ACCURACY_FACTOR) && (abs(Pz - pz) < ACCURACY_FACTOR))
	{
		if (!_checkRepetition(angle, Q123))
		{
			flag = true; // save the new solution
		}
	}

	return flag;
}
