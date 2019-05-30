


/* only suitable for Ming_Robot */



#pragma once

#include "Frame.h"

#define KINE_SIZE	(50)

#define INVALID		(0xFF)

#define SIGULARITY	(1E-2)

#define R_DH_T1		(0.0)
#define R_DH_T2		(-PI/2.0)
#define R_DH_T3		(0.0)
#define R_DH_T4		(0.0)
#define R_DH_T5		(0.0)
#define R_DH_T6		(PI)

#define	R_DH_A1		(300.0/1000.0)
#define R_DH_A2		(700.0/1000.0)
#define R_DH_A3		(110.0/1000.0)
#define R_DH_D4		(725.0/1000.0)



typedef struct R_DH
{

	double  d4;
	double  a[3];
	double	theta[MAX_DOF];

	R_DH(void)
	{
		d4 = R_DH_D4;
		a[0] = R_DH_A1;
		a[1] = R_DH_A2;
		a[2] = R_DH_A3;
		theta[0] = R_DH_T1;
		theta[1] = R_DH_T2;
		theta[2] = R_DH_T3;
		theta[3] = R_DH_T4;
		theta[4] = R_DH_T5;
		theta[5] = R_DH_T6;
	}
}R_DH;

class Kine
{/* RPY for rotation pos */
public:
	Kine();
	~Kine();

public:
	bool    Fkine(double pose[6],double angle[]);

public:
	bool	Jacob0(double Jacob[6][MAX_DOF],double angle[]);

public:
	bool	Ikine(double solutions[8][MAX_DOF], double pose[6]);
	bool	Ikine(double angle[], double pose[6], double ref[]=nullptr);

private:
	int     _dof;
	R_DH	_r_dh;
	double	_angle[MAX_DOF];
	double	_solutions[KINE_SIZE][MAX_DOF];

private:
	bool _innerKine(double T[4][4], double angle[]) const;
	bool _checkRepetition(double angle[], double Q[8][MAX_DOF]) const;
	bool _checkTranslation(double angle[], double Q123[8][MAX_DOF], double T60[4][4]) const;
};