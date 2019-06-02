


/* only suitable for Ming_Robot */



#pragma once

#include "Frame.h"


typedef struct  
{
	double theta;	// joint angle
	double d;		// link offset
	double a;		// link length
	double alpha;	// link twist
	int    sigma;	// o if revolute, 1 if prismatic
	int    mdh;		// 0 if standard D&H, else 1
	double offset;	// joint offset
	double qlim[2];	// joint limits, [min max]

	double m;		// dynamic, not used yet
	double r[3];	// dynamic, not used yet
	double I[3][3]; // dynamic, not used yet
	double B;		// dynamic, not used yet
	double Tc;		// dynamic, not used yet
	double G;		// dynamic, not used yet
	double Jm;		// dynamic, not used yet
}Link;

typedef struct
{
	int		dof;
	Point6D	base;
	Point6D flange;
	Link	links[MAX_DOF];
}SerialLink;


class Kine
{/* RPY for rotation pos */
public:
	Kine();
	~Kine();

public:
	int GetDof();

public:
	int Fkine(double pose[6],double angle[]);

public:
	int Jacobn(double Jacob[][MAX_DOF],double angle[]);
	int Jacob0(double Jacob[6][MAX_DOF],double angle[]);
	int InvJacobn(double invJacob[MAX_DOF][6],double angle[]);
	int InvJacob0(double invJacob[MAX_DOF][6],double angle[]);

public:
	int Ikine(double angle[], double pose[6], double ref[]=nullptr,int mask[6]=nullptr);

public:
	int Initiate(int dof, double DH[][9],double base[6]=nullptr, double flange[6]=nullptr);

private:
	SerialLink _serialLink;
	int _getLinkTransform(double T[4][4], Link* link, double theta);
};