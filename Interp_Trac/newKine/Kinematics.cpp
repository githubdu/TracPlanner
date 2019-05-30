


#pragma once

#include "Kinematics.h"

#define MAX_LOOP	(100)

Kine::Kine()
{
	_serialLink.dof = 0;
}

Kine::~Kine()
{

}

int Kine::GetDof()
{
	return _serialLink.dof;
}

int Kine::Fkine(double pose[6],double angle[])
{
	double T[4][4];
	_serialLink.base.getTransform(T);

	double tmp[4][4],ret[4][4];
	for (int i=0; i<_serialLink.dof; i++)
	{
		_getLinkTransform(tmp,&_serialLink.links[i],angle[i]);
		M4p4(ret,T,tmp); memcpy(T,ret,sizeof(ret));
	}

	_serialLink.flange.getTransform(tmp);
	M4p4(ret,T,tmp); memcpy(T,ret,sizeof(ret));

	homogeneous2pose(pose,T);

	return 0;
}

int Kine::Jacobn(double Jacob[6][MAX_DOF],double angle[])
{
	double U[4][4];
	double tmp[4][4];
	double ret[4][4];
	int dof = _serialLink.dof;

	_serialLink.flange.getTransform(U);

	for (int i=dof-1; i>=0; i--)
	{
		if (0 == _serialLink.links[i].mdh)
		{
			//standard DH convention
			_getLinkTransform(tmp,&_serialLink.links[i],angle[i]);
			M4p4(ret,tmp,U); memcpy(U,ret,sizeof(ret));
		}

		if (0 == _serialLink.links[i].sigma)
		{
			//revolute axis
			Jacob[0][i] = -U[0][0]*U[1][3] + U[1][0]*U[0][3];
			Jacob[1][i] = -U[0][1]*U[1][3] + U[1][1]*U[0][3];
			Jacob[2][i] = -U[0][2]*U[1][3] + U[1][2]*U[0][3];
			Jacob[3][i] =  U[2][0];
			Jacob[4][i] =  U[2][1];
			Jacob[5][i] =  U[2][2];
		}
		else
		{
			// prismatic axis
			Jacob[0][i] = U[2][0];
			Jacob[1][i] = U[2][1];
			Jacob[2][i] = U[2][2];
			Jacob[3][i] = 0;
			Jacob[4][i] = 0;
			Jacob[5][i] = 0;
		}

		if (0 != _serialLink.links[i].mdh)
		{
			//modified DH convention
			_getLinkTransform(tmp,&_serialLink.links[i],angle[i]);
			M4p4(ret, tmp, U); memcpy(U,ret,sizeof(ret));
		}
	}

	return 0;
}

int Kine::Jacob0(double Jacob[6][MAX_DOF],double angle[])
{
	int dof = _serialLink.dof;

	//Jacobian from joint to wrist space
	double jacobn[6][MAX_DOF];
	Jacobn(jacobn,angle);

	//end-effector transformation
	double T[4][4],pose[6];
	Fkine(pose,angle);
	pose2homogeneous(T,pose);

	double R[6][6];
	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			R[i][j+3] = 0;
			R[i+3][j] = 0;
			R[i][j] = T[i][j];
			R[i+3][j+3] = T[i][j];
		}
	}

	//dX_tn = Jn * dq
	M6pN(Jacob,R,jacobn,dof);
	return 0;
}

int Kine::_getLinkTransform(double T[4][4], Link* link, double theta)
{
	double d  = 0;
	double st = 0;
	double ct = 0;
	double a  = link->a;
	double sa = sin(link->alpha);
	double ca = cos(link->alpha);
	double q  = theta + link->offset;

	if (0 == link->sigma)
	{
		//revolute
		st = sin(q);
		ct = cos(q);
		d  = link->d;
	}
	else
	{
		//prismatic
		d  = q;
		st = sin(link->theta);
		ct = cos(link->theta);
	}

	if (0 == link->mdh)
	{
		/*standard D&H
		T =    [    ct  -st*ca    st*sa   a*ct
                    st   ct*ca   -ct*sa   a*st
                    0    sa       ca      d
                    0    0        0       1];*/
		T[0][0] = ct;	T[0][1] = -st*ca;	T[0][2] = st*sa;	T[0][3] = a*ct;
		T[1][0] = st;	T[1][1] = ct*ca;	T[1][2] = -ct*sa;	T[1][3] = a*st;
		T[2][0] = 0;	T[2][1] = sa;		T[2][2] = ca;		T[2][3] = d;
		T[3][0] = 0;	T[3][1] = 0;		T[3][2] = 0;		T[3][3] = 1.0;
	}
	else
	{
		/*modified D&H
		T =    [    ct      -st       0    a
                    st*ca    ct*ca   -sa  -sa*d
                    st*sa    ct*sa    ca   ca*d
                    0        0        0    1];*/
		T[0][0] = ct;		T[0][1] = -st;		T[0][2] = 0;	T[0][3] = a;
		T[1][0] = st*ca;	T[1][1] = ct*ca;	T[1][2] = -sa;	T[1][3] = -sa*d;
		T[2][0] = st*sa;	T[2][1] = ct*sa;	T[2][2] = ca;	T[2][3] = ca*d;
		T[3][0] = 0;		T[3][1] = 0;		T[3][2] = 0;	T[3][3] = 1.0;
	}

	return 0;
}

int Kine::Ikine(double angle[], double pose[6], double ref[],int mask[6])
{

	int iLimit = MAX_LOOP;
	double stol = ACCURACY_FACTOR*0.1;

	int dof = _serialLink.dof;

	double q[MAX_DOF] = {0};
	double dq[MAX_DOF] = {0};
	if (ref != nullptr)
	{
		memcpy(q,ref,sizeof(double)*dof);
	}

	if (dof < 6 && mask == nullptr)
	{
		DUMP_WARNING("WARNING: For a manipulator with fewer than 6DOF a mask matrix argument should be specified!\n");
	}

	int m[6] = {1,1,1,1,1,1};
	if (mask != nullptr)
	{
		for (int i=0; i<6; i++)
		{
			m[i] = (ABS(mask[i])>0)?1:0;
		}
	}

	int count = 0;
	double T[4][4];
	double delta[6];
	double err = 1E20;
	double saveErr[6];
	double saveQ[MAX_DOF];
	double jacob[6][MAX_DOF];
	double tmpJacob[6*MAX_DOF];
	double invJacob[MAX_DOF][6];
	double tmpInvJacob[6*MAX_DOF];

	saveErr[0] = 1E20;
	saveErr[1] = 1E20;
	saveErr[2] = 1E20;
	saveErr[3] = 1E20;
	saveErr[4] = 1E20;
	saveErr[5] = 1E20;

	pose2homogeneous(T,pose);

	double alpha = 1;  // step size

	while(err > stol)
	{
		//----
		count = count + 1;
		if (count > iLimit)
		{
			memcpy(angle,q,sizeof(double)*dof);
			DUMP_ERROR("ERR:Ikine: iteration limit %d, final err %f",iLimit,err);
			DUMP_ERROR("\n  probably because the refJointAngle is in singularity condition!\n");

			return -1;
		}

		//----
		double tmpPose[6];
		double tmp[4][4];
		Fkine(tmpPose,q);
		pose2homogeneous(tmp,tmpPose);
		tr2delta(delta,tmp,T);

		//----
		err = 0;
		double tmpErr = 0;
		for (int i=0; i<6; i++)
		{
			delta[i] *= m[i];
			saveErr[i] *= m[i];
			err += delta[i]*delta[i];
			tmpErr += saveErr[i]*saveErr[i];
		}
		err = sqrt(err);
		tmpErr = sqrt(tmpErr);

		if (err < tmpErr)
		{
			// error reduced, save current state
			memcpy(saveQ,q,sizeof(double)*dof);
			memcpy(saveErr,delta,sizeof(double)*6);
			alpha = alpha * 1.0905077327;//((2.0^(1.0/8)))
		}
		else
		{
			// error got worse, restore to last good solution
			memcpy(q,saveQ,sizeof(double)*dof);
			memcpy(delta,saveErr,sizeof(double)*6);
			alpha = alpha * 0.5;
		}

		//----
		Jacob0(jacob,q);

		for (int i=0; i<6; i++)
		{
			for (int j=0; j<dof; j++)
			{
				tmpJacob[i*6+j] = jacob[i][j];
			}
		}

		pinv(tmpInvJacob,tmpJacob,6,dof);

		for (int i=0; i<dof; i++)
		{
			dq[i] = 0;
			for (int j=0; j<6; j++)
			{
				invJacob[i][j] = tmpInvJacob[i*dof+j];
				dq[i] += invJacob[i][j]*delta[j];
			}
			q[i] += alpha * dq[i];
		}
	}

	memcpy(angle,q,sizeof(double)*dof);

	return 0;
}

int Kine::Initiate(int dof, double DH[][9],double base[], double flange[])
{
	_serialLink.dof = dof;

	for (int i=0; i<dof; i++)
	{
		_serialLink.links[i].theta	= DH[i][0];
		_serialLink.links[i].d		= DH[i][1];
		_serialLink.links[i].a		= DH[i][2];
		_serialLink.links[i].alpha	= DH[i][3];
		_serialLink.links[i].sigma	= (int)DH[i][4];
		_serialLink.links[i].mdh    = (int)DH[i][5];
		_serialLink.links[i].offset	= DH[i][6];
		_serialLink.links[i].qlim[0]= DH[i][7];
		_serialLink.links[i].qlim[1]= DH[i][8];
	}

	if (base!=nullptr)
	{
		_serialLink.base.setWrench(base);
	}

	if (flange!=nullptr)
	{
		_serialLink.flange.setWrench(flange);
	}

	return 0;
}