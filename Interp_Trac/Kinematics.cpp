


#pragma once

#include "Kinematics.h"



#define MAX_LOOP	(500)

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

int Kine::InvJacobn(double invJacob[MAX_DOF][6],double angle[])
{
	double Jacob[6][MAX_DOF];

	Jacobn(Jacob,angle);

	double tmpJacob[6*MAX_DOF];
	double tmpPinvJacob[MAX_DOF*6];
	for (int i=0; i<6; i++)
	{
		for (int j=0; j<_serialLink.dof; j++)
		{
			tmpJacob[i*_serialLink.dof+j] = Jacob[i][j];
		}
	}

	pinv(tmpPinvJacob,tmpJacob,6,_serialLink.dof);

	for (int i=0; i<_serialLink.dof; i++)
	{
		for (int j=0; j<6; j++)
		{
			invJacob[i][j] = tmpPinvJacob[i*6+j];
		}
	}

	return 0;
}

int Kine::InvJacob0(double invJacob[MAX_DOF][6],double angle[])
{
	double Jacob[6][MAX_DOF];

	Jacob0(Jacob,angle);

	double tmpJacob[6*MAX_DOF];
	double tmpPinvJacob[MAX_DOF*6];
	for (int i=0; i<6; i++)
	{
		for (int j=0; j<_serialLink.dof; j++)
		{
			tmpJacob[i*_serialLink.dof+j] = Jacob[i][j];
		}
	}

	pinv(tmpPinvJacob,tmpJacob,6,_serialLink.dof);

	for (int i=0; i<_serialLink.dof; i++)
	{
		for (int j=0; j<6; j++)
		{
			invJacob[i][j] = tmpPinvJacob[i*6+j];
		}
	}

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
	int dof = _serialLink.dof;

	if (dof < 6 && mask == nullptr)
	{
		DUMP_WARNING("WARNING: <Kine::Ikine> For a manipulator with fewer than 6DOF a mask matrix argument should be specified!\n\n");
	}

	int iLimit = MAX_LOOP;
	int rLimit = MAX_LOOP;
	int sLimit = MAX_LOOP;
	double tol = ACCURACY_FACTOR*0.1;

	double lambda = 0.1;
	double lambdaMin = 0;

	int m[6] = {1,1,1,1,1,1};
	if (mask != nullptr)
	{
		for (int i=0; i<6; i++)
		{
			m[i] = (ABS(mask[i])>0)?1:0;
		}
	}

	double W[6][6] = {{0}};
	for (int i=0; i<6; i++)
	{
		W[i][i] = m[i];
	}

	double q[MAX_DOF] = {0};
	double dq[MAX_DOF] = {0};
	double newQ[MAX_DOF] = {0};
	if (ref != nullptr)
	{
		memcpy(q,ref,sizeof(double)*dof);
		memcpy(newQ,ref,sizeof(double)*dof);
	}

	int rejcount = 0;
	int iterations = 0;
	bool failed = false;

	double T[4][4];
	double delta[6];
	double err = 1E20;

	double J[6][MAX_DOF];

	double tJ[MAX_DOF][6];
	double tJW[MAX_DOF][6];

	double tJWJ[MAX_DOF][MAX_DOF];
	double tmpTJWJ[MAX_DOF*MAX_DOF];

	double invTJWJ[MAX_DOF][MAX_DOF];
	double tmpInvTJWJ[MAX_DOF*MAX_DOF];

	double invTJWJtJ[MAX_DOF][6];
	double invTJWJtJW[MAX_DOF][6];

	pose2homogeneous(T,pose);

	while(true)
	{
		//update the count and test against iteration limit
		iterations = iterations + 1;
		if (iterations > iLimit)
		{
			memcpy(angle,q,sizeof(double)*dof);
			DUMP_ERROR("ERR: <Kine::Ikine> Iteration limit %d exceeded, final err %f\n\n",iLimit,err);
			failed = true;
			break;
		}

		//----
		err = 0;
		double tmpPose[6];
		double tmpFlange[4][4];
		Fkine(tmpPose,q);
		pose2homogeneous(tmpFlange,tmpPose);
		tr2delta(delta,tmpFlange,T);

		for (int i=0; i<6; i++)
		{
			delta[i] *= m[i];
			err += delta[i]*delta[i];
		}
		err = sqrt(err);
		if (err < tol)
		{
			break;
		}

		// JtJ = J'*W*J; JtJ += (lambda + opt.lambdamin) * eye(size(JtJ))
		Jacobn(J,q);
		for (int i=0; i<dof; i++)
		{
			for (int j=0; j<6; j++)
			{
				tJ[i][j] = J[j][i];
			}
		}
		for (int i=0; i<dof; i++)
		{
			for (int j=0; j<6; j++)
			{
				tJW[i][j] = 0.0;
				for (int k=0; k<6; k++)
				{
					tJW[i][j] += tJ[i][k] * W[k][j];
				}
			}
		}
		for (int i=0; i<dof; i++)
		{
			for (int j=0; j<dof; j++)
			{
				tJWJ[i][j] = 0;
				for (int k=0; k<6;k++)
				{
					tJWJ[i][j] += tJW[i][k]*J[k][j];
				}
			}
			tJWJ[i][i] += (lambda + lambdaMin);
		}

		//	dq = inv(JtJ + (lambda + opt.lambdamin) * eye(size(JtJ)) ) * J' * W * e;
		for (int i=0; i<dof; i++)
		{
			for (int j=0; j<dof; j++)
			{
				tmpTJWJ[i*dof+j] = tJWJ[i][j];
			}
		}

		inv(tmpInvTJWJ,tmpTJWJ,dof);

		for (int i=0; i<dof; i++)
		{
			for (int j=0; j<dof; j++)
			{
				invTJWJ[i][j] = tmpInvTJWJ[i*dof+j];
			}
		}

		
		// test inv
		double ret[MAX_DOF][MAX_DOF] = {{0}};
		for (int i=0; i<dof; i++)
		{
			for (int j=0; j<dof; j++)
			{
				ret[i][j] = 0;
				for (int k=0; k<dof; k++)
				{
					ret[i][j] += tJWJ[i][k]*invTJWJ[k][j];
				}

				if (i==j && ABS(ret[i][j] - 1.0) > ACCURACY_FACTOR)
				{
					memcpy(angle,q,sizeof(double)*dof);
					DUMP_ERROR("ERR: <Kine::Ikine> 1.0 lambda is too large: %.1f, final err %f\n\n",lambda,err);
					failed = true;
					break;
				}

				if (i!=j && ABS(ret[i][j]) > ACCURACY_FACTOR)
				{
					memcpy(angle,q,sizeof(double)*dof);
					DUMP_ERROR("ERR: <Kine::Ikine> 0.0 lambda is too large: %.1f, final err %f\n\n",lambda,err);
					failed = true;
					break;
				}
			}
		}

		//---
		for (int i=0; i<dof; i++)
		{
			for (int j=0; j<6; j++)
			{
				invTJWJtJ[i][j] = 0;
				for (int k=0; k<dof; k++)
				{
					invTJWJtJ[i][j] += invTJWJ[i][k]*tJ[k][j];
				}
			}
		}

		for (int i=0; i<dof; i++)
		{
			for (int j=0; j<6; j++)
			{
				invTJWJtJW[i][j] = 0;
				for (int k=0; k<6; k++)
				{
					invTJWJtJW[i][j] += invTJWJtJ[i][k]*W[k][j];
				}
			}
		}

		for (int i=0; i<dof; i++)
		{
			dq[i] = 0;
			for (int j=0; j<6; j++)
			{
				dq [i] += invTJWJtJW[i][j]*delta[j];
			}
		}

		for (int i=0; i<dof; i++)
		{
			newQ[i] = q[i] + dq[i];
		}

		// new error
		double newErr = 0;
		double newDelta[6];
		Fkine(tmpPose,newQ);
		pose2homogeneous(tmpFlange,tmpPose);
		tr2delta(newDelta,tmpFlange,T);

		for (int i=0; i<6; i++)
		{
			newDelta[i] *= m[i];
			newErr += newDelta[i]*newDelta[i];
		}
		newErr = sqrt(newErr);

		// was it a good update?
		if (newErr < err)
		{
			memcpy(delta,newDelta,sizeof(delta));
			memcpy(q,newQ,sizeof(double)*dof);
			lambda = lambda/2.0;
			rejcount = 0;
			err = newErr;
		}
		else
		{
			// increase the damping and retry
			lambda = lambda*2.0;
			rejcount = rejcount + 1;
			if (rejcount > rLimit)
			{
				failed = true;
				DUMP_ERROR("ERR: <Kine::Ikine> Rejected-step limit %d exceeded, final err %f\n\n",rejcount,err);
				break;
			}
		}

		// wrap angles for revolute joints
		for (int i=0; i<dof; i++)
		{
			if (_serialLink.links[i].sigma == 0)
			{
				if (q[i] > PI)
				{
					q[i] = q[i] - 2.0*PI;
				}
				if (q[i] < -PI)
				{
					q[i] = q[i] + 2.0*PI;
				}
			}
		}
	}

	memcpy(angle,q,sizeof(double)*dof);

	if (failed)
	{
		DUMP_ERROR("ERR:  <Kine::Ikine> Failed to converge: try a different initial value of joint coordinates!\n\n");
	}

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