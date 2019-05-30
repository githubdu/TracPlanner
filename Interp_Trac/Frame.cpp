




#pragma  once

#include "Frame.h"


Point6D::Point6D()
{
	cartesianZero();
}

Point6D::~Point6D()
{

}

double& Point6D::operator[](int i)
{
	switch(i)
	{
	case 0:
		return X;
		break;

	case 1:
		return Y;
		break;

	case 2:
		return Z;
		break;

	case 3:
		return A;
		break;

	case 4:
		return B;
		break;

	case 5:
		return C;
		break;

	default:
		DUMP_ERROR("ERR: out of range in <R_Point6D>!\n");
		return X;
	}
}

bool Point6D::operator==(Point6D point)
{
	if (X != point.X)
	{
		return false;
	}

	if (Y != point.Y)
	{
		return false;
	}

	if (Z != point.Z)
	{
		return false;
	}

	if (A != point.A)
	{
		return false;;
	}

	if (B != point.B)
	{
		return false;
	}

	if (C != point.C)
	{
		return false;
	}

	return true;
}

bool Point6D::operator!=(Point6D point)
{
	if (   X != point.X 
		|| Y != point.Y
		|| Z != point.Z
		|| A != point.A
		|| B != point.B
		|| C != point.C)
	{
		return true;
	}

	return false;
}

Point6D Point6D::operator*(Point6D point)
{
	double A[4][4] = {{0}};
	double B[4][4] = {{0}};
	double C[4][4] = {{0}};

	this->getTransform(A);
	point.getTransform(B);
	
	M4p4(C,A,B);

	Point6D ret;
	ret.setTransform(C);

	return ret;
}

Point6D Point6D::operator/(Point6D point)
{
	Point6D ret;
	double A[4][4] = {{0}};
	double B[4][4] = {{0}};
	double C[4][4] = {{0}};

	this->getTransform(A);
	ret = point.getInverse();
	ret.getTransform(B);

	M4p4(C,A,B);
	ret.setTransform(C);

	return ret;
}

bool Point6D::cartesianZero()
{
	X = 0.0; Y = 0.0; Z = 0.0;
	A = 0.0; B = 0.0; C = 0.0;

	return true;
}

bool Point6D::copyFrom(Point6D refP)
{
	for (int i=0; i<6; i++)
	{
		(*this)[i] = refP[i];
	}

	return true;
}

bool Point6D::setPos(double Pos[3])
{
	X = Pos[0];
	Y = Pos[1];
	Z = Pos[2];
	return true;
}

bool Point6D::getPos(double Pos[3])
{
	Pos[0] = X;
	Pos[1] = Y;
	Pos[2] = Z;
	return true;
}

bool Point6D::setRPY(double RPY[3])
{
	A = RPY[0];
	B = RPY[1];
	C = RPY[2];
	return true;
}

bool Point6D::getRPY(double RPY[3])
{
	RPY[0] = A;
	RPY[1] = B;
	RPY[2] = C;
	return true;
}

bool Point6D::setRot(double R[3][3])
{
	double RPY[3] = {0};
	matrix2rpy(RPY,R);

	return setRPY(RPY);
}

bool Point6D::getRot(double R[3][3])
{
	double RPY[3] = {0};
	getRPY(RPY);

	rpy2matrix(R,RPY);

	return true;
}

bool Point6D::setWrench(double wrench[6])
{
	setPos(wrench);
	return setRPY(wrench+3);
}

bool Point6D::getWrench(double wrench[6])
{
	getPos(wrench);
	return getRPY(wrench+3);
}

bool Point6D::setTransform(double T[4][4])
{
	X = T[0][3];
	Y = T[1][3];
	Z = T[2][3];

	double R[3][3] = { { 0 } };
	homogeneous2rot(R, T);

	return setRot(R);
}

bool Point6D::getTransform(double T[4][4])
{
	double R[3][3] = { { 0 } };
	getRot(R);

	rot2homogeneous(T, R);
	T[0][3] = X;
	T[1][3] = Y;
	T[2][3] = Z;

	return true;
}

Point6D Point6D::getInverse()
{
	double R[3][3], invR[3][3], P[3], invP[3];
	this->getRot(R);
	Trp3(invR,R);

	this->getPos(P);
	M3p3(invP,invR,P);

	invP[0] = -invP[0];
	invP[1] = -invP[1];
	invP[2] = -invP[2];

	Point6D ret;
	ret.setRot(invR);
	ret.setPos(invP);

	return ret;
}

bool Point6D::setQuaternion(double Quaternion[4])
{
	double R[3][3] = {{0}};
	quaternion2matrix(R,Quaternion);
	return setRot(R);
}

bool Point6D::getQuaternion(double Quaternion[4])
{
	double R[3][3] = {{0}};
	getRot(R);
	matrix2quaternion(Quaternion,R);

	return true;
}
