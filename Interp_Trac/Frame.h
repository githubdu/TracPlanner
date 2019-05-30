









#pragma once

#ifndef FRAMES_H_
#define FRAMES_H_

#include "robotMath.h"

class Point6D
{
public:

	Point6D();
	~Point6D();

public:
	double X;
	double Y;
	double Z;
	double A;
	double B;
	double C;

public:
	double& operator[](int i);

	bool operator == (Point6D point);

	bool operator != (Point6D point);

	Point6D operator * (Point6D point);

	Point6D operator / (Point6D point);

public:
	Point6D getInverse();

public:

	bool cartesianZero();

	bool copyFrom(Point6D refP);

	bool setPos(double Pos[3]);
	bool getPos(double Pos[3]);

	bool setRPY(double RPY[3]);
	bool getRPY(double RPY[3]);

	bool setRot(double R[3][3]);
	bool getRot(double R[3][3]);

	bool setWrench(double wrench[6]);
	bool getWrench(double wrench[6]);

	bool setTransform(double T[4][4]);
	bool getTransform(double T[4][4]);

	bool setQuaternion(double Quaternion[4]);
	bool getQuaternion(double Quaternion[4]);

};

#endif