










#pragma once

#ifndef ROBOTMATH_H_
#define ROBOTMATH_H_

#include <math.h>
#include "typesAndErrors.h"


#ifndef ABS
#define ABS(x)   (((x) < (0)) ? -(x) : (x))
#endif

#ifndef MAX
#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef sign
#define sign(x) (((x) > 0) - ((x) < 0))
#endif


/// Returns X!
int factorial(int x);

/// Returns norm in R3
double norm3(double* a);

/// Return norm in Rn
double normN(double*a, int N);

/// Returns Euclidean distance in R3
double dis3(double* a, double* b);

/// Returns dot product y=x1.x2 in R3
double dot3(double* x1, double* x2);

/// Return Euclidean distance in Rn
double disN(double* a, double* b, int N);

/// Returns dot product y=x1.x2 in Rn
double dotN(double* x1, double* x2, int N);

/// Returns cross multiplication y=x1¡Áx2
void cross3(double* y, double* x1, double* x2);

/// Return distance between a1 and a2 in rad
double radDistance(double angle1,double angle2);

/// get the heading direction from velocity vector in 2D
double headingDirectionIn2D(double vxy[2],double _default);

/// get the turning velocity from velocity vector in 2D
double headingAngularVelIn2D(double vxy[2], double axy[2]);





/// solve tridiagonal linear homogeneous equations, used by B spline
int crout(double* mid,double* up,double* down,double* b,double* x, int n);

/// get center angle of a circle defined by 3 points
double circleAngle(double startPoint[3],double midPoint[3],double endPoint[3]);

/// get normal vector of a circle (norm vector of the plan) defined by 3 points
double circleVector(double nVector[3],double startPoint[3],double midPoint[3],double endPoint[3]);

/// get Center Position and radius of a circle defined by 3 points
double circleCenter(double cenPoint[3],double startPoint[3],double midPoint[3],double endPoint[3]);

/// get the distance and closed points of two lines in 3D space
double distanceOf2LinesIn3D(double cp0[3], double cp1[3], double p0[3],double v0[3],double p1[3], double v1[3]);






/// matrix R1(3x3) transposition
void Trp3(double R[3][3],  double R1[3][3]);

/// matrix multiplication: V(3x1) = M(3x3)*V2(3x1)
void M3p3(double V[3], double M[3][3], double V2[3]);

/// matrix multiplication: V(1x3) = V2(1x3)*M(3x3)
void M3p3(double V[3], double V2[3], double M[3][3]);

/// matrix multiplication: V(4x1) = M(4x4)*V2(4x1)
void M4p4(double V[4], double M[4][4], double V2[4]);

/// matrix multiplication: V(1x4) = V2(1x4)*M(4x4)
void M4p4(double V[4], double V2[4], double R2[4][4]);

/// matrix multiplication: V(6x1) = J(6x6)*V2(6x1)
void M6p6(double V[6], double J[6][6], double V2[6]);

/// matrix multiplication: V(1X6) = V2(1x6)*J(6x6)
void M6p6(double V[6], double V2[6], double J[6][6]);

/// matrix multiplication: M(3x3) = M1(3x3)*M2(3x3)
void M3p3(double M[3][3], double M1[3][3], double M2[3][3]);

/// matrix multiplication: M(4x4) = M1(4x4)*M2(4x4)
void M4p4(double M[4][4], double M1[4][4], double M2[4][4]);

/// matrix multiplication: M(6x6) = M1(6x6)*M2(6x6)
void M6p6(double M[6][6],  double M1[6][6], double M2[6][6]);

/// matrix multiplication: V(1x6) = M(6xN)*V2(Nx1)
void M6pN(double V[6], double M[6][MAX_DOF], double V2[MAX_DOF], int N);

/// matrix multiplication: M(6xN) = M1(6x6)*M2(6xN)
void M6pN(double M[6][MAX_DOF], double M1[6][6], double M2[6][MAX_DOF], int N);





/// a rotation of t radians about the x-axis.
void rot_x(double R[4][4], double t);

/// a rotation of t radians about the y-axis.
void rot_y(double R[4][4], double t);

/// a rotation of t radians about the z-axis.
void rot_z(double R[4][4], double t);

/// skew symmetric matrix of rotate velocity
void vex2skew(double skew[3][3], double p[3]);

/// skew symmetric matrix to rotate velocity
void skew2vex(double p[3], double skew[3][3]);

/// rotation matrix to quaternion
void matrix2quaternion(double* q,double R[3][3]);

/// quaternion to rotation matrix
void quaternion2matrix(double R[3][3], double*q);

/// pose(x,y,z,a,b,c, 6x1) to homogeneous matrix
void pose2homogeneous(double T[4][4], double X[6]);

/// homogeneous to matrix pose(x,y,z,a,b,c, 6x1)
void homogeneous2pose(double X[6], double T[4][4]);

/// translation to homogeneous matrix
void trans2homogeneous(double T[4][4], double X[3]);

/// get translation vector from homogeneous
void homogeneous2trans(double X[3],double T[4][4]);

/// rotation matrix to homogeneous matrix
void rot2homogeneous(double T[4][4], double R[3][3]);

/// get rotation matrix from homogeneous matrix
void homogeneous2rot(double R[3][3], double T[4][4]);

/// get rotate axis and angle from rotation matrix
int matrix2rot(double* k, double* t, double R[3][3]);

/// rotate axis and angle to rotation matrix
int rot2matrix(double R[3][3], double* k, double* t);

/*Convert a rotate matrix to roll-pitch-yaw angles
/*	rpy = [roll, pitch, yaw]
/*	0, ZYX order, R = rotz(yaw) * roty(pitch) * rotx(roll),
/*	       equals R = rotx(roll) * roty(pitch) * rotz(yaw);
/*	1, XYZ order, R = rotx(yaw) * roty(pitch) * rotz(roll);*/
void matrix2rpy(double rpy[],double R[3][3],int type=0);

/*Convert roll-pitch-yaw angles to a rotate matrix
/*	rpy = [roll, pitch, yaw]
/*	0, ZYX order, R = rotz(yaw) * roty(pitch) * rotx(roll),
/*	       equals R = rotx(roll) * roty(pitch) * rotz(yaw);
/*	1, XYZ order, R = rotx(yaw) * roty(pitch) * rotz(roll);*/
void rpy2matrix(double R[3][3],double rpy[],int type=0);

/// homogeneous matrix to differential motion
void tr2delta(double delta[6], double T1[4][4], double T[4][4]);







int pinv(double* pinv_a, double* a, int m, int n);

#endif // !ROBOTMATH_H_