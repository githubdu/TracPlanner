


#pragma once
#include "robotMath.h"



int factorial(int x)
{
	int y=1;

	for(int i=1; i<=x; i++)
	{
		y = y*i;
	}

	return y;
}

double norm3(double* a)
{
	double c = 0.0;

	c = sqrt(a[0]*a[0] + 
		     a[1]*a[1] + 
			 a[2]*a[2]);

	return c;
}

double normN(double* a,int N)
{
	double c = 0.0;

	for (int i=0; i<N; i++)
	{
		c = c + a[i]*a[i];
	}

	return sqrt(c);
}

double dis3(double* a, double* b)
{
	double c = 0.0;

	c = sqrt((a[0]-b[0])*(a[0]-b[0])
		    +(a[1]-b[1])*(a[1]-b[1])
			+(a[2]-b[2])*(a[2]-b[2]));

	return c;
}

double dot3(double* x1, double* x2)
{
	double y = x1[0]*x2[0] 
			 + x1[1]*x2[1] 
			 + x1[2]*x2[2];
	return y;
}

double disN(double* a, double* b,int N)
{
	double c = 0.0;

	for (int i=0; i<N; i++)
	{
		c = c + (a[i]-b[i])*(a[i]-b[i]);
	}

	return sqrt(c);
}

double dotN(double* x1,double* x2,int N)
{
	double y = 0.0;

	for (int i=0; i<N; i++)
	{
		y = y + x1[i]*x2[i];
	}

	return y;
}

void cross3(double* y,double* x1,double* x2)
{
    y[0] = x1[1]*x2[2] - x1[2]*x2[1];
    y[1] = x1[2]*x2[0] - x1[0]*x2[2];
    y[2] = x1[0]*x2[1] - x1[1]*x2[0];

}

double radDistance(double angle1,double angle2)
{
	double ret = (angle2 - angle1);
	return ret - floor((ret+PI)/(2*PI+ALMOST_ZERO))*2*PI;
}

double headingDirectionIn2D(double vxy[2],double _default)
{
	if (normN(vxy,2) < ALMOST_ZERO)
	{
		return _default;
	}

	if (ABS(vxy[1]) < ALMOST_ZERO)
	{
		if ((vxy[0]) > ALMOST_ZERO)
		{
			return 0;
		}
		else
		{
			return PI;
		}
	}

	return sign(vxy[1]) * acos(vxy[0]/normN(vxy,2));
}

double headingAngularVelIn2D(double vxy[2], double axy[2])
{
	if (normN(vxy,2) < ACCURACY_FACTOR)
	{
		return 0.0;
	}

	double K = axy[1]*vxy[0] - vxy[1]*axy[0];
	
	return K/normN(vxy,2);
}




int crout(double* mid,double* up,double* down,double* b,double* x, int n)
{
	//double p[n],q[n-1],y[n];
	double* p = (double*)malloc(sizeof(double)*n);
	double* y = (double*)malloc(sizeof(double)*n);
	double* q = (double*)malloc(sizeof(double)*(n-1));

	p[0] = mid[0];

	for(int i=0;i<n-1;i++)
	{
		q[i] = up[i]/p[i];
		p[i+1] = mid[i+1]-down[i]*q[i];
	}

	y[0] = b[0]/p[0];

	for(int i=1;i<n;i++)
	{
		y[i] = (b[i]-down[i-1]*y[i-1])/p[i];
	}

	x[n-1] = y[n-1];

	for(int i=n-2;i>=0;i--)
	{
		x[i] = y[i] - q[i]*x[i+1];
	}

	free(p);
	free(y);
	free(q);

	return 0;
}

double circleAngle(double startPoint[3],double middlePoint[3],double entPoint[3])
{
	double nV[3];

	return circleVector(nV,startPoint,middlePoint,entPoint);
}

double circleVector(double normVector[3],double startPoint[3],double middlePoint[3],double endPoint[3])
{

	double n0[3],AB[3],BC[3],cP[3];

	// center point
	double radius = circleCenter(cP,startPoint,middlePoint,endPoint);
	if (radius <= 0)
	{
		DUMP_ERROR("ERR: 3 points are same in <circleVector>!\n");
		return 0;
	}

	// deal with the situation that startP and endP are same
	if (dis3(startPoint,endPoint) <= ALMOST_ZERO)
	{
		if (dis3(startPoint,middlePoint) <= ALMOST_ZERO)
		{
			DUMP_ERROR("ERR: 3 points are same in <circleVector>!\n");
			return 0;
		}

		DUMP_WARNING("WARNING: start point and end point are same in <circleVector>!\n");
		normVector[0] = 0; normVector[1] = 0; normVector[2] = 1.0;
		return 2.0*PI;
	}

	for(int i=0;i<3;i++)
	{
		AB[i] = startPoint[i]-middlePoint[i];
		BC[i] = middlePoint[i]-endPoint[i];
	}

	double ca = dot3(AB,BC)/(norm3(AB)*norm3(BC));
	if (ca > 1.0)
	{
		ca = 1.0;
	}
	if (ca < -1.0)
	{
		ca = -1.0;
	}

	double angle = acos(ca)*2.0;

	// norm vector
	cross3(n0,AB,BC);

	double tmp = norm3(n0);
	if (tmp < ALMOST_ZERO)
	{
		DUMP_ERROR("ERR: the normal vector is invalid in <circleVector>\n");
		return 0;
	}
	else
	{
		normVector[0] = n0[0] / tmp;
		normVector[1] = n0[1] / tmp;
		normVector[2] = n0[2] / tmp;
	}

	return angle;
}

double circleCenter(double centerPoint[3],double startPoint[3],double middlePoint[3],double endPoint[3])
{
	// deal with the situation that startP and endP are same
	if (dis3(startPoint,endPoint) <= ALMOST_ZERO)
	{
		if (dis3(startPoint,middlePoint) <= ALMOST_ZERO)
		{
			DUMP_ERROR("ERR: 3 points are same in <circleCenter>!\n");
			return 0;
		}

		DUMP_WARNING("WARNING: start point and end point are same in <circleCenter>!\n");
		for (int i=0; i<3; i++) // two points for a diameter
		{
			centerPoint[i] = (startPoint[i] + middlePoint[i])/2.0;
		}

		return dis3(startPoint,centerPoint);
	}

	for (int i = 0; i < 3; i++)
	{
		// exchange x y z axis
		double dPx=startPoint[i%3];
		double dQx=middlePoint[i%3];
		double dRx=endPoint[i%3];

		double dPy=startPoint[(i+1)%3];
		double dQy=middlePoint[(i+1)%3];
		double dRy=endPoint[(i+1)%3];

		double dPz=startPoint[(i+2)%3];
		double dQz=middlePoint[(i+2)%3];
		double dRz=endPoint[(i+2)%3];

		// get normal vector
		double  x1=dQx-dPx;
		double  x2=dRx-dPx;

		double  y1=dQy-dPy;
		double  y2=dRy-dPy;

		double  z1=dQz-dPz;
		double  z2=dRz-dPz;

		double  pi=y1*z2-z1*y2;
		double  pj=z1*x2-x1*z2;
		double  pk=x1*y2-y1*x2;

		if ((pi==0) && (pj==0) && (pk==0))
		{
			DUMP_WARNING("WARNING: 3 points collinear in <circleCenter>!\n");

			for (int j=0; j<3; j++)
			{
				centerPoint[j] = (startPoint[j] + endPoint[i])/2.0;
			}

			return dis3(startPoint,centerPoint);
		}

		// middle point of PQ
		double dMx=(dPx+dQx)/2;
		double dMy=(dPy+dQy)/2;
		double dMz=(dPz+dQz)/2;

		//[MI MJ MK]=[PI,PJ,PK]x[x1,y1,z1]
		double dMi=pj*z1-pk*y1;
		double dMj=pk*x1-pi*z1;
		double dMk=pi*y1-pj*x1;

		// middle point of PR
		double  dNx=(dPx+dRx)/2;
		double  dNy=(dPy+dRy)/2;
		double  dNz=(dPz+dRz)/2;

		//[NI NJ NK]=[PI PJ PK]x[x2 y2 z2]
		double  dNi=pj*z2-pk*y2;
		double  dNj=pk*x2-pi*z2;
		double  dNk=pi*y2-pj*x2;    

		if ((dNj*dMi-dMj*dNi) == 0)
		{
			// exchange x y z axises
			continue;
		}

		double tn=((dMy-dNy)*dMi+dMj*(dNx-dMx))/(dNj*dMi-dMj*dNi);
		centerPoint[i%3]=dNx+dNi*tn;
		centerPoint[(i+1)%3]=dNy+dNj*tn;
		centerPoint[(i+2)%3]=dNz+dNk*tn;

		break;
	}

	return dis3(startPoint,centerPoint);
}

double distanceOf2LinesIn3D(double cp0[3], double cp1[3], double p0[3], double  v0[3], double  p1[3], double v1[3])
{
	if (norm3(v0) < ALMOST_ZERO && norm3(v1) < ALMOST_ZERO)
	{
		for (int i=0; i<3; i++)
		{
			cp0[i] = 0.5*p0[i] + 0.5*p1[i];
		}
		return dis3(p0,p1);
	}

	double nV0[3], nV1[3];
	for (int i=0; i<3; i++)
	{
		nV0[i] = v0[i];///norm3(v0);
		nV1[i] = v1[i];///norm3(v1);
	}

	double d0_d1[3] = {0.0}, e_d0[3] = {0.0}, e_d1[3] = {0.0};
	double deltaP[3] = {p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2]};

	cross3(d0_d1,nV0,nV1);	
	cross3(e_d0,deltaP,nV0);
	cross3(e_d1,deltaP,nV1);

	double dd = norm3(d0_d1);
	if (dd == 0)
	{
		for (int i=0; i<3; i++)
		{
			cp1[i] = p1[i] + 0.5*dot3(deltaP,nV1)*nV1[i];
			cp0[i] = p0[i] + 0.5*dot3(deltaP,nV0)*nV0[i];
		}
		return sqrt(norm3(deltaP)*norm3(deltaP) - dot3(deltaP,nV0)*dot3(deltaP,nV0));
	}

	double t0 = dot3(e_d1,d0_d1)/(dd*dd);
	double t1 = dot3(e_d0,d0_d1)/(dd*dd);

	for (int i=0; i<3; i++)
	{
		cp0[i] = p0[i] + nV0[i]*t0;
		cp1[i] = p1[i] + nV1[i]*t1;
	}

	return dis3(cp0,cp1);
}





void Trp3(double R[3][3], double R1[3][3])
{
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			R[j][i] = R1[i][j];
		}
	}
}

void M3p3(double V[3], double M[3][3], double V2[3])
{
    for (int i=0; i<3; i++)
    {
		V[i] = 0;
		for (int j=0; j<3; j++)
		{
			V[i] += M[i][j] * V2[j];
		}
    }
}

void M3p3(double V[3], double V2[3], double M[3][3])
{
	for (int i=0; i<3; i++)
	{
		V[i] = 0;
		for (int j=0; j<3; j++)
		{
			V[i] += V2[j] * M[j][i];
		}
	}
}

void M4p4(double V[4], double M[4][4], double V2[4])
{
	for (int i=0; i<4; i++)
	{
		V[i] = 0;
		for (int j=0; j<4; j++)
		{
			V[i] += M[i][j] * V2[j];
		}
	}
}

void M4p4(double V[4], double V2[4], double M[4][4])
{
	for (int i=0; i<4; i++)
	{
		V[i] = 0;
		for (int j=0; j<4; j++)
		{
			V[i] += V2[j] * M[j][i];
		}
	}
}

void M6p6(double V[6], double M[6][6], double V2[6])
{
	for (int i=0; i<6; i++)
	{
		V[i] = 0;
		for (int j=0; j<6; j++)
		{
			V[i] += M[i][j] * V2[j];
		}
	}
}

void M6p6(double V[6], double V2[6], double M[6][6])
{
	for (int i=0; i<6; i++)
	{
		V[i] = 0;
		for (int j=0; j<6; j++)
		{
			V[i] += V2[j] * M[j][i];
		}
	}
}

void M3p3(double M[3][3], double M1[3][3], double M2[3][3])
{
    for(int i = 0; i < 3; i++)
	{
        for(int j = 0; j < 3; j++)
		{
			M[i][j] = 0;
            for(int k = 0; k < 3; k++)
			{
                M[i][j] += M1[i][k] * M2[k][j];
            }
        }
    }
}

void M4p4(double M[4][4], double M1[4][4], double M2[4][4])
{
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			M[i][j] = 0;
			for(int k = 0; k < 4; k++)
			{
				M[i][j] += M1[i][k] * M2[k][j];
			}
		}
	}
}

void M6p6(double M[6][6], double M1[6][6], double M2[6][6])
{
	for(int i = 0; i < 6; i++)
	{
		for(int j = 0; j < 6; j++)
		{
			M[i][j] = 0;
			for(int k = 0; k < 6; k++)
			{
				M[i][j] += M1[i][k] * M2[k][j];
			}
		}
	}
}

void M6pN(double V[6], double M[6][MAX_DOF], double V2[MAX_DOF], int N)
{
	for (int i=0; i<6; i++)
	{
		V[i] = 0;
		for (int j=0; j<N; j++)
		{
			V[i] += M[i][j] * V2[j];
		}
	}
}

void M6pN(double M[6][MAX_DOF], double M1[6][6], double M2[6][MAX_DOF], int N)
{
	for(int i = 0; i < 6; i++)
	{
		for(int j = 0; j < N; j++)
		{
			M[i][j] = 0;
			for(int k = 0; k < 6; k++)
			{
				M[i][j] += M1[i][k] * M2[k][j];
			}
		}
	}
}



void rot_x(double R[4][4], double t)
{
	R[0][0] = 1.0;	R[0][1] = 0.0;		R[0][2] = 0.0;		R[0][3] = 0.0;
	R[1][0] = 0.0;	R[1][1] = cos(t);	R[1][2] = -sin(t);	R[1][3] = 0.0;
	R[2][0] = 0.0;	R[2][1] = sin(t);	R[2][2] = cos(t);	R[2][3] = 0.0;
	R[3][0] = 0.0;	R[3][1] = 0.0;		R[3][2] = 0.0;		R[3][3] = 1.0;
}

void rot_y(double R[4][4], double t)
{
	R[0][0] = cos(t);	R[0][1] = 0.0;	R[0][2] = sin(t);	R[0][3] = 0.0;
	R[1][0] = 0.0;		R[1][1] = 1.0;	R[1][2] = 0.0;		R[1][3] = 0.0;
	R[2][0] = -sin(t);	R[2][1] = 0.0;	R[2][2] = cos(t);	R[2][3] = 0.0;
	R[3][0] = 0.0;		R[3][1] = 0.0;	R[3][2] = 0.0;		R[3][3] = 1.0;
}

void rot_z(double R[4][4], double t)
{
	R[0][0] = cos(t);	R[0][1] = -sin(t);	R[0][2] = 0.0;	R[0][3] = 0.0;
	R[1][0] = sin(t);	R[1][1] = cos(t);	R[1][2] = 0.0;	R[1][3] = 0.0;
	R[2][0] = 0.0;		R[2][1] = 0.0;		R[2][2] = 1.0;	R[2][3] = 0.0;
	R[3][0] = 0.0;		R[3][1] = 0.0;		R[3][2] = 0.0;	R[3][3] = 1.0;
}

void vex2skew(double skew[3][3], double p[3])
{
	skew[0][0] = 0;		skew[0][1] = -p[2];	skew[0][2] = p[1];
	skew[1][0] = p[2];	skew[1][1] = 0;		skew[1][2] = -p[0];
	skew[2][0] = -p[1];	skew[2][1] = p[0];	skew[2][2] = 0;
}

void skew2vex(double p[3], double skew[3][3])
{
	p[0] =0.5*(skew[2][1] - skew[1][2]);
	p[1] = 0.5*(skew[0][2] - skew[2][0]);
	p[2] = 0.5*(skew[1][0] - skew[0][1]);
}

void matrix2quaternion(double* q,double R[3][3])
{
	double Qxx = R[0][0];
	double Qxy = R[0][1];
	double Qxz = R[0][2];

	double Qyx = R[1][0];
	double Qyy = R[1][1];
	double Qyz = R[1][2];

	double Qzx = R[2][0];
	double Qzy = R[2][1];
	double Qzz = R[2][2];

	double w = 0.0;
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	double r = 0.0;
	double s = 0.0;

	double t = Qxx + Qyy + Qzz;
	double maxv = MAX(Qxx, MAX(Qyy, Qzz));

	if (t >= 0)
	{
		r = sqrt(1 + t);
		s = 0.5 / r;
		w = 0.5*r;
		x = (Qzy - Qyz)*s;
		y = (Qxz - Qzx)*s;
		z = (Qyx - Qxy)*s;
	}
	else
	{
		if (maxv == Qxx)
		{
			r = sqrt(1 + Qxx - Qyy - Qzz);
			s = 0.5 / r;
			w = (Qzy - Qyz)*s;
			x = 0.5*r;
			y = (Qyx + Qxy)*s;
			z = (Qxz + Qzx)*s;
		}
		else
		{
			if (maxv == Qyy)
			{
				r = sqrt(1 + Qyy - Qxx - Qzz);
				s = 0.5 / r;
				w = (Qxz - Qzx)*s;
				x = (Qyx + Qxy)*s;
				y = 0.5*r;
				z = (Qzy + Qyz)*s;
			}
			else
			{
				r = sqrt(1 + Qzz - Qxx - Qyy);
				s = 0.5 / r;
				w = (Qyx - Qxy)*s;
				x = (Qxz + Qzx)*s;
				y = (Qzy + Qyz)*s;
				z = 0.5*r;
			}
		}
	}

	q[0] = w;
	q[1] = x;
	q[2] = y;
	q[3] = z;
}

void quaternion2matrix(double R[3][3], double*q)
{
	double q0 = q[0];
	double qx = q[1];
	double qy = q[2];
	double qz = q[3];

	R[0][0] = q0*q0 + qx*qx - qy*qy - qz*qz;
	R[0][1] = 2 * qx*qy - 2 * q0*qz;
	R[0][2] = 2 * qx*qz + 2 * q0*qy;

	R[1][0] = 2 * qx*qy + 2 * q0*qz;
	R[1][1] = q0*q0 - qx*qx + qy*qy - qz*qz;
	R[1][2] = 2 * qy*qz - 2 * q0*qx;

	R[2][0] = 2 * qx*qz - 2 * q0*qy;
	R[2][1] = 2 * qy*qz + 2 * q0*qx;
	R[2][2] = q0*q0 - qx*qx - qy*qy + qz*qz;
}

void pose2homogeneous(double T[4][4], double X[6])
{
	double R[3][3];
	rpy2matrix(R, X + 3);
	rot2homogeneous(T, R);

	for (int i = 0; i < 3; i++)
	{
		T[i][3] = X[i];
	}
}

void homogeneous2pose(double X[6], double T[4][4])
{
	double R[3][3];
	homogeneous2rot(R, T);
	matrix2rpy(X + 3, R);
	homogeneous2trans(X, T);
}

void trans2homogeneous(double T[4][4],double X[3])
{
	for (int i=0; i<3; i++)
	{
		T[i][0] = 0;
		T[i][1] = 0;
		T[i][2] = 0;
		T[i][3] = X[i];
		T[i][i] = 1;
	}
	T[3][3] = 1;
}

void homogeneous2trans(double X[3], double T[4][4])
{
	for (int i = 0; i < 3; i++)
	{
		X[i] = T[i][3];
	}
}

void rot2homogeneous(double T[4][4], double R[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			T[i][j] = R[i][j];
		}
		T[i][3] = 0;
	}

	T[3][0] = 0.0;
	T[3][1] = 0.0;
	T[3][2] = 0.0;
	T[3][3] = 1.0;
}

void homogeneous2rot(double R[3][3], double T[4][4])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R[i][j] = T[i][j];
		}
	}
}

int matrix2rot(double* k, double* t,double R[3][3])
{
	double nx = R[0][0], ny = R[1][0], nz = R[2][0];
	double ox = R[0][1], oy = R[1][1], oz = R[2][1];
	double ax = R[0][2], ay = R[1][2], az = R[2][2];
	double cosT = (nx + oy + az - 1.0) / 2.0;
	if (ABS(cosT) > 1.0)
	{
		cosT = sign(cosT)*1.0;
	}
	*t = acos(cosT);

	// small rotation, physically ill
	if (ABS(*t) < 1E-2)
	{
		k[0] = 0, k[1] = 0, k[2] = 1.0;
		k[ROT_INDEX - 3] = 1.0;
		return 0;
	}

	// poorly accuracy when theta approaches PI, so use other method when theta > 90 deg
	if (ABS(*t) > 0.6*PI)
	{
		k[0] = sign(oz - ay)*sqrt((nx - cosT) / (1.0 - cosT));
		k[1] = sign(ax - nz)*sqrt((oy - cosT) / (1.0 - cosT));
		k[2] = sign(ny - ox)*sqrt((az - cosT) / (1.0 - cosT));

		if (ABS(k[0]) >= ABS(k[1]) && ABS(k[0]) >= ABS(k[2]))
		{
			k[1] = (ny + ox) / (2.0*k[0] * (1.0 - cosT));
			k[2] = (ax + nz) / (2.0*k[0] * (1.0 - cosT));
		}
		else if(ABS(k[1]) >= ABS(k[0]) && ABS(k[1]) >= ABS(k[2]))
		{
			k[0] = (ny + ox) / (2.0*k[1] * (1.0 - cosT));
			k[2] = (oz + ay) / (2.0*k[1] * (1.0 - cosT));
		}
		else if (ABS(k[2]) >= ABS(k[1]) && ABS(k[2]) >= ABS(k[0]))
		{
			k[0] = (ax + nz) / (2.0*k[2] * (1.0 - cosT));
			k[1] = (oz + ay) / (2.0*k[2] * (1.0 - cosT));
		}
		return 0;
	}

	// normal condition, use Rodrigues' rotation formula
	double tmp[3];
	tmp[0] = 0.5*(R[2][1] - R[1][2]) / sin(*t);
	tmp[1] = 0.5*(R[0][2] - R[2][0]) / sin(*t);
	tmp[2] = 0.5*(R[1][0] - R[0][1]) / sin(*t);

	if (k != nullptr)
	{
		double d1 = (tmp[0] - k[0])*(tmp[0] - k[0]) + (tmp[1] - k[1])*(tmp[1] - k[1]) + (tmp[2] - k[2])*(tmp[2] - k[2]);
		double d2 = (tmp[0] + k[0])*(tmp[0] + k[0]) + (tmp[1] + k[1])*(tmp[1] + k[1]) + (tmp[2] + k[2])*(tmp[2] + k[2]);
		if (d1 > d2)
		{
			k[0] = -tmp[0];
			k[1] = -tmp[1];
			k[2] = -tmp[2];

			*t = -(*t);
		}
		else
		{
			k[0] = tmp[0];
			k[1] = tmp[1];
			k[2] = tmp[2];
		}
	}
	else
	{
		k[0] = tmp[0];
		k[1] = tmp[1];
		k[2] = tmp[2];
	}
	
	return 0;
}

int rot2matrix(double R[3][3], double* k, double* t)
{
	if (t==nullptr)
	{
		*t = sqrt(k[0]*k[0] + k[1]*k[1] + k[2]*k[2]);
		if(*t==0)
		{
			DUMP_ERROR("Error in <rot2matrix> for invalid rotation axis 'k'(zero axis vector).\n");
			return -1;
		}
		k[0] = k[0]/(*t);
		k[1] = k[1]/(*t);
		k[2] = k[2]/(*t);
	}
	else if(  ABS(sqrt(k[0]*k[0] + k[1]*k[1] + k[2]*k[2]) - 1.0) > ALMOST_ZERO)
	{
		DUMP_ERROR("Error in <rot2matrix> for invalid rotation axis 'k'.\n");
		return -1;
	}

	double v = 1 - cos(*t);

	R[0][0] = k[0]*k[0]*v + cos(*t);		R[0][1] = k[0]*k[1]*v - k[2]*sin(*t);	R[0][2] = k[0]*k[2]*v + k[1]*sin(*t);
	R[1][0] = k[0]*k[1]*v + k[2]*sin(*t);	R[1][1] = k[1]*k[1]*v + cos(*t);		R[1][2] = k[1]*k[2]*v - k[0]*sin(*t);
	R[2][0] = k[0]*k[2]*v - k[1]*sin(*t);	R[2][1] = k[1]*k[2]*v + k[0]*sin(*t);	R[2][2] = k[2]*k[2]*v + cos(*t);

	return 0;
}

void matrix2rpy(double rpy[],double R[3][3],int type)
{
	// rpy = [roll, pitch, yaw]

	double cos_beta = sqrt(R[0][0]*R[0][0] + R[1][0]*R[1][0]);

	switch (type)
	{
	case 0:/*	0, ZYX order, R = rotz(yaw) * roty(pitch) * rotx(roll),
		   /*	       equals R = rotx(roll) * roty(pitch) * rotz(yaw);*/
		if (ABS(cos_beta) > 1e-3)
		{
			rpy[0] = atan2(R[2][1],R[2][2]);
			rpy[1] = atan2(-R[2][0],cos_beta);
			rpy[2] = atan2(R[1][0],R[0][0]);
		}
		else
		{
			rpy[0] = atan2(R[0][1],R[1][1]);
			rpy[1] = PI/2.0;
			rpy[2] = 0.0;
		}
		break;

	case 1:/*1, XYZ order, R = rotx(yaw) * roty(pitch) * rotz(roll);*/
		if (ABS(ABS(R[0][2])-1.0) < ACCURACY_FACTOR)
		{
			//singularity
			rpy[0] = 0;//roll is zero
			if (R[0][2] > 0)
			{
				rpy[2] = atan2(R[2][1],R[1][1]); // R+Y
			}
			else
			{
				rpy[2] = -atan2(R[1][0],R[2][0]);// R-Y
			}
			rpy[1] = asin(R[0][2]);
		}
		else
		{
			rpy[0] = -atan2(R[0][1],R[0][0]);
			rpy[2] = -atan2(R[1][2],R[2][2]);

			rpy[1] = atan(R[0][2]*cos(rpy[0])/R[0][0]);
		}
		break;

	default:
		break;
	}
}

void rpy2matrix(double R[3][3],double rpy[],int type)
{
	double roll = rpy[0], pitch = rpy[1], yaw = rpy[2];

	double ret1[4][4] = {0};
	double ret2[4][4] = {0};
	double ret3[4][4] = {0};

	ret1[0][0] = 1.0;
	ret1[1][1] = 1.0;
	ret1[2][2] = 1.0;
	ret1[3][3] = 1.0;

	switch(type)
	{
	case 0:// XYZ order, rotx(roll)*roty(pitch)*rotz(yaw)
		R[0][0] = cos(yaw)*cos(pitch);	
		R[0][1] = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll); 
		R[0][2] = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll);

		R[1][0] = sin(yaw)*cos(pitch);
		R[1][1] = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll); 
		R[1][2] = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);

		R[2][0] = -sin(pitch);
		R[2][1] = cos(pitch)*sin(roll);				
		R[2][2] = cos(pitch)*cos(roll);
		break;

	case 1://XYZ order, rotx(yaw)*roty(pitch)*rotz(roll)
		R[0][0] = cos(pitch)*cos(roll);
		R[0][1] =  -cos(pitch)*sin(roll);
		R[0][2] = sin(pitch);

		R[1][0] = cos(yaw)*sin(roll) + cos(roll)*sin(pitch)*sin(yaw); 
		R[1][1] = cos(roll)*cos(yaw) - sin(pitch)*sin(roll)*sin(yaw);  
		R[1][2] = -cos(pitch)*sin(yaw);

		R[2][0] = sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch); 
		R[2][1] = cos(roll)*sin(yaw) + cos(yaw)*sin(pitch)*sin(roll);  
		R[2][2] = cos(pitch)*cos(yaw);
		break;

	default:
		break;
	}
}

void tr2delta(double delta[6], double T1[4][4], double T[4][4])
{
	delta[0] = T1[0][3] - T[0][3]; 
	delta[1] = T1[1][3] - T[1][3]; 
	delta[2] = T1[2][3] - T[2][3];

	delta[3] = (T1[2][0] - T[2][0])*(T[1][0]) + (T1[2][1] - T[2][1])*(T[1][1]) + (T1[2][2] - T[2][2])*(T[1][2]);
	delta[4] = (T1[0][0] - T[0][0])*(T[2][0]) + (T1[0][1] - T[0][1])*(T[2][1]) + (T1[0][2] - T[0][2])*(T[2][2]);
	delta[5] = (T1[1][0] - T[1][0])*(T[0][0]) + (T1[1][1] - T[1][1])*(T[0][1]) + (T1[1][2] - T[1][2])*(T[0][2]);

}




















#define MAX_ITERA		(500)
#define MIN_DOUBLE		(1e-20)

void   sss(double fg[2],double cs[2]);
void   damul(double a[],double b[],int m,int n,int k,double c[]);
void   ppp(double a[],double e[],double s[],double v[],int m,int n);
double norm(double a[],int m,int n,double u[],double v[],double eps,int ka);
int    dluav(double a[],int m,int n,double u[],double v[],double eps,int ka);


void   sss(double fg[2],double cs[2])
{
	double r,d;
	
	if((fabs(fg[0])+fabs(fg[1]))==0.0)//if((fabs(fg[0])+fabs(fg[1]))<MIN_DOUBLE)
	{
		cs[0]=1.0;cs[1]=0.0;d=0.0;
	}
	else
	{
		d=sqrt(fg[0]*fg[0]+fg[1]*fg[1]);

		if(fabs(fg[0])>fabs(fg[1]))
		{
			d=fabs(d);

			if(fg[0]<0.0)
			{
				d=-d;
			}
		}

		if(fabs(fg[1])>=fabs(fg[0]))
		{
			d=fabs(d);

			if(fg[1]<0.0)
			{
				d=-d;
			}
		}

		cs[0]=fg[0]/d;
		cs[1]=fg[1]/d;
	}

	r=1.0;

	if(fabs(fg[0])>fabs(fg[1]))
	{
		r=cs[1];
	}
	else
	{
		if(cs[0]!=0.0)//if(fabs(cs[0])>MIN_DOUBLE)
		{
			r=1.0/cs[0];
		}
	}

	fg[0]=d;
	fg[1]=r;
	return;
}

int    pinv(double* pinv_a, double* a, int m, int n)
{
	int i,j,err;

	double* u = new double[m*m];
	double* v = new double[n*n];
	double* c = new double[m*n];
	double* d = new double[m*n];

	for(i=0;i<m*m;i++)
	{
		u[i]=0;
	}

	err = dluav(a,m,n,u,v,ALMOST_ZERO,m+1);

	double* vT = new double[n*n];
	double* uT = new double[m*m];
	double* pinvW = new double[n*m];

	for (i=0; i<n; i++)
	{
		for (j=0; j<m; j++)
		{
			if(a[j*n+i] != 0.0)//if (fabs(a[j*n+i])>MIN_DOUBLE)
			{
				pinvW[i*m + j] = 1.0 / a[j*n + i];
			}
			else
			{
				pinvW[i*m + j] = 0;
			}
		}
	}

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			vT[i*n + j] = v[j*n + i];
		}
	}

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < m; j++)
		{
			uT[i*m + j] = u[j*m + i];
		}
	}

	damul(vT,pinvW,n,n,m,c);
	damul(c,uT,n,m,m,pinv_a);

	delete[] u, v, c, d, pinvW, vT, uT;

	return err;
}

void   damul(double a[],double b[],int m,int n,int k,double c[])
{
	int i,j,l,u;
	for(i=0;i<=m-1;i++)
	{
		for(j=0;j<=k-1;j++)
		{
			u=i*k+j;

			c[u]=0;

			for(l=0;l<=n-1;l++)
			{
				c[u]=c[u]+a[i*n+l]*b[l*k+j];
			}
		}
	}
}

void   ppp(double a[],double e[],double s[],double v[],int m,int n)
{
	double d;
	int i,j,p,q;

	if(m>=n)
	{
		i=n;
	}
	else
	{
		i=m;
	}

	for(j=1;j<=i-1;j++)
	{
		a[(j-1)*n+j-1]=s[j-1];
		a[(j-1)*n+j]=e[j-1];
	}

	a[(i-1)*n+i-1]=s[i-1];

	if(m<n)
	{
		a[(i-1)*n+i]=e[i-1];
	}

	for(i=1;i<=n-1;i++)
	{
		for(j=i+1;j<=n;j++)
		{
			p=(i-1)*n+j-1;
			q=(j-1)*n+i-1;

			d=v[p];
			v[p]=v[q];
			v[q]=d;
		}
	}
}

double norm(double a[],int m,int n,double u[],double v[],double eps,int ka)
{
	dluav(a,m,n,u,v,eps,ka);
	return a[0];
}

int    dluav(double a[],int m,int n,double u[],double v[],double eps,int ka)
{
/*********************************************************************
 * 矩阵的奇异值分解，参见《c 常用算法程序集》徐世良P169
 * 参数说明：
 * a m*n的实矩阵，返回时其对角线给出奇异值（非递增顺序），其余元素为0
 * m,n 矩阵A的行数和列数
 * u m*m的矩阵，存放左奇异向量
 * v n*n的矩阵，存放右奇异向量
 * eps 双精度实型变量，给定精度要求
 * ka 整形变量，其值为max(m,n)+1
 * 返回值：如果返回标志小于0，则说明出现了迭代MAX_ITERA次还未求得某个
 * 奇异值的情况，此时矩阵A的分解式为UAV，如果返回标志大于0，则说明
 * 程序正常运行
 ********************************************************************/

	double *s=(double*)malloc(ka*sizeof(double));
	double *e=(double*)malloc(ka*sizeof(double));
	double *w=(double*)malloc(ka*sizeof(double));

	int i,j,k,l,it,ll,kk,ix,iy,mm,nn,iz,ml,ks;
	double d,dd,t,sm,sml,eml,sk,ek,b,c,shh,fg[2],cs[2];

	for(i=1;i<=m;i++)
	{
		ix=(i-1)*m+i-1;
		u[ix]=0;
	}

	for(i=1;i<=n;i++)
	{
		iy=(i-1)*n+i-1;
		v[iy]=0;
	}

	k=n;
	it=MAX_ITERA;

	if(m-1<n)
	{
		k=m-1;
	}

	l=m;

	if(n-2<m)
	{
		l=n-2;
	}

	if(l<0)
	{
		l=0;
	}

	ll=k;

	if(l>k)
	{
		ll=l;
	}

	if(ll>=1)
	{
		for(kk=1;kk<=ll;kk++)
		{
			if(kk<=k)
			{
				d=0.0;

				for(i=kk;i<=m;i++)
				{
					ix=(i-1)*n+kk-1;
					d=d+a[ix]*a[ix];
				}

				s[kk-1]=sqrt(d);

				if(s[kk-1]!=0.0)//if(fabs(s[kk-1])>MIN_DOUBLE)//
				{
					ix=(kk-1)*n+kk-1;
					
					if(a[ix]!=0.0)//if(fabs(a[ix])>MIN_DOUBLE)//
					{
						s[kk-1]=fabs(s[kk-1]);

						if(a[ix]<0.0)
						{
							s[kk-1]=-s[kk-1];
						}
					}

					for(i=kk;i<=m;i++)
					{
						iy=(i-1)*n+kk-1;
						a[iy]=a[iy]/s[kk-1];
					}

					a[ix]=1.0+a[ix];
				}

				s[kk-1]=-s[kk-1];
			}

			if(n>=kk+1)
			{
				for(j=kk+1;j<=n;j++)
				{
					if((kk<=k)&&(s[kk-1]!=0.0))//if((kk<=k)&&(fabs(s[kk-1])>MIN_DOUBLE))//
					{
						d=0.0;

						for(i=kk;i<=m;i++)
						{
							ix=(i-1)*n+kk-1;
							iy=(i-1)*n+j-1;
							d=d+a[ix]*a[iy];
						}

						d=-d/a[(kk-1)*n+kk-1];

						for(i=kk;i<=m;i++)
						{
							ix=(i-1)*n+j-1;
							iy=(i-1)*n+kk-1;
							a[ix]=a[ix]+d*a[iy];
						}
					}

					e[j-1]=a[(kk-1)*n+j-1];
				}
			}

			if(kk<=k)
			{
				for(i=kk;i<=m;i++)
				{
					ix=(i-1)*m+kk-1;
					iy=(i-1)*n+kk-1;
					u[ix]=a[iy];
				}
			}

			if(kk<=l)
			{
				d=0.0;

				for(i=kk+1;i<=n;i++)
				{
					d=d+e[i-1]*e[i-1];
				}

				e[kk-1]=sqrt(d);
				
				if(e[kk-1]!=0.0)//if(fabs(e[kk-1])>MIN_DOUBLE)//
				{
					
					if(e[kk]!=0.0)//if(fabs(e[kk])>MIN_DOUBLE)//
					{
						e[kk-1]=fabs(e[kk-1]);

						if(e[kk]<0.0)
						{
							e[kk-1]=-e[kk-1];
						}
					}

					for(i=kk+1;i<=n;i++)
					{
						e[i-1]=e[i-1]/e[kk-1];
					}

					e[kk]=1.0+e[kk];
				}

				e[kk-1]=-e[kk-1];
				
				if((kk+1<=m)&&(e[kk-1]!=0.0))//if((kk+1<=m)&&(fabs(e[kk-1])>MIN_DOUBLE))//
				{
					for(i=kk+1;i<=m;i++)
					{
						w[i-1]=0.0;
					}

					for(j=kk+1;j<=n;j++)
					{
						for(i=kk+1;i<=m;i++)
						{
							w[i-1]=w[i-1]+e[j-1]*a[(i-1)*n+j-1];
						}
					}

					for(j=kk+1;j<=n;j++)
					{
						for(i=kk+1;i<=m;i++)
						{
							ix=(i-1)*n+j-1;
							a[ix]=a[ix]-w[i-1]*e[j-1]/e[kk];
						}
					}
				}

				for(i=kk+1;i<=n;i++)
				{
					v[(i-1)*n+kk-1]=e[i-1];
				}
			}
		}
	}

	mm=n;

	if(m+1<n)
	{
		mm=m+1;
	}

	if(k<n)
	{
		s[k]=a[k*n+k];
	}

	if(m<mm)
	{
		s[mm-1]=0.0;
	}

	if(l+1<mm)
	{
		e[l]=a[l*n+mm-1];
	}

	e[mm-1]=0.0;

	nn=m;

	if(m>n)
	{
		nn=n;
	}

	if(nn>=k+1)
	{
		for(j=k+1;j<=nn;j++)
		{
			for(i=1;i<=m;i++)
			{
				u[(i-1)*m+j-1]=0.0;
			}

			u[(j-1)*m+j-1]=1.0;
		}
	}

	if(k>=1)
	{
		for(ll=1;ll<=k;ll++)
		{
			kk=k-ll+1;
			iz=(kk-1)*m+kk-1;
			
			if(s[kk-1]!=0.0)//if(fabs(s[kk-1])>MIN_DOUBLE)//
			{
				if(nn>=kk+1)
				{
					for(j=kk+1;j<=nn;j++)
					{
						d=0.0;

						for(i=kk;i<=m;i++)
						{
							ix=(i-1)*m+kk-1;
							iy=(i-1)*m+j-1;
							d=d+u[ix]*u[iy]/u[iz];
						}

						d=-d;

						for(i=kk;i<=m;i++)
						{
							ix=(i-1)*m+j-1;
							iy=(i-1)*m+kk-1;
							u[ix]=u[ix]+d*u[iy];
						}
					}
				}

				for(i=kk;i<=m;i++)
				{
					ix=(i-1)*m+kk-1;
					u[ix]=-u[ix];
				}

				u[iz]=1.0+u[iz];

				if(kk-1>=1)
				{
					for(i=1;i<=kk-1;i++)
					{
						u[(i-1)*m+kk-1]=0.0;
					}
				}
			}
			else
			{
				for(i=1;i<=m;i++)
				{
					u[(i-1)*m+kk-1]=0.0;
				}

				u[(kk-1)*m+kk-1]=1.0;
			}
		}
	}

	for(ll=1;ll<=n;ll++)
	{
		kk=n-ll+1;
		iz=kk*n+kk-1;
		
		if((kk<=l)&&(e[kk-1]!=0.0))//if((kk<=l)&&(fabs(e[kk-1])>MIN_DOUBLE))//
		{
			for(j=kk+1;j<=n;j++)
			{
				d=0.0;

				for(i=kk+1;i<=n;i++)
				{
					ix=(i-1)*n+kk-1;
					iy=(i-1)*n+j-1;
					d=d+v[ix]*v[iy]/v[iz];
				}

				d=-d;

				for(i=kk+1;i<=n;i++)
				{
					ix=(i-1)*n+j-1;
					iy=(i-1)*n+kk-1;
					v[ix]=v[ix]+d*v[iy];
				}
			}
		}

		for(i=1;i<=n;i++)
		{
			v[(i-1)*n+kk-1]=0.0;
		}

		v[iz-n]=1.0;
	}

	for(i=1;i<=m;i++)
	{
		for(j=1;j<=n;j++)
		{
			a[(i-1)*n+j-1]=0.0;
		}
	}

	ml=mm;

	it=MAX_ITERA;

	while(1==1)
	{
		if(mm==0)
		{
			ppp(a,e,s,v,m,n);

			free(s);
			free(e);
			free(w);

			return l;
		}

		if(it==0)
		{
			ppp(a,e,s,v,m,n);

			free(s);
			free(e);
			free(w);

			return -1;
		}

		kk=mm-1;
		
		while((kk!=0)&&(fabs(e[kk-1])!=0.0))//while((kk!=0)&&(fabs(e[kk-1])>MIN_DOUBLE))//
		{
			d=fabs(s[kk-1])+fabs(s[kk]);

			dd=fabs(e[kk-1]);

			if(dd>eps*d)
			{
				kk=kk-1;
			}
			else
			{
				e[kk-1]=0.0;
			}
		}

		if(kk==mm-1)
		{
			kk=kk+1;

			if(s[kk-1]<0.0)
			{
				s[kk-1]=-s[kk-1];

				for(i=1;i<=n;i++)
				{
					ix=(i-1)*n+kk-1;
					v[ix]=-v[ix];
				}
			}

			while((kk!=ml)&&(s[kk-1]<s[kk]))
			{
				d=s[kk-1];
				s[kk-1]=s[kk];
				s[kk]=d;

				if(kk<n)
				{
					for(i=1;i<=n;i++)
					{
						ix=(i-1)*n+kk-1;
						iy=(i-1)*n+kk;
						d=v[ix];
						v[ix]=v[iy];
						v[iy]=d;
					}
				}

				if(kk<m)
				{
					for(i=1;i<=m;i++)
					{
						ix=(i-1)*m+kk-1;
						iy=(i-1)*m+kk;
						d=u[ix];
						u[ix]=u[iy];
						u[iy]=d;
					}
				}

				kk=kk+1;
			}

			it=MAX_ITERA;

			mm=mm-1;
		}
		else
		{
			ks=mm;
			
			while((ks>kk)&&(fabs(s[ks-1])!=0.0))//while((ks>kk)&&(fabs(s[ks-1])>MIN_DOUBLE))//
			{
				d=0.0;

				if(ks!=mm)
				{
					d=d+fabs(e[ks-1]);
				}

				if(ks!=kk+1)
				{
					d=d+fabs(e[ks-2]);
				}

				dd=fabs(s[ks-1]);

				if(dd>eps*d)
				{
					ks=ks-1;
				}
				else
				{
					s[ks-1]=0.0;
				}
			}

			if(ks==kk)
			{
				kk=kk+1;
				d=fabs(s[mm-1]);
				t=fabs(s[mm-2]);

				if(t>d)
				{
					d=t;
				}

				t=fabs(e[mm-2]);

				if(t>d)
				{
					d=t;
				}

				t=fabs(s[kk-1]);

				if(t>d)
				{
					d=t;
				}

				t=fabs(e[kk-1]);

				if(t>d)
				{
					d=t;
				}

				sm=s[mm-1]/d;
				sml=s[mm-2]/d;
				eml=e[mm-2]/d;
				sk=s[kk-1]/d;
				ek=e[kk-1]/d;
				b=((sml+sm)*(sml-sm)+eml*eml)/2.0;
				c=sm*eml;
				c=c*c;
				shh=0.0;
				
				if((b!=0.0)||(c!=0.0))//if((fabs(b)>MIN_DOUBLE)||(fabs(c)>MIN_DOUBLE))//
				{
					shh=sqrt(b*b+c);

					if(b<0.0)
					{
						shh=-shh;
					}

					shh=c/(b+shh);
				}

				fg[0]=(sk+sm)*(sk-sm)-shh;

				fg[1]=sk*ek;

				for(i=kk;i<=mm-1;i++)
				{
					sss(fg,cs);

					if(i!=kk)
					{
						e[i-2]=fg[0];
					}

					fg[0]=cs[0]*s[i-1]+cs[1]*e[i-1];
					e[i-1]=cs[0]*e[i-1]-cs[1]*s[i-1];
					fg[1]=cs[1]*s[i];
					s[i]=cs[0]*s[i];
					
					if((cs[0]!=1.0)||(cs[1]!=0.0))//if((fabs(cs[0]-1.0)>MIN_DOUBLE)||(fabs(cs[1])>MIN_DOUBLE))//
					{
						for(j=1;j<=n;j++)
						{
							ix=(j-1)*n+i-1;
							iy=(j-1)*n+i;
							d=cs[0]*v[ix]+cs[1]*v[iy];
							v[iy]=-cs[1]*v[ix]+cs[0]*v[iy];
							v[ix]=d;
						}
					}

					sss(fg,cs);

					s[i-1]=fg[0];
					fg[0]=cs[0]*e[i-1]+cs[1]*s[i];
					s[i]=-cs[1]*e[i-1]+cs[0]*s[i];
					fg[1]=cs[1]*e[i];
					e[i]=cs[0]*e[i];

					if(i<m)
					{
						if((cs[0]!=1.0)||(cs[1]!=0.0))//if((fabs(cs[0]-1.0)>MIN_DOUBLE)||(fabs(cs[1])>MIN_DOUBLE))//
						{
							for(j=1;j<=m;j++)
							{
								ix=(j-1)*m+i-1;
								iy=(j-1)*m+i;
								d=cs[0]*u[ix]+cs[1]*u[iy];
								u[iy]=-cs[1]*u[ix]+cs[0]*u[iy];
								u[ix]=d;
							}
						}
					}
				}

				e[mm-2]=fg[0];

				it=it-1;
			}
			else
			{
				if(ks==mm)
				{
					kk=kk+1;
					fg[1]=e[mm-2];e[mm-2]=0.0;

					for(ll=kk;ll<=mm-1;ll++)
					{
						i=mm+kk-ll-1;
						fg[0]=s[i-1];

						sss(fg,cs);

						s[i-1]=fg[0];

						if(i!=kk)
						{
							fg[1]=-cs[1]*e[i-2];
							e[i-2]=cs[0]*e[i-2];
						}
						
						if((cs[0]!=1.0)||(cs[1]!=0.0))//if((fabs(cs[0]-1.0)>MIN_DOUBLE)||(fabs(cs[1])>MIN_DOUBLE))//
						{
							for(j=1;j<=n;j++)
							{
								ix=(j-1)*n+i-1;
								iy=(j-1)*n+mm-1;
								d=cs[0]*v[ix]+cs[1]*v[iy];
								v[iy]=-cs[1]*v[ix]+cs[0]*v[iy];
								v[ix]=d;
							}
						}
					}
				}
				else
				{
					kk=ks+1;

					fg[1]=e[kk-2];

					e[kk-2]=0.0;

					for(i=kk;i<=mm;i++)
					{
						fg[0]=s[i-1];

						sss(fg,cs);

						s[i-1]=fg[0];
						fg[1]=-cs[1]*e[i-1];
						e[i-1]=cs[0]*e[i-1];
						
						if((cs[0]!=1.0)||(cs[1]!=0.0))//if((fabs(cs[0]-1.0)>MIN_DOUBLE)||(fabs(cs[1])>MIN_DOUBLE))//
						{
							for(j=1;j<=m;j++)
							{
								ix=(j-1)*m+i-1;
								iy=(j-1)*m+kk-2;
								d=cs[0]*u[ix]+cs[1]*u[iy];								
								u[iy]=-cs[1]*u[ix]+cs[0]*u[iy];
								u[ix]=d;
							}
						}
					}
				}
			}
		}
	}

	free(s);
	free(e);
	free(w);
	return l;
}
