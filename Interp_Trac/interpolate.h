

#pragma once


#ifndef INTERPOLATE_H_
#define INTERPOLATE_H_


#include "robotMath.h"


enum prfTyp
{
	doubleS,	// S
	trapezoid,	// T
	dS_or_Trap,	// U
	multi_ST_1D	// M
};

class DoubleS
{
public:
	DoubleS();
	~DoubleS();

public:
	void   initiate();

public:
	bool   isValid() const;

	double getMaxVel() const;

	double getMaxAcc() const;

	double getMaxJerk() const;

	double getDuration() const;

	prfTyp getProfileType() const;

public:
	double pos(double t) const;

	double vel(double t) const;

	double acc(double t) const;

	double jerk(double t) const;

public:
	bool   scaleTo(double newDuration);

	void   setLimit(double v_limit, double a_limit, double j_limit);

	bool   planProfile(double t0, double p0, double pf, double v0=0, double vf=0);

private:
	double _t[8];
	double _x[8];
	double _v[8];
	double _a[8];
	double _j[8];

	int	   _dir;
	bool   _isValid;
	double _velLimit;
	double _accLimit;
	double _jerkLimit;
};

class Trapezoid
{
public:
	Trapezoid();
	~Trapezoid();

public:
	void   initiate();

public:
	bool   isValid() const;

	double getMaxVel() const;

	double getMaxAcc() const;

	double getMaxJerk() const;

	double getDuration() const;

	prfTyp getProfileType() const;

public:
	double pos(double t) const;

	double vel(double t) const;

	double acc(double t) const;

	double jerk(double t) const;

public:
	bool   scaleTo(double newDuration);

	void   setLimit(double v_limit, double a_limit, double j_limit);

	bool   planProfile(double t0,  double p0, double pf, double v0=0, double vf=0);

private:
	double _t[4];
	double _x[4];
	double _v[4];
	double _a[4];

	int	   _dir;
	bool   _isValid;
	double _velLimit;
	double _accLimit;
	double _jerkLimit;
};

class DS_or_Trap
{
public:
	DS_or_Trap();
	~DS_or_Trap();

public:
	void   initiate();

public:
	bool   isValid() const;

	double getMaxVel() const;

	double getMaxAcc() const;

	double getMaxJerk() const;

	double getDuration() const;

	prfTyp getProfileType() const;

public:
	double pos(double t) const;

	double vel(double t) const;

	double acc(double t) const;

	double jerk(double t) const;

public:
	bool   scaleTo(double newDuration);

	void   setLimit (double vl, double al, double jl=0);

	bool   planProfile(double t0, double p0, double pf, double v0=0, double vf=0);

private:
	prfTyp	  _curPrf;
	DoubleS	  _doubleS;
	Trapezoid _trapezoid;
};

class Multi_ST_1D
{
public:
	Multi_ST_1D();
	~Multi_ST_1D();

public:
	void   initiate();

public:
	bool   isValid() const;

	double getMaxVel() const;

	double getMaxAcc() const;

	double getMaxJerk() const;

	double getDuration() const;

	prfTyp getProfileType() const;

public:
	double pos(double t) const;

	double vel(double t) const;

	double acc(double t) const;

	double jerk(double t) const;

public:
	bool   setStartVel(double v0);

	bool   setTargetVel(double vf);

	void   setLimit(double vl, double al, double jl = 0);

public:
	bool   scaleTo(double newDuration);

	bool   rePlanProfile(double t, double vl, double al, double jl);

	bool   planProfile(double* t, int num, double* p, double* v = nullptr);

	bool   planProfile(double t0, double tf, int num, double* p, double* v = nullptr);

public:
	bool   isTimeChanged() const;

	bool   getTimeSequence(double* t, int num) const;

	bool   move(double t, double* pos, double* vel, double* acc) const;

private:
	int  _computeMax();
	bool _computeTimeT();
	bool _computeVelocity();
	bool _computeAcceleration();
	bool _checkAndRecomputeTimeT();

private:
	int  _iSpan(double t) const;
	bool _checkTimeT(double* t) const;

private:
	double		_v0;					// start velocity
	double		_vf;					// target velocity
	double		_vMax;					// maximum velocity
	double		_aMax;					// maximum acceleration
	double		_jMax;					// maximum jerk
	double		_vLimit;				// velocity limit
	double		_aLimit;				// acceleration limit
	double		_jLimit;				// jerk limit
	double		_rePlanT;				// re-plan time

	int			_pNum;					// total points number, including start
	int			_rePlanN;				// for on line re-planning
	bool		_isValid;				// whether the profiles are valid or not
	bool		_fistMax;				// vMax, aMax and jMax only needs to compute once
	bool		_timeChanged;			// the user input time sequence may be invalid( causing over-speeding)

private:
	double		_t[POINTS_MAX_NUM];		// using pointers to the outside data instead of data copying
	double		_p[POINTS_MAX_NUM];		//     is less significant, allocate its own memory for t/p/v
	double		_v[POINTS_MAX_NUM];		//     makes it non-affected by the outside changing
	DS_or_Trap	_dst[POINTS_MAX_NUM];	// it can NOT keep acceleration continuous at the nodal points
};


#endif  //!INTERPOLATE_H_