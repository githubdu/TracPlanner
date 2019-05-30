



#pragma once


#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "robotMath.h"
#include "interpolate.h"
#include "typesAndErrors.h"

#define TRANS_D		(3)
#define ROTATE_D	(3)
#define REP_SUCCEED	(0)

class TRAC
{
public:
	TRAC();
	~TRAC();

public:
	void   initiate();

public:
	double getMaxVel();
	double getMaxAcc();
	double getMaxJerk();

public:
	double getChordLength();
	double getProfileLength();

public:
	bool   isValid() const;
	trcTyp getType() const;
	double getDuration() const;
	double getStartTime() const;
	double getTargetTime() const;

public:
	void   getStart(double startPos[]) const;
	void   getTarget(double targetPos[]) const;

public:
	virtual bool   stopTrajectory(double t);
	virtual bool   scaleToDuration(double newDuration);
	virtual int    rePlanTrajectory(double t,double newRefVel);

public:
	void   setLimit(double vLimit,double aLimit,double jLimit);

public:
	void   getLimit(double& vLimit,double& aLimit,double& jLimit) const;

public:
	virtual bool   isInCruisingPhase(double t);
	virtual double pos(double t, double pos[]) const;
	virtual double vel(double t, double vel[]) const;
	virtual double acc(double t, double acc[]) const;
	virtual double jerk(double t, double jerk[]) const;
	virtual double tangent(double t, double tangent[]) const;
	virtual double move(double t, double pos[], double vel[], double acc[])const;

protected:
	int		_dof;
	bool	_isValid;
	bool	_newParam;
	bool	_firstArc;
	bool	_firstMax;
	bool	_isAxisMode;

protected:
	double	_velMax;
	double	_accMax;
	double	_jerkMax;
	double	_profile;
	double  _chordLength;

protected:
	double	_velLimit;
	double	_accLimit;
	double	_jerkLimit;

protected:
	trcTyp  _tracType;
	double	_startTime;
	double  _targetTime;
	double	_startPos[MAX_DOF];
	double  _targetPos[MAX_DOF];

protected:
	virtual void _searchMax();
	virtual void _intArcLength();
	virtual bool _planTrajectory();

protected:
	void _setStart(double startPos[]);
	void _setTarget(double targetPos[]);
	void _setStartTime(double startTime);
};

class AXIS:public TRAC
{
public:
	AXIS();
	~AXIS();

public:
	void   initiate();

public:
	void   setAxisLimits(int dof, double vLimit[],double aLimit[],double jLimit[]);
	
public:
	int    getDOF() const;
	void   getAxisLimits(double vLimit[], double aLimit[], double jLimit[]) const;

public:
	bool   stopTrajectory(double t);
	bool   scaleToDuration(double newDuration);
	int    rePlanTrajectory(double t,double newVelLimit);
	int    rePlanAxisTrajectory(double t,double newVelLimits[]);
	bool   planTrajectory(double t0, double p0[], double pf[],double v0[]=nullptr, double vf[]=nullptr);

public:
	bool   isInCruisingPhase(double t);
	double pos(double t, double pos[]) const;
	double vel(double t, double vel[]) const;
	double acc(double t, double acc[]) const;
	double jerk(double t, double jerk[]) const;
	double tangent(double t, double tangent[]) const;
	double move(double t, double pos[], double vel[], double acc[])const;

private:
	void _searchMax();
	void _intArcLength();
	bool _planTrajectory();

private:
	void _setAxisStartVel(double vel[]);
	void _setAxisTargetVel(double vel[]);

private:
	DS_or_Trap	_dstPlanner[MAX_DOF];

private:
	double		_v0[MAX_DOF];
	double      _vf[MAX_DOF];
	double		_velAxisLimits[MAX_DOF];
	double		_accAxisLimits[MAX_DOF];
	double		_jerkAxisLimits[MAX_DOF];
};

class LINE:public TRAC
{
public:
	LINE();
	~LINE();

public:
	void   initiate();

public:
	bool   stopTrajectory(double t);
	bool   scaleToDuration(double newDuration);
	int    rePlanTrajectory(double t,double newRefVel);
	bool   planTrajectory(double t0, double sP[], double tP[]);

public:
	bool   isInCruisingPhase(double t);
	double pos(double t, double pos[]) const;
	double vel(double t, double vel[]) const;
	double acc(double t, double acc[]) const;
	double jerk(double t, double jerk[]) const;
	double tangent(double t, double tangent[]) const;
	double move(double t, double pos[], double vel[], double acc[])const;

private:
	void   _searchMax();
	void   _intArcLength();
	bool   _planTrajectory();

private:
	double		_xyzScale[3];
	DS_or_Trap  _interpolater;
};

class CIRCLE:public TRAC
{
public:
	CIRCLE();
	~CIRCLE();

public:
	void   initiate();

public:
	void   getTarget(double targetPos[]) const;
	double getNewCenterAngleAndMiddlePointAfterStop(double midPoint[]) const;

public:
	bool   stopTrajectory(double t);
	bool   scaleToDuration(double newDuration);
	int    rePlanTrajectory(double t,double newVelLimit);
	bool   planTrajectory(double t0, double sP[],double cP[],double nV[],double cA);
	bool   planTrajectory(double t0, double sP[],double mP[],double tP[],int cycle=0);

public:
	bool   isInCruisingPhase(double t);
	double pos(double t, double pos[]) const;
	double vel(double t, double vel[]) const;
	double acc(double t, double acc[]) const;
	double jerk(double t, double jerk[]) const;
	double tangent(double t, double tangent[]) const;
	double move(double t, double pos[], double vel[], double acc[])const;

private:
	void   _searchMax();
	void   _intArcLength();
	bool   _planTrajectory();

private:
	void   _setTarget(double tP[]);
	void   _setCenterAngle(double cA);
	void   _setCenterPoint(double cP[]);
	void   _setNormalVector(double nV[]);

private:
	double _theta;
	double _radius;
	double _center[TRANS_D];
	double _normal[TRANS_D];
	double _matrix[TRANS_D][TRANS_D];

private:
	double _newTheta;
	double _middlePoint[TRANS_D];

private:
	DS_or_Trap _interpolater;
};

class BSPLINE:public TRAC
{
#define BSPLINE_ORDER (3)

public:
	BSPLINE();
	~BSPLINE();

public:
	void   initiate();

public:
	double getChordLength();

public:
	int    getPointsNumber() const;
	int    getNowPointIndexAfterStop() const;
	int    getTimeSequence(double timeSequence[]) const;
	int    getPointsSequence(double pointsSequence[][TRANS_D]) const;

public:
	bool   stopTrajectory(double t);
	bool   scaleToDuration(double newDuration);
	int    rePlanTrajectory(double t, double newRefVel);
	bool   planTrajectory(double timeSequence[], double pointsSequence[][TRANS_D], int num);
	bool   planTrajectory(double startTime, double targetTime, double pointsSequence[][TRANS_D], int num);

public:
	bool   isInCruisingPhase(double t);
	double pos(double t,double pos[]) const;
	double vel(double t,double vel[]) const;
	double acc(double t,double acc[]) const;
	double jerk(double t,double jerk[]) const;
	double tangent(double t, double tangent[]) const;
	double move(double t,double pos[], double vel[], double acc[])const;

private:
	void   _searchMax();
	void   _intArcLength();
	bool   _planTrajectory();

private:
	bool   _checkTimeSequence();
	bool   _cubicControlPoint();
	bool   _cubicBsplinePoint();
	bool   _computeKnotVector();

private:	
	int    _iSpan(double time)const;
	int    _Bpoint(double time, double* s) const;
	bool   _3rdControlPoint(double cp[4][TRANS_D]);
	int    _basisfuncs(int i_span, double time, double* B);
	int    _basisfuncs_single(int i_span, double time, double* B);

private:
	int    _setTargetTime(double tTime);
	int    _setTimeSequence(double timeSequence[],int num);
	int    _setPointsSequence(double pointsSequence[][TRANS_D],int num);

private:
	int    _span;
	int	   _tNum;
	int	   _pNum;
	int	   _bNum;
	int	   _kNum;
	int	   _cpNum;

private:
	bool   _stop;
	double _ratio;

	// allocate memory for time saving
	double _sVel[TRANS_D];
	double _tVel[TRANS_D];
	double _bCP[6][TRANS_D];
	double _t[POINTS_MAX_NUM];
	double _oP[POINTS_MAX_NUM][TRANS_D];
	double _kT[POINTS_MAX_NUM+2*BSPLINE_ORDER+1];
	double _cP[POINTS_MAX_NUM+BSPLINE_ORDER][TRANS_D];
	double _bP[POINTS_MAX_NUM][(BSPLINE_ORDER+1)*TRANS_D];

	double _DL[BSPLINE_ORDER+1];
	double _DR[BSPLINE_ORDER+1];
	double _DS[BSPLINE_ORDER+1][TRANS_D];
	double _DA[BSPLINE_ORDER+1][BSPLINE_ORDER+1];
	double _DB[BSPLINE_ORDER+1][BSPLINE_ORDER+1];
	double _DU[BSPLINE_ORDER+1][BSPLINE_ORDER+1];
	double _B[(BSPLINE_ORDER+1)*(BSPLINE_ORDER+1)];
};

class BLENDER:public TRAC
{
#define BEZIER_ORDER (5)	

public:
	BLENDER();
	~BLENDER();

public:
	void    initiate();

public:
	void	setPostTrac(TRAC* postTrac);
	void	setPrevTrac(TRAC* prevTrac);
	void    setBlendZone(int zoneType, double zoneArea);

public:
	TRAC*	getPostTrac() const;
	TRAC*	getPrevTrac() const;
	double	getPostTime() const;
	double	getPrevTime() const;
	void    getBlendZone(int& zoneType, double& zoneArea) const;

public:
	bool	planTrajectory();
	bool	stopTrajectory(double t);
	bool	scaleToDuration(double newDuration);
	int 	rePlanTrajectory(double t, double newRefVel);

public:
	double	pos(double t, double pos[]) const;
	double	vel(double t, double vel[]) const;
	double	acc(double t, double acc[]) const;
	double	jerk(double t, double jerk[]) const;
	double  tangent(double t, double tangent[]) const;
	double	move(double t, double pos[], double vel[], double acc[])const;

private:
	int		_zType;
	double	_tTime;
	double  _zArea;

	TRAC*	_postTrac;
	TRAC*	_prevTrac;

	double	_ratio;
	double	_prevTime;
	double	_postTime;
	double	_mP[TRANS_D];
	double	_cP[6][TRANS_D];
	double	_postP[TRANS_D];
	double	_prevP[TRANS_D];
	double	_postPos[TRANS_D];
	double	_postVel[TRANS_D];
	double	_postAcc[TRANS_D];
	double	_prevPos[TRANS_D];
	double	_prevVel[TRANS_D];
	double	_prevAcc[TRANS_D];

private:
	void	_searchMax();
	void	_intArcLength();
	bool    _planTrajectory(){return planTrajectory();};
	bool	isInCruisingPhase(double t){ return false; };

private:
	bool	_checkBlendType();
	bool	_5thControlPoints();
	bool	_computeTracPoints();
};

class ROTATION:public TRAC
{
	/* RPY is used for pos */
public:
	ROTATION();
	~ROTATION();

public:
	void   initiate();

public:
	bool   stopTrajectory(double t);
	bool   scaleToDuration(double newDuration);
	int    rePlanTrajectory(double t,double newRelVel);
	bool   planTrajectory(double t0, double p0[], double pf[]);
	bool   planTrajectory(double t0, double p0[], double axis[], double angle);

public:
	bool   isInCruisingPhase(double t);
	double pos(double t, double pos[]) const;
	double vel(double t, double vel[]) const;
	double acc(double t, double acc[]) const;
	double jerk(double t, double jerk[]) const;
	double tangent(double t, double tangent[]) const;
	double move(double t,double pos[], double vel[], double acc[])const;

private:
	double		_axis[3];
	double		_sR[3][3];
	DS_or_Trap  _interpolater;

private:
	void   _searchMax();
	void   _intArcLength();
	bool   _planTrajectory();

private:
	int    _setTarget(double targetPos[]);
	int    _setAxisAngle(double axis[], double angle);
};

class MULTI_ROT:public TRAC
{
	/* RPY is used for pos */
public:
	MULTI_ROT();
	~MULTI_ROT();

public:
	void   initiate();

public:
	int    getPointsNumber() const;
	int    getTimeSequence(double time[]) const;
	int    getRpySequence(double euler[][ROTATE_D]) const;
	int    getAxisAngleSequence(double axis[][ROTATE_D], double angle[]) const;

public:
	bool   stopTrajectory(double t);
	bool   scaleToDuration(double newDuration);
	int    rePlanTrajectory(double t, double newVelLimit);
	bool   planTrajectory(double t[], int num, double rpy[][ROTATE_D]);
	bool   planTrajectory(double t[], double sR[ROTATE_D], int num, double axis[][ROTATE_D], double angle[]);

public:
	bool   isTimeChanged() const;
	bool   isInCruisingPhase(double t);
	double pos(double t, double pos[]) const;
	double vel(double t, double vel[]) const;
	double acc(double t, double acc[]) const;
	double jerk(double t, double jerk[]) const;
	double tangent(double t, double tangent[]) const;
	double move(double t, double pos[], double vel[], double acc[]) const;

private:
	int	   _pNum;
	int	   _rePlanN;
	bool   _tChange;

private:
	ROTATION _rotate[POINTS_MAX_NUM];

private:
	double _t[POINTS_MAX_NUM];
	double _angle[POINTS_MAX_NUM];
	double _rpy[POINTS_MAX_NUM][ROTATE_D];
	double _axis[POINTS_MAX_NUM][ROTATE_D];

private:
	void   _searchMax();
	void   _intArcLength();
	bool   _planTrajectory();
	bool   _synchronizePlanning();

private:
	int    _iSpan(double t) const;
	bool   _checkTimeT(double t[]) const;

private:
	int    _setRpySequence(double rpy[][ROTATE_D],int num);
	int    _setTimeSequence(double timeSequence[],int num);
	int    _setAxisAngleSequence(double axis[][ROTATE_D], double angle[], int num);
	int	   _distanceOfRpy(double axix[3],double& angle, double E1[3], double E2[3]);
};



#endif // !TRAJECTORY_H_