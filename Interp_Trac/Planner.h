



#pragma once


#ifndef _PLANNER_H_
#define _PLANNER_H_

#include "Frame.h"
#include "trajectory.h"

#define PLANNER_SUCCEED	(0)
#define PLANNER_DONE	(201)
#define PLANNER_WAITING	(202)
#define PLANNER_MOVING  (203)
#define PLANNER_ZONE_IN	(204)
#define PLANNER_ZONING	(205)
#define PLANNER_ZONEOUT	(206)
#define PLANNER_NONEXT	(207)
#define PLANNER_NOZONE	(208)
#define PLANNER_NOTYET	(209)





class Planner6D
{
/* RPY for rotation pos */

public:
	Planner6D();
	~Planner6D();

public:
	int   initiate();
	int   getUnfinishedBsplineIndex();
	int   getUnfinishedCircleCenterAngle(double& angle);
	int   getUnfinishedCircleMiddlePoint(double midPoint[]);

public:
	bool  isInJointSpace(){ return _isNowInJointSpace;};

public:
	/********************************************************************************/
	/* planner will stop current trajectory and hold still
	/* 
	/*	Return:	PLANNER_SUCCEED,	planner succeed.
	/*			PLANNER_NOTYET,		planner is in transition, call this later
	/*			ERR_PLAN_FAILED,	something wrong happened with the planner
	/********************************************************************************/
	int stop();
	
	/********************************************************************************/
	/* planner will hold still for wTime seconds
	/*
	/* Input:	wTime,	waiting time, s
	/* 
	/* Return:	PLANNER_SUCCEED,	planner succeed.
	/*			PLANNER_NOTYET,		the prev trajectory is not done, call this later
	/*			ERR_PLAN_FAILED,	something wrong happened with the planner
	/********************************************************************************/
	int wait(double wTime);
	
	/********************************************************************************/
	/* planner will re-plan the current trajectory with new reference velocity
	/* 
	/* Input:	refVel, new set reference velocity
	/*
	/* Return:	PLANNER_SUCCEED,	planner succeed.
	/*			PLANNER_NOTYET,		planner not ready, call this later
	/*			ERR_PLAN_FAILED,	something wrong happened with the planner
	/********************************************************************************/
	int rePlan(double refVel);
	
	/********************************************************************************/
	/* initialize the planner by call this function
	/*			call this right before moveJ
	/*
	/* Input:	dof,				degree of freedom
	/*			currentJointAngle,  current joint angle of the robot
	/*
	/* Return:	PLANNER_SUCCEED,	planner succeed.
	/*			PLANNER_NOTYET,		the prev trajectory is not done, call this later
	/********************************************************************************/
	int updateCurrentJoint(int dof, double currentJointAngle[]);

	/********************************************************************************/
	/* initialize the planner by call this function
	/*			call this right before moveL/moveC/moveB
	/*			do NOT call this right before or during moveJ
	/*
	/* Input:	currentPos,			current Cartesian pose of the flange
	/*			refTool,			reference tool center point(flange --> TCP)
	/*
	/* Return:	PLANNER_SUCCEED,	planner succeed.
	/*			PLANNER_NOTYET,		the prev trajectory is not done, call this later
	/********************************************************************************/
	int updateCurrentPos(double currentPos[6],double refTool[6]=nullptr);
	
	/********************************************************************************/
	/* set the joint space limits, v_limit, a_limit, j_limit, only 1 call is needed
	/*
	/* Input:	dof,				degree of freedom of the joint space
	/*			jvl,jal,jjl:		vel_limit, acc_limit, jerk_limit
	/*
	/* Return:	PLANNER_SUCCEED,	planner succeed.
	/********************************************************************************/
	int setJointLimits(int dof, double jvl[],double jal[],double jjl[]);
	
	/********************************************************************************/
	/* move() needs to be called cyclically to get nextPos/nextVel/nextAcc
	/*
	/* Input:	step,				time step, ms
	/* Output:	nextPos,			next target pose of the flange
	/*			nextVel,			next target velocity of the flange
	/*			nextAcc,			next target acceleration of the flange
	/*
	/*	Return: PLANNER_DONE,		all motions are done.
	/*			PLANNER_WAITING,	planner is sleeping, responed for wait()
	/*			PLANNER_MOVING,		the current trajectory is moving, not finished
	/*			PLANNER_ZONE_IN,	the current trajectory is finished or entering
	/*								blender zone, please extract next motion and
	/*								call wait/moveR/moveL/moveB/moveC(if exists)
	/*			PLANNER_ZONEING,	the planner is transiting to next trajectory
	/*			ERR_PLAN_FAILED,	something wrong happened with the planner
	/********************************************************************************/
	int move(int step, double nextPos[], double nextVel[], double nextAcc[]);
		
	/********************************************************************************/
	/* set the translation and rotation limits, v_limit, a_limit, j_limit
	/*			initialize the planner by call this function, only 1 call is needed
	/*
	/* Input:	tvl,tal,tjl			for translation
	/*			rvl,ral,rjl			or rotation
	/*
	/* Return:	PLANNER_SUCCEED,	planner succeed.
	/********************************************************************************/
	int setCartesianLimits(double tvl,double tal,double tjl,double rvl,double ral,double rjl);

	/********************************************************************************/
	/* plan a pure rotation motion, 
	/*			call this when new rotation motion arrived
	/*
	/* Input:	targetPos,			target pose of the TCP, position will be ignored
	/*			refVel,				reference velocity of the motion
	/*			refAcc,				reference acceleration of the motion
	/*			refJerk,			reference jerk of the motion
	/*          rel,				the target is relative or absolute
	/*			refTime,			the reference time of the motion
	/*
	/* Return:	PLANNER_SUCCEED,	planner succeed.
	/*			PLANNER_NOTYET,		the prev trajectory is not done, call this later
	/*			ERR_PLAN_FAILED,	something wrong happened with the planner
	/********************************************************************************/
	int moveR(double targetPos[6], double refVel, double refAcc=1.0, double refJerk=1.0, bool rel=false, double refTime=0);

	/********************************************************************************/
	/* plan a line trajectory
	/*			call this when new PTP motion arrived, and after the function
	/*			updateCurrentJoint() returns PLANNER_SUCCEED
	/*
	/* Input:	targetJoint,		target joints of the TCP
	/*			refVel,				reference velocity of the motion
	/*			refAcc,				reference acceleration of the motion
	/*			refJerk,			reference jerk of the motion
	/*          rel,				is the target relative or absolute
	/*			refTime,			reference time
	/*
	/* Return:	PLANNER_SUCCEED,	planner succeed.
	/*			PLANNER_NOTYET,		the prev trajectory is not done, call this later
	/*			ERR_PLAN_FAILED,	something wrong happened with the planner
	/********************************************************************************/
	int moveJ(double targetJoint[], double refVel, double refAcc=1.0, double refJerk=1.0, bool rel=false, double refTime=0);

	/********************************************************************************/
	/* plan a B spline trajectory
	/*			call this when new B spline motion arrived
	/*
	/* Input:	targetPos,			target pose of the TCP
	/*			pointsNum,			points number of the b spline
	/*			refVel,				reference velocity for the trajectory
	/*			zArea,				size of the zone area
	/*			zType,				length or ratio of the zone area
	/*          rel,				the target is relative or absolute
	/*			refTime,			reference total time for the trajectory
	/*
	/* Return:	PLANNER_SUCCEED,	planner succeed.
	/*			PLANNER_NOTYET,		the prev trajectory is not done, call this later
	/*			ERR_PLAN_FAILED,	something wrong happened with the planner
	/********************************************************************************/
	int moveB(double targetPos[][6], int posNum, double refVel, double zArea=0, int zType=0, bool rel=false, double refTime=0);
	
	/********************************************************************************/
	/* plan a line trajectory
	/*			call this when new line motion arrived
	/*
	/* Input:	targetPos,			target pose of the TCP
	/*			refVel,				reference velocity of the motion
	/*			refAcc,				reference acceleration of the motion
	/*			refJerk,			reference jerk of the motion
	/*			zArea,				size of the zone area
	/*          zType,				length or ratio of the zone area
	/*          rel,				the target is relative or absolute
	/*			refTime,			the reference time of the motion
	/*
	/* Return:	PLANNER_SUCCEED,	planner succeed.
	/*			PLANNER_NOTYET,		the prev trajectory is not done, call this later
	/*			ERR_PLAN_FAILED,	something wrong happened with the planner
	/********************************************************************************/
	int moveL(double targetPos[6], double refVel, double refAcc=1.0, double refJerk=1.0, double zArea=0, int zType=0, bool rel=false, double refTime=0);

	/********************************************************************************/
	/* plan a circle trajectory
	/*			call this when new circle motion arrived
	/*
	/* Input:	targetPos,			target pose of the TCP
	/*			middlePos,			middle point of the circle
	/*			refVe,				reference velocity of the motion
	/*			refAcc,				reference acceleration of the motion
	/*			refJerk,			reference jerk of the motion
	/*			zArea,				size of the zone area
	/*			zType,				length or ratio of the zone area
	/*          rel,				the target is relative or absolute
	/*			refTime,			the reference time of the motion
	/*			cycle,				the cycle of the circle
	/*
	/* Return:	PLANNER_SUCCEED,	planner succeed.
	/*			PLANNER_NOTYET,		the prev trajectory is not done, call this later
	/*			ERR_PLAN_FAILED,	something wrong happened with the planner
	/********************************************************************************/
	int moveC(double targetPos[6], double middlePos[3], double refVel, double refAcc=1.0, double refJerk=1.0, double zArea=0, int zType=0, bool rel=false,double refTime=0, int cycle=0);

	/********************************************************************************/
	/* plan a circle trajectory
	/*			call this when new circle motion arrived
	/*
	/* Input:	centerPoint,		center point of the circle
	/*			centerAngle,		center angle of the circle
	/*			normVec,			normal vector of the circle
	/*			targetRot,			target posture of the TCP
	/*			refVe,				reference velocity of the motion
	/*			refAcc,				reference acceleration of the motion
	/*			refJerk,			reference jerk of the motion
	/*			zArea,				size of the zone area
	/*			zType,				length or ratio of the zone area
	/*          rel,				the target is relative or absolute
	/*			refTime,			the reference time of the motion
	/*
	/* Return:	PLANNER_SUCCEED,	planner succeed.
	/*			PLANNER_NOTYET,		the prev trajectory is not done, call this later
	/*			ERR_PLAN_FAILED,	something wrong happened with the planner
	/********************************************************************************/
	int moveC(double centerPoint[3],double centerAngle, double normVec[3], double targetRot[3], double refVel, double refAcc=1.0,double refJerk=1.0, double zArea=0, int zType=0, double refTime=0);


protected:
	int	_moveZ();
	int _transferVelToFlange(double nextVel[6]);
	int _transferAccToFlange(double nextVel[6],double nextAcc[6]);

protected:
	int	_getTarget(double nextPos[], double nextVel[], double nextAcc[]);

protected:
	TRAC*		_rotPtr;
	TRAC*		_tracPtr;

	AXIS		_axisPtpPlanner;

	LINE		_prevLinePlanner;
	LINE		_postLinePlanner;

	CIRCLE		_prevCirclePlanner;
	CIRCLE		_postCirclePlanner;

	BSPLINE		_prevBsplinePlanner;
	BSPLINE		_postBsplinePlanner;

	BLENDER		_blendPlanner;

	ROTATION	_prevRotPlanner;
	ROTATION	_postRotPlanner;

	MULTI_ROT	_prevMultiRotPlanner;
	MULTI_ROT	_postMultiRotPlanner;

	int			_dof; // used for point-to-point in joint space

	/*******************************************************************************
	/*	_plannerState:	DONE/WAITING/MOVING/ZONING
	/*					1, DONE-->WAITING,	  call wait(wTime)
	/*					2, WAITING-->DONE,	  call wait() and _plannerWaitTime <= 0
	/*					3, DONE-->MOVING,	  call moveJ/moveL/moveC/moveR/moveB
	/*					4, MOVING-->ZONEIN,	  _plannerStartTime > _plannerSwitchTime
	/*					5, ZONEIN-->DONE,	  _blenderState == PLANNER_NOZONE
	/*					6, ZONEIN-->ZONING,   _blenderState == PLANNER_SUCCEED
	/*										    || _blenderState == PLANNER_NONEXT
	/*					7, ZONING-->ZONEOUT,  _plannerStartTime > _plannerSwitchTime
	/*										    && _blenderState == PLANNER_SUCCEED
	/*					8, ZONING-->DONE,	  _plannerStartTime > _plannerSwitchTime
	/*										    && _blenderState == PLANNER_NONEXT
	/*					9, ZONEOUT-->MOVING,  moveJ/moveL/moveC/moveR/moveB SUCCEED	
	*******************************************************************************/
	int			_plannerState;

	/*******************************************************************************
	/*	_blenderState:	PLANNER_SUCCEED/PLANNER_NOZONE/PLANNER_NONEXT/ERR_PLAN_FAILED
	/*					PLANNER_SUCCEED,	transiting zone between two trajectories
	/*					PLANNER_NOZONE,		no transiting zone
	/*					PLANNER_NONEXT,		this trajectory is the last one
	/*					ERR_PLAN_FAILED,	transiting zone
	*******************************************************************************/
	int			_blenderState;

	bool		_isNowInJointSpace;
	bool		_isPostPlannerPlanned;

	double		_rotVelLimit;
	double		_rotAccLimit;
	double		_rotJerkLimit;

	double		_tracVelLimit;
	double      _tracAccLimit;
	double		_tracJerkLimit;

	double      _rotTimeOffset;
	double		_plannerStartTime;
	double		_plannerSwitchTime;

	double		_lastTracRefTool[6];
	Point6D     _lastTracRefToolInv;

	double		_lastTracTargetTcpPos[6];
	double		_lastTracTargetJoint[MAX_DOF];

	double		_jointVelLimit[MAX_DOF];
	double		_jointAccLimit[MAX_DOF];
	double		_jointJerkLimit[MAX_DOF];

	int			_newPointIndexAfterStop;
	double		_newCenterAngleAfterStop;
	double		_newMiddlePointAfterStop[TRANS_D];
};

class PlannerAgv
{
public:
	PlannerAgv();
	~PlannerAgv();

public:
	int   initiate();
	int   deInitiate();
	int   getTracType();
	int   getUnfinishedBsplineIndex();
	int   skipToPlannerTime(double time);
	int   getUnfinishedCircleCenterAngle(double& angle);
	int   getTargetTurnOfNextMotionOfFlange(double& turn);
	int   getUnfinishedCircleMiddlePoint(double midPoint[]);
	int   getUnfinishedBlenderMiddlePoint(double midPoint[]);
	int   getStartAndSwitchTime(double& startTime, double& switchTime);

public:
	int stop();

	int wait(double wTime);
	
	int rePlan(double refVel);

	int moveT(double targetW, bool rel = false, double refTime = 0);

	int updateCurrentPos(double currentPos[4],double refTool[3]=nullptr);

	int move(int step, double nextPos[4], double nextVel[4], double nextAcc[4]);

	int setCartesianLimits(double tvl,double tal,double tjl,double rvl,double ral,double rjl);

	int moveB(double targetPos[][3], int pNum, double refVel, double zArea=0, int zType=0, bool rel=false, double refTime=0, double turnTime=0);
	
	int moveR(double targetPos[], double refDir, double refVel, double refAcc=1.0, double refJerk=1.0, int rCycle=0, bool rel=false, double refTime=0, double turnTime=0);

	int moveL(double targetPos[3], double refVel, double refAcc=1.0, double refJerk=1.0, double zArea=0, int zType=0, int rCycle=0, bool rel=false, double refTime=0, double turnTime=0);

	int moveC(double targetPos[3], double middlePos[2], double refVel, double refAcc=1.0, double refJerk=1.0, double zArea=0, int zType=0, int cCycle=0, int rCycle=0, bool rel=false,double refTime=0, double turnTime=0);

	int moveC(double centerPos[2], double centerAngle, double targetA, double refVel, double refAcc=1.0,double refJerk=1.0, double zArea=0, int zType=0, int rCycle=0, bool rel=false,double refTime=0, double turnTime=0);

private:
	int	_moveZ();
	int	_prepareTurning();
	int	_stopTurningPlanner();
	int _transferVelToFlange(double nextVel[]);
	int _transferAccToFlange(double nextVel[],double nextAcc[]);
	int _getTargetTurnOfFlange(double&targetTurn, TRAC* tPtr, TRAC* rPtr);
	int	_getTarget(double nextPos[4], double nextVel[4], double nextAcc[4]);


private:
	TRAC*		_rotPtr;
	TRAC*		_tracPtr;

	LINE		_prevLinePlanner;
	LINE		_postLinePlanner;

	CIRCLE		_prevCirclePlanner;
	CIRCLE		_postCirclePlanner;

	BSPLINE		_prevBsplinePlanner;
	BSPLINE		_postBsplinePlanner;

	BLENDER		_blendPlanner;

	ROTATION	_prevRotPlanner;
	ROTATION	_postRotPlanner;

	MULTI_ROT	_prevMultiRotPlanner;
	MULTI_ROT	_postMultiRotPlanner;

	DS_or_Trap	_turningPlanner;

	//---------------------------------------
	int			_tracType;
	int			_plannerState;
	int			_blenderState;

	bool		_isPostPlannerPlanned;

	double		_rotVelLimit;
	double		_rotAccLimit;
	double		_rotJerkLimit;

	double		_tracVelLimit;
	double      _tracAccLimit;
	double		_tracJerkLimit;

	double      _rotTimeOffset;
	double		_plannerStartTime;
	double		_plannerSwitchTime;

	double		_lastTracRefTool[6];
	Point6D     _lastTracRefToolInv;

	double		_lastTracTargetTcpPos[6];
	double		_lastTracTargetTurnOfFlange;
	double		_targetTurnOfNextMotionOfFlange;
	double		_refTurnTimeOfNextMotionOfFlange;

	int			_newPointIndexAfterStop;
	double		_newCenterAngleAfterStop;
	double		_newMiddlePointAfterStop[TRANS_D];
	double		_newBlenderPointAfterStop[TRANS_D+ROTATE_D];
};


#include "Kinematics.h"

class PlannerOpt : public Planner6D
{
public:
	PlannerOpt();
	~PlannerOpt();

public:
	int setKine(Kine kine);
	int setFlangeMask(int flangeMask[6]);

private:
	int _getJointVelLimit(double velUpLim[],double velLowLim[]);
	int	_getTarget(double nextPos[], double nextVel[], double nextAcc[]);
	int _projectMotion(double nextPos[], double nextVel[],double nextAcc[]);
	int _scaleOptForRedundant(double& scales, double jointVel[],double nextVel[]);
	int _scaleOptForNonRedundant(double& scales, double jointVel[],double nextVel[]);


public:
	int updateCurrentJoint(int dof, double currentJointAngle[]);
	int updateCurrentPos(double currentPos[6],double refTool[6]=nullptr);
	int move(int step, double nextPos[], double nextVel[], double nextAcc[]);

public:
	int setJointRange(int dof, double jointUpBound[], double jointLowBound[]);

public:
	int move(int step, double nextPos[], double nextVel[], double nextJoint[], double nextJointVel[]);

private:
	int	_step;
	Kine _kine;

	int    _flangeMask[6];

	double _jointUpBound[MAX_DOF];
	double _jointLowBound[MAX_DOF];
	double _lastTargetJointVel[MAX_DOF];

	double _lastTargetFlangePos[6];
	double _lastTargetFlangeVel[6];
};

#endif