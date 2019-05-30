



#pragma once
#include "trajectory.h"



TRAC::TRAC()
{
	initiate();
}

TRAC::~TRAC()
{

}

void   TRAC::initiate()
{
	_isValid = false;
	_newParam = true;
	_firstArc = true;
	_firstMax = true;

	_velMax = 0.0;
	_accMax = 0.0;
	_jerkMax = 0.0;

	_profile = 0.0;
	_chordLength = 0.0;

	_velLimit = 0.0;
	_accLimit = 0.0;
	_jerkLimit = 0.0;

	_startTime = 0.0;
	_targetTime = 0.0;

	memset(_startPos, 0, sizeof(_startPos));
	memset(_targetPos, 0, sizeof(_startPos));
}

double TRAC::getMaxVel()
{
	if (!_isValid)
	{
		DUMP_ERROR("ERR: the trajectory is invalid in <TRAC::getMaxVel>!\n");
		return -1;
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <TRAC::getMaxVel>!\n");
	}

	if (_firstMax)
	{
		_searchMax();
		_firstMax = false;
	}

	return _velMax;
}

double TRAC::getMaxAcc()
{
	if (!_isValid)
	{
		DUMP_ERROR("ERR: the trajectory is invalid in <TRAC::getMaxAcc>!\n");
		return -1;
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <TRAC::getMaxAcc>!\n");
	}

	if (_firstMax)
	{
		_searchMax();
		_firstMax = false;
	}
	return _accMax;
}

double TRAC::getMaxJerk()
{
	if (!_isValid)
	{
		DUMP_ERROR("ERR: the trajectory is invalid in <TRAC::getMaxJerk>!\n");
		return -1;
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <TRAC::getMaxJerk>!\n");
	}
	if (_firstMax)
	{
		_searchMax();
		_firstMax = false;
	}
	return _jerkMax;
}

void   TRAC::_searchMax()
{
	DUMP_ERROR("ERR: the tracType of TRAC is invalid in <TRAC::_searchMax>!\n");
	return;
}

void   TRAC::_intArcLength()
{
	DUMP_ERROR("ERR: the tracType of TRAC is invalid in <TRAC::_intArcLength>!\n");
	return;
}

bool   TRAC::isValid() const
{
	return _isValid;
}

trcTyp TRAC::getType() const
{
	return _tracType;
}

double TRAC::getChordLength()
{
	if (!_isValid)
	{
		DUMP_ERROR("ERR: the trajectory is invalid in <TRAC::getChordLength>!\n");
		return -0;
	}
	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <TRAC::getChordLength>!\n");
	}
	return ABS(_chordLength);
}

bool   TRAC::_planTrajectory()
{
	DUMP_ERROR("ERR: the ptr of TRAC is invalid in <TRAC::planTrajectory>!\n");
	return false;
}

double TRAC::getProfileLength()
{
	if (!_isValid)
	{
		DUMP_ERROR("ERR: the trajectory is invalid in <TRAC::getArcLength>!\n");
		return -1;
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <TRAC::getArcLength>!\n");
	}
	if (_firstArc)
	{
		_intArcLength();
		_firstArc = false;
	}

	return ABS(_profile);
}

double TRAC::getDuration() const
{
	if (!_isValid)
	{
		DUMP_ERROR("ERR: the trajectory is invalid in <TRAC::getDuration>!\n");
		return -0;
	}
	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <TRAC::getDuration>!\n");
	}

	return _isValid?(_targetTime - _startTime):-0;
}

double TRAC::getStartTime() const
{
	return _startTime;
}

double TRAC::getTargetTime() const
{
	if (!_isValid)
	{
		DUMP_ERROR("ERR: the trajectory is invalid in <TRAC::getTargetTime>!\n");
		return _startTime;
	}
	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <TRAC::getTargetTime>!\n");
	}

	return _targetTime;
}

bool   TRAC::stopTrajectory(double t)
{
	DUMP_ERROR("ERR: the tracType of TRAC is invalid in <TRAC::stopTrajectory>!\n");
	return false;
}

bool   TRAC::isInCruisingPhase(double t)
{
	DUMP_ERROR("ERR: the tracType of TRAC is invalid in <TRAC::isInCruisingPhase>!\n");
	return false;
}

void   TRAC::_setStart(double startPos[])
{
	if (startPos == nullptr)
	{
		DUMP_ERROR("ERR: input parameter is nullptr in <TRAC::setStart>!\n");
		return;
	}

	for (int i = 0; i < _dof; i++)
	{
		if (_startPos[i] != startPos[i])
		{
			_isValid = false;
			_newParam = true;
			_startPos[i] = startPos[i];
		}
	}
}

void   TRAC::_setTarget(double targetPos[])
{
	if (targetPos == nullptr)
	{
		DUMP_ERROR("ERR: input parameter is nullptr in <TRAC::setTarget>!\n");
		return;
	}
	for (int i=0; i<_dof; i++)
	{
		if (_targetPos[i] != targetPos[i])
		{
			_isValid = false;
			_newParam = true;
			_targetPos[i] = targetPos[i];
		}
	}
}

void   TRAC::_setStartTime(double startTime)
{
	if (_startTime != startTime)
	{	// without changing the profile
		_targetTime += startTime - _startTime;
		_startTime += startTime - _startTime;
	}
}

double TRAC::pos(double t, double pos[]) const
{
	DUMP_ERROR("ERR: the tracType of TRAC is invalid in <TRAC::pos>!\n");
	return 0;
}

double TRAC::vel(double t, double vel[]) const
{
	DUMP_ERROR("ERR: the tracType of TRAC is invalid in <TRAC::vel>!\n");
	return 0;
}

double TRAC::acc(double t, double acc[]) const
{
	DUMP_ERROR("ERR: the tracType of TRAC is invalid in <TRAC::acc>!\n");
	return 0;
}

void   TRAC::getStart(double startPos[]) const
{
	if (nullptr == startPos)
	{
		DUMP_ERROR("ERR: input parameter is nullptr in <TRAC::getStart>!\n");
		return ;
	}
	memcpy(startPos,_startPos,sizeof(double)*_dof);
}

double TRAC::jerk(double t, double jerk[]) const
{
	DUMP_ERROR("ERR: the tracType of TRAC is invalid in <TRAC::jerk>!\n");
	return 0;
}

void   TRAC::getTarget(double targetPos[]) const
{
	if (nullptr == targetPos)
	{
		DUMP_ERROR("ERR: input parameter is nullptr in <TRAC::getTarget>!\n");
		return;
	}
	memcpy(targetPos,_targetPos,sizeof(double)*_dof);
}

bool   TRAC::scaleToDuration(double newDuration)
{
	DUMP_ERROR("ERR: the tracType of TRAC is invalid in <TRAC::scaleToDuration>!\n");
	return false;
}

double TRAC::tangent(double t, double tangent[]) const
{
	DUMP_ERROR("ERR: the tracType of TRAC is invalid in <TRAC::acc>!\n");
	return 0;
}

int    TRAC::rePlanTrajectory(double t, double newVelLimit)
{
	DUMP_ERROR("ERR: the tracType of TRAC is invalid in <TRAC::rePlanTrajectory>!\n");
	return 0;
}

void   TRAC::setLimit(double vLimit, double aLimit, double jLimit)
{
	if (_velLimit != abs(vLimit))
	{
		_newParam = true;
		_velLimit = abs(vLimit);
	}

	if (_accLimit != abs(aLimit))
	{
		_newParam = true;
		_accLimit = abs(aLimit);
	}

	if (_jerkLimit != abs(jLimit))
	{
		_newParam = true;
		_jerkLimit = jLimit;
	}
}

void   TRAC::getLimit(double& vLimit, double& aLimit, double& jLimit) const
{
	vLimit = _velLimit;
	aLimit = _accLimit;
	jLimit = _jerkLimit;
}

double TRAC::move(double t, double pos[], double vel[], double acc[]) const
{
	DUMP_ERROR("ERR: the tracType of TRAC is invalid in <TRAC::move>!\n");
	return 0;
}





AXIS::AXIS()
{
	initiate();
}

AXIS::~AXIS()
{

}

void   AXIS::initiate()
{
	_dof = 0;
	_isAxisMode = true;
	_tracType = tracPtp;

	memset(_v0,0,sizeof(_v0));
	memset(_vf,0,sizeof(_vf));
	memset(_velAxisLimits,0,sizeof(_velAxisLimits));
	memset(_accAxisLimits,0,sizeof(_accAxisLimits));
	memset(_jerkAxisLimits,0,sizeof(_jerkAxisLimits));
}

void   AXIS::_searchMax() 
{
	// TODO
}

int    AXIS::getDOF() const
{
	return _dof;
}

void   AXIS::_intArcLength()
{
	// do nothing
}

bool   AXIS::_planTrajectory()
{
	// declare flags
	_isAxisMode = true;
	_tracType = tracPtp;

	// avoid non-necessary planning
	if (!_newParam)
	{
		return _isValid;
	}

	// check DOF
	if (_dof <= 0)
	{
		DUMP_ERROR("ERR: please [re-]call <setAxisLimits> first in <AXIS::planTrajectory>!\n");
		_isValid = false;
		return false;
	}

	// check limits
	for (int i=0; i<_dof; i++)
	{
		if (_velAxisLimits[i] <= 0)
		{
			DUMP_ERROR("ERR: please [re-]call <setAxisLimits> first in <AXIS::planTrajectory>!\n");
			_isValid = false;
			return false;
		}

		if (_accAxisLimits[i] <= 0)
		{
			DUMP_ERROR("ERR: please [re-]call <setAxisLimits> first in <AXIS::planTrajectory>!\n");
			_isValid = false;
			return false;
		}

		if (_jerkAxisLimits[i] <= 0)
		{
			DUMP_ERROR("ERR: please [re-]call <setAxisLimits> first in <AXIS::planTrajectory>!\n");
			_isValid = false;
			return false;
		}
	}

	// plan
	double duration = 0; 
	for (int i=0; i<_dof; i++)
	{
		_dstPlanner[i].setLimit(_velAxisLimits[i],_accAxisLimits[i],_jerkAxisLimits[i]);
		if (!_dstPlanner[i].planProfile(0,_startPos[i],_targetPos[i],_v0[i],_vf[i]))
		{
			_isValid = false;
			return false;
		}

		if (_dstPlanner[i].getDuration() > duration)
		{
			duration = _dstPlanner[i].getDuration();
		}
	}

	// synchronize
	for (int i=0; i<_dof; i++)
	{
		if (duration > _dstPlanner[i].getDuration())
		{
			if(!_dstPlanner[i].scaleTo(duration))
			{
				_isValid = false;
				return false;
			}
		}
	}

	_profile = disN(_startPos,_targetPos,_dof);
	_targetTime = _startTime + duration;
	_chordLength = _profile;
	_newParam = false;
	_firstArc = true;
	_firstMax = true;
	_isValid = true;

	return true;
}

bool   AXIS::stopTrajectory(double t)
{
	if (!_isValid)
	{
		DUMP_ERROR("ERR: the trajectory is invalid in <AXIS::rePlanAxisTrajectory>!\n");
		return false;
	}

	if (ABS(_profile) <= 0)
	{
		_targetTime = t;
		return true;
	}

	bool ret = true;
	double duration = 0;
	DS_or_Trap tempInterp[MAX_DOF];
	for (int i=0; i<_dof; i++)
	{
		// current status of the profile
		double a_limit = ABS(_accAxisLimits[i]);
		double j_limit = ABS(_jerkAxisLimits[i]);
		double v  = _dstPlanner[i].vel(t - _startTime);
		double a  = _dstPlanner[i].acc(t - _startTime);

		double timeToVelMax = a/j_limit;
		double newVelLimit = ABS(v) + ABS(0.5*a*timeToVelMax);

		// a new profile without a cruising phase
		double Ta;
		if ((newVelLimit - 0) * j_limit < a_limit * a_limit)
		{
			Ta = 2.0*sqrt((newVelLimit - 0)/j_limit);
		}
		else
		{
			Ta = a_limit / j_limit + (newVelLimit - 0)/a_limit;
		}

		tempInterp[i].setLimit(newVelLimit,a_limit,j_limit);
		double deltaP = sign(_targetPos[i]-_startPos[i])*(Ta + PLAN_CYCLE/1000.0)*newVelLimit;

		if (tempInterp[i].planProfile(0,0,deltaP))
		{
			double p0 = _dstPlanner[i].pos(t-_startTime) - tempInterp[i].pos(Ta-timeToVelMax);
			tempInterp[i].planProfile(timeToVelMax-Ta,p0,p0+deltaP);
			if (tempInterp[i].isValid())
			{
				if (duration < timeToVelMax - Ta + tempInterp[i].getDuration())
				{
					duration = timeToVelMax - Ta + tempInterp[i].getDuration();
				}
				continue;
			}
			else
			{
				ret = false;
				return false;
			}
		}
		else
		{
			ret = false;
			return false;
		}
	}

	if (ret) // all new profile succeed, update
	{
		for (int i=0; i<_dof; i++)
		{
			_dstPlanner[i] = tempInterp[i];
			_targetPos[i] = _dstPlanner[i].pos(_targetTime-t);
		}
		_profile = disN(_startPos,_targetPos,_dof);
		_targetTime = t + duration;
		_startTime = t;
		return true;
	}

	return ret;
}

bool   AXIS::isInCruisingPhase(double t)
{
	for (int i=0; i<_dof; i++)
	{
		// check acceleration
		if (ABS(_dstPlanner[i].acc(t - _startTime)) > ALMOST_ZERO)
		{
			return false;
		}
	}

	return true;
}

void   AXIS::_setAxisStartVel(double vel[])
{
	for (int i=0; i<_dof; i++)
	{
		if (_v0[i] != vel[i])
		{
			_newParam = true;
			_isValid = false;
			_v0[i] = vel[0];
		}
	}
}

void   AXIS::_setAxisTargetVel(double vel[])
{
	for (int i=0; i<_dof; i++)
	{
		if (_vf[i] != vel[i])
		{
			_newParam = true;
			_isValid = false;
			_vf[i] = vel[0];
		}
	}
}

double AXIS::pos(double t, double pos[]) const
{
	t = t - _startTime;
	if (t<=0)
	{
		t = 0;
	}
	if (t>getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <AXIS::pos>!\n");
	}

	if (_isValid)
	{
		for (int i=0; i<_dof; i++)
		{
			pos[i] = _dstPlanner[i].pos(t);
		}

		return (getDuration()>0)?t/getDuration():1;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <AXIS::pos>!\n");
	}
	return -1;
}

double AXIS::vel(double t, double vel[]) const
{
	t = t - _startTime;
	if (t<=0)
	{
		t = 0;
	}
	if (t>getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <AXIS::vel>!\n");
	}

	if (_isValid)
	{
		for (int i=0; i<_dof; i++)
		{
			vel[i] = _dstPlanner[i].vel(t);
		}

		return (getDuration()>0)?t/getDuration():1;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <AXIS::vel>!\n");
	}
	return -1;
}

double AXIS::acc(double t, double acc[]) const
{
	t = t - _startTime;
	if (t<=0)
	{
		t = 0;
	}
	if (t>getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <AXIS::acc>!\n");
	}

	if (_isValid)
	{
		for (int i=0; i<_dof; i++)
		{
			acc[i] = _dstPlanner[i].acc(t);
		}

		return (getDuration()>0)?t/getDuration():1;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <AXIS::acc>!\n");
	}
	return -1;
}

double AXIS::jerk(double t, double jerk[]) const
{
	t = t - _startTime;
	if (t<=0)
	{
		t = 0;
	}
	if (t>getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <AXIS::jerk>!\n");
	}

	if (_isValid)
	{
		for (int i=0; i<_dof; i++)
		{
			jerk[i] = _dstPlanner[i].jerk(t);
		}

		return (getDuration()>0)?t/getDuration():1;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <AXIS::jerk>!\n");
	}
	return -1;
}

bool   AXIS::scaleToDuration(double newDuration)
{
	// deal with new set parameters
	if (_newParam)
	{
		_planTrajectory();
	}

	if (_isValid)
	{
		if (newDuration <= getDuration())
		{
			DUMP_ERROR("ERR: new duration must be longer in <AXIS::scaleToDuration>!\n");
			return false;
		}

		// clear flags of re-planning results
		bool ret = true;
		for (int i=0; i<_dof; i++)
		{
			ret = _dstPlanner[i].scaleTo(newDuration);

			if (!ret)
			{
				DUMP_ERROR("ERR: trajectory is inValid in <AXIS::scaleToDuration>!\n");
				return ret;
			}
			else
			{
				continue;
			}
		}

		_firstMax = true;

		_targetTime = _startTime + newDuration;

		return ret;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is inValid in <AXIS::scaleToDuration>!\n");
	}

	return _isValid;
}

double AXIS::tangent(double t, double tangent[]) const
{
	DUMP_WARNING("WARNING: tangent vector of AXIS is not available!\n");
	return vel(t,tangent);
}

int    AXIS::rePlanTrajectory(double t,double newVelLimit)
{
	DUMP_ERROR("ERR: please use <rePlanAxisTrajectory> instead in <AXIS::rePlanTrajectory> !\n");
	return ERR_REP_NOTYET;
}

int    AXIS::rePlanAxisTrajectory(double t,double newVelLimit[])
{
	if (!_isValid)
	{
		DUMP_ERROR("ERR: the trajectory is invalid in <AXIS::rePlanAxisTrajectory>!\n");
		return ERR_REP_FAILED;
	}

	for (int i=0; i<_dof; i++)
	{
		// check new limits
		if (ABS(newVelLimit[i]) <= 0 )
		{
			DUMP_ERROR("ERR: please check the new set limits in <AXIS::rePlanTrajectory>!\n");
			return ERR_REP_FAILED;
		}
		if (ABS(ABS(newVelLimit[i]) - ABS(_dstPlanner[i].getMaxVel())) < ALMOST_ZERO)
		{
			continue;
		}

		// check acceleration
		if (ABS(_dstPlanner[i].acc(t - _startTime)) > ALMOST_ZERO)
		{
			return ERR_REP_NOTYET; // only in cruising phase can re-plan
		}
	}

	double duration = 0;
	DS_or_Trap tempInterp[MAX_DOF];
	for (int i=0; i<_dof; i++)
	{
		// re-planning
		double alimit = ABS(_accAxisLimits[i]);
		double jlimit = ABS(_jerkAxisLimits[i]);

		if (alimit < ALMOST_ZERO || jlimit < ALMOST_ZERO)
		{   // ????
			tempInterp[i] = _dstPlanner[i];
			continue;
		}

		double p0 = _dstPlanner[i].pos(t - _startTime);
		double v0 = _dstPlanner[i].vel(t - _startTime);
		tempInterp[i].setLimit(ABS(newVelLimit[i]),ABS(alimit),ABS(jlimit));
		if (tempInterp[i].planProfile(0,p0,_targetPos[i],v0))
		{	// re-Plan succeed
			if (duration < tempInterp[i].getDuration())
			{
				duration = tempInterp[i].getDuration();
			}
			continue;
		}
		else // re-Plan failed, recover
		{
			return ERR_REP_FAILED;
		}
	}

	// synchronize
	for (int i=0; i<_dof; i++)
	{
		if (tempInterp[i].getDuration() < duration)
		{
			if (!tempInterp[i].scaleTo(duration))
			{
				return ERR_REP_FAILED;
			}
		}
	}

	// update
	for (int i=0; i<_dof; i++)
	{
		_dstPlanner[i] = tempInterp[i];
	}
	_targetTime = t + duration;
	_newParam = false;
	_firstMax = true;
	_isValid = true;
	_startTime = t;

	return REP_SUCCEED;
}

double AXIS::move(double t, double pos[], double vel[], double acc[]) const
{
	this->pos(t,pos);
	this->vel(t,vel);

	return	this->acc(t,acc);
}

void   AXIS::getAxisLimits(double vLimit[], double aLimit[], double jLimit[]) const
{
	memcpy(vLimit,_velAxisLimits,sizeof(double)*_dof);
	memcpy(aLimit,_accAxisLimits,sizeof(double)*_dof);
	memcpy(jLimit,_jerkAxisLimits,sizeof(double)*_dof);
}

void   AXIS::setAxisLimits(int dof, double vLimit[],double aLimit[],double jLimit[])
{
	if (dof <= 0)
	{
		DUMP_ERROR("ERR: the input dof if invalid in <AXIS::setAxisLimits>\n");
	}

	if (_dof != ABS(dof))
	{
		_newParam = true;
		_isValid = false;
		_dof = ABS(dof);
	}

	for (int i=0; i<ABS(dof); i++)
	{
		if (_velAxisLimits[i] != ABS(vLimit[i]))
		{
			_newParam = true;
			_isValid = false;
			_velAxisLimits[i] = ABS(vLimit[i]);
		}

		if (_accAxisLimits[i] != ABS(aLimit[i]))
		{
			_newParam = true;
			_isValid = false;
			_accAxisLimits[i] = ABS(aLimit[i]);
		}

		if (_jerkAxisLimits[i] != ABS(jLimit[i]))
		{
			_newParam = true;
			_isValid = false;
			_jerkAxisLimits[i] = ABS(jLimit[i]);
		}
	}
}

bool   AXIS::planTrajectory(double t0, double p0[], double pf[],double v0[], double vf[])
{
	_setStart(p0);
	_setTarget(pf);
	_setStartTime(t0);

	if (v0!=nullptr)
	{
		_setAxisStartVel(v0);
	}

	if (vf != nullptr)
	{
		_setAxisTargetVel(vf);
	}

	return _planTrajectory();
}






LINE:: LINE()
{
	initiate();
}

LINE::~LINE()
{
}

void   LINE::initiate()
{
	_velMax = 0.0;
	_accMax = 0.0;
	_jerkMax = 0.0;

	_velLimit = 0.0;
	_accLimit = 0.0;
	_jerkLimit = 0.0;

	_profile = 0.0;
	_startTime = 0.0;
	_targetTime = 0.0;
	_chordLength = 0.0;

	_isValid = false;
	_newParam = true;
	_firstArc = true;
	_firstMax = true;
	_isAxisMode = false;

	_dof = TRANS_D;
	_tracType = tracLine;
	_interpolater.initiate();

	memset(_xyzScale, 0, sizeof(_xyzScale));
	memset(_startPos, 0, sizeof(_startPos));
	memset(_targetPos, 0, sizeof(_startPos));
}

void   LINE::_searchMax()
{
	if (_interpolater.isValid())
	{
		_velMax = ABS(_interpolater.getMaxVel());
		_accMax = ABS(_interpolater.getMaxAcc());
		_jerkMax = ABS(_interpolater.getMaxJerk());
	}
	else
	{
		_velMax = -0;
		_accMax = -0;
		_jerkMax = -0;
	}
}

void   LINE::_intArcLength()
{
	// for line, do nothing
}

bool   LINE::_planTrajectory()
{
	// declare flags
	_dof = TRANS_D;
	_isAxisMode = false;
	_tracType = tracLine;

	// avoiding non-necessary re-planning
	if (!_newParam)
	{
		return _isValid;
	}

	// check limits
	if (_velLimit <= 0 || _accLimit <= 0 || _jerkLimit <= 0)
	{
		DUMP_ERROR("ERR: please check/set the limits in <LINE::planTrajectory>!\n");
		_isValid = false;
		return false;
	}

	// clear results
	_isValid = false;
	_firstArc = true;
	_firstMax = true;
	_newParam = false;

	// prepare profile
	_profile = disN(_startPos,_targetPos,TRANS_D);
	_chordLength = _profile;
	if (_profile < ALMOST_ZERO)
	{
		_profile = 0;
		memset(_xyzScale, 0, sizeof(_xyzScale));
		DUMP_WARNING("WARNING: the target is too close to the start, ignored in <LINE:planTrajectory>\n");
	}
	else
	{
		for (int i=0; i<TRANS_D; i++)
		{
			_xyzScale[i] = (_targetPos[i] - _startPos[i])/_profile;
		}
	}

	// planning
	_interpolater.setLimit(_velLimit,_accLimit,_jerkLimit);
	_isValid = _interpolater.planProfile(0,0,_profile);
	if (_isValid)
	{
		_targetTime = _startTime + _interpolater.getDuration();
	}

	return _isValid;
}

bool   LINE::stopTrajectory(double t)
{	
	if (t >= _targetTime)
	{
		return true;
	}

	if (ABS(_profile) <= 0 || t <= _startTime)
	{
		_startTime = t;
		_targetTime = t;
		return true;
	}

	// current status of the profile
	double a_limit = ABS(_accLimit);
	double j_limit = ABS(_jerkLimit);
	double v  = _interpolater.vel(t - _startTime);
	double a  = _interpolater.acc(t - _startTime);

	double timeToVelMax = a/j_limit;
	double newVelLimit = ABS(v) + ABS(0.5*a*timeToVelMax);

	// new profile without cruising phase
	double Ta;  
	if ((newVelLimit - 0) * j_limit < a_limit * a_limit)
	{
		Ta = 2.0*sqrt((newVelLimit - 0)/j_limit);
	}
	else
	{
		Ta = a_limit / j_limit + (newVelLimit - 0)/a_limit;
	}

	DS_or_Trap tempInterP; tempInterP.initiate();
	tempInterP.setLimit(newVelLimit,a_limit,j_limit);
	double deltaP = (Ta + PLAN_CYCLE/1000.0)*newVelLimit;

	if (tempInterP.planProfile(0,0,deltaP))
	{
		double p0 = _interpolater.pos(t-_startTime) - tempInterP.pos(Ta-timeToVelMax);
		tempInterP.planProfile(0,p0,p0+deltaP);
		if (tempInterP.isValid())
		{
			_targetTime = t + timeToVelMax - Ta + tempInterP.getDuration();
			_startTime = t + timeToVelMax - Ta;
			_interpolater = tempInterP;
			_profile  = p0 + deltaP;
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool   LINE::isInCruisingPhase(double t)
{
	// check acceleration, re-plan in cruising phase only
	if (ABS(_interpolater.acc(t - _startTime)) > ALMOST_ZERO)
	{
		return false; 
	}
	else
	{
		return true;
	}
}

double LINE::pos(double t, double p[]) const
{
	t = t - _startTime;
	if (t <= 0)
	{
		t = 0;
	}
	if (t >= getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <LINE::pos>!\n");
	}

	if (_isValid)
	{
		double tempP = _interpolater.pos(t);
		for (int i=0; i<TRANS_D; i++)
		{
			p[i] = _xyzScale[i] * tempP + _startPos[i];
		}

		return (getDuration()>0)?(t/getDuration()):1;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <LINE::pos>!\n");
	}
	return -1;
}

double LINE::vel(double t, double v[]) const
{
	t = t - _startTime;
	if (t <= 0)
	{
		t = 0;
	}
	if (t >= getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <LINE::vel>!\n");
	}

	if (_isValid)
	{
		double tempV = _interpolater.vel(t);
		for (int i=0; i<TRANS_D; i++)
		{
			v[i] = _xyzScale[i] * tempV;
		}

		return (getDuration()>0) ? (t / getDuration()) : 1;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <LINE::vel>!\n");
	}
	return -1;
}

double LINE::acc(double t, double a[]) const
{
	t = t - _startTime;
	if (t <= 0)
	{
		t = 0;
	}
	if (t >= getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <LINE::acc>!\n");
	}

	if (_isValid)
	{
		double tempA = _interpolater.acc(t);
		for (int i=0; i<TRANS_D; i++)
		{
			a[i] = _xyzScale[i] * tempA;
		}

		return (getDuration()>0) ? (t / getDuration()) : 1;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <LINE:acc>!\n");
	}
	return -1;
}

double LINE::jerk(double t, double j[]) const
{
	t = t - _startTime;
	if (t <= 0)
	{
		t = 0;
	}
	if (t >= getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <LINE::jerk>!\n");
	}

	if (_isValid)
	{
		double tempJ = _interpolater.jerk(t);
		for (int i=0; i<TRANS_D; i++)
		{
			j[i] = _xyzScale[i] * tempJ;
		}

		return (getDuration()>0) ? (t / getDuration()) : 1;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <LINE::jerk>!\n");
	}
	return -1;
}

bool   LINE::scaleToDuration(double newDuration)
{
	// deal with new set parameters
	if (_newParam)
	{
		_planTrajectory();
	}

	if (_isValid)
	{
		if (newDuration <= getDuration())
		{
			DUMP_ERROR("ERR: new duration must be longer in <LINE::scaleToDuration>!\n");
			return _isValid;
		}

		if (_interpolater.scaleTo(newDuration))
		{
			_isValid = true;
			_firstMax = true;
			_targetTime = _startTime + newDuration;
		}
		else
		{
			_isValid = false;
			DUMP_ERROR("ERR: trajectory is inValid in <LINE::scaleToDuration>!\n");
		}

		return _isValid;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is inValid in <LINE::scaleToDuration>!\n");
	}

	return _isValid;
}

double LINE::tangent(double t, double tangent[]) const
{
	memcpy(tangent, _xyzScale, sizeof(double)*TRANS_D);

	return (getDuration() > 0) ? ((t - _startTime) / getDuration()) : 1;
}

int    LINE::rePlanTrajectory(double t,double newVelLimit)
{
	// check new limits
	if (ABS(newVelLimit) <= 0 )
	{
		DUMP_ERROR("ERR: please check the new set limits in <LINE::rePlanTrajectory>!\n");
		return ERR_REP_FAILED;
	}
	if (ABS(_profile) < ALMOST_ZERO)
	{
		_targetTime = _startTime;
		return REP_SUCCEED;
	}

	// check acceleration
	if (!isInCruisingPhase(t))
	{
		return ERR_REP_NOTYET; // re-plan in cruising phase only
	}

	// check maximum velocity
	if (ABS(ABS(newVelLimit) - ABS(_interpolater.vel(t - _startTime))) < ALMOST_ZERO)
	{
		return REP_SUCCEED;
	}

	// re-planning
	DS_or_Trap tempInterp;
	tempInterp.initiate();
	double alimit = ABS(_accLimit);
	double jlimit = ABS(_jerkLimit);
	double p0 = _interpolater.pos(t - _startTime);
	double v0 = _interpolater.vel(t - _startTime);
	tempInterp.setLimit(ABS(newVelLimit),ABS(alimit),ABS(jlimit));
	if (tempInterp.planProfile(0,p0,_profile,v0))
	{	// re-Plan succeed, saving parameters
		_targetTime = t + tempInterp.getDuration();
		setLimit(newVelLimit,alimit,jlimit); 
		_interpolater = tempInterp;
		_newParam = false;
		_firstMax = true;
		_isValid = true;
		_startTime = t;
		return REP_SUCCEED;
	}
	else // re-Plan failed, recover
	{
		return ERR_REP_FAILED;
	}
}

bool   LINE::planTrajectory(double t0, double p0[], double pf[])
{
	_setStart(p0);
	_setTarget(pf);
	_setStartTime(t0);

	return _planTrajectory();
}

double LINE::move(double t,double pos[], double vel[], double acc[]) const
{
	this->pos(t,pos);
	this->vel(t,vel);
	return this->acc(t,acc);
}






CIRCLE::CIRCLE()
{
	initiate();
}

CIRCLE::~CIRCLE()
{
}

void   CIRCLE::initiate()
{
	_theta = 0.0;
	_radius = 0.0;

	_velMax = 0.0;
	_accMax = 0.0;
	_jerkMax = 0.0;

	_velLimit = 0.0;
	_accLimit = 0.0;
	_jerkLimit = 0.0;

	_startTime = 0.0;
	_targetTime = 0.0;

	_profile = 0.0;
	_chordLength = 0.0;

	_isValid = false;
	_newParam = true;
	_firstArc = true;
	_firstMax = true;
	_isAxisMode = false;

	_dof = TRANS_D;
	_tracType = tracCircle;
	_interpolater.initiate();

	memset(_matrix, 0, sizeof(_matrix));
	memset(_normal, 0, sizeof(_normal));
	memset(_center, 0, sizeof(_center));
	memset(_startPos, 0, sizeof(_startPos));
	memset(_targetPos, 0, sizeof(_startPos));

	for (int i = 0; i < TRANS_D; i++)
	{
		_matrix[i][i] = 1.0;
	}
}

void   CIRCLE::_searchMax()
{
	if (_interpolater.isValid())
	{	
		_velMax = ABS(_interpolater.getMaxVel());
		_accMax = ABS(_interpolater.getMaxAcc());
		_jerkMax = ABS(_interpolater.getMaxJerk());
	}
	else
	{
		_velMax = -0;
		_accMax = -0;
		_jerkMax = -0;
	}
}

void   CIRCLE::_intArcLength()
{
	// for CIRCLE, do nothing
}

bool   CIRCLE::_planTrajectory()
{
	// declare flags
	_dof = TRANS_D;
	_isAxisMode = false;
	_tracType = tracCircle;

	// avoiding non-necessary re-planning
	if (!_newParam)
	{
		return _isValid;
	}

	// check limits
	if (_velLimit <= 0 || _accLimit <= 0 || _jerkLimit <= 0)
	{
		DUMP_ERROR("ERR: please check/set the limits in <CIRCLE::planTrajectory>!\n");
		_isValid = false;
		return false;
	}

	// clear results
	_isValid = false;
	_firstArc = true;
	_firstMax = true;
	_newParam = false;

	// prepare and check
	_radius = disN(_startPos, _center, TRANS_D);
	_profile = _radius*_theta;
	_chordLength = _profile;

	if (_radius < ALMOST_ZERO && ABS(_theta) > ALMOST_ZERO)
	{
		DUMP_ERROR("ERR: center angle is no zero with a zero radius in <CIRCLE>, please use <ROTATION> instead!\n");
		_isValid = false;
		return _isValid;
	}

	if (normN(_normal,TRANS_D) < ALMOST_ZERO)
	{
		DUMP_ERROR("ERR: the normal vector of the CIRCLE is invalid, please check in <CIRCLE:planTrajectory>\n");
		_isValid = false;
		return _isValid;
	}

	// transfer matrix
	double x[3], y[3], z[3];
	for (int i = 0; i < 3; i++)
	{
		z[i] = _normal[i] / normN(_normal, TRANS_D);
		if (_radius < ALMOST_ZERO)
		{
			x[i] = 0;
		}
		else
		{
			x[i] = (_startPos[i] - _center[i]) / _radius;
		}
	}

	cross3(y, z, x);

	for (int i = 0; i < 3; i++)
	{
		_matrix[i][0] = x[i];
		_matrix[i][1] = y[i];
		_matrix[i][2] = z[i];
	}

	// planning
	_interpolater.setLimit(_velLimit,_accLimit,_jerkLimit);
	_interpolater.planProfile(0,0,_profile);

	_isValid = _interpolater.isValid();
	if (_isValid)
	{
		_targetTime = _startTime + _interpolater.getDuration();
		this->pos(_targetTime, _targetPos);
	}

	return _isValid;
}

bool   CIRCLE::stopTrajectory(double t)
{	
	if (t >= _targetTime)
	{
		return true;
	}

	if (ABS(_profile) <= 0 || t <= _startTime)
	{
		_startTime = t;
		_targetTime = t;
		return true;
	}

	double mT = 0.5*(_targetTime - (t - _startTime));
	double midPoint[TRANS_D]; this->pos(_startTime + mT,midPoint);

	// current status of the profile
	double a_limit = ABS(_accLimit);
	double j_limit = ABS(_jerkLimit);
	double v = _interpolater.vel(t - _startTime);
	double a = _interpolater.acc(t - _startTime);

	double timeToVelMax = a/j_limit;
	double newVelLimit = ABS(v) + ABS(0.5*a*timeToVelMax);

	// new profile without cruising phase
	double Ta;
	if ((newVelLimit - 0) * j_limit < a_limit * a_limit)
	{
		Ta = 2.0*sqrt((newVelLimit - 0)/j_limit);
	}
	else
	{
		Ta = a_limit / j_limit + (newVelLimit - 0)/a_limit;
	}

	DS_or_Trap tempInterP; tempInterP.initiate();
	tempInterP.setLimit(newVelLimit,a_limit,j_limit);
	double deltaP = sign(_profile)*(Ta + PLAN_CYCLE/1000.0)*newVelLimit;

	if (tempInterP.planProfile(0,0,deltaP))
	{
		double p0 = _interpolater.pos(t-_startTime) - tempInterP.pos(Ta-timeToVelMax);
		tempInterP.planProfile(0,p0,p0+deltaP);
		if (tempInterP.isValid())
		{
			_targetTime = t + timeToVelMax - Ta + tempInterP.getDuration();
			memcpy(_middlePoint,midPoint,sizeof(midPoint));
			_startTime = t + timeToVelMax - Ta;
			_newTheta = _profile - p0 - deltaP;
			_interpolater = tempInterP;
			_profile  = p0 + deltaP;
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

void   CIRCLE::_setCenterAngle(double cA)
{
	_theta = cA;
	if (ABS(_theta) < ALMOST_ZERO)
	{
		_theta = 0;
		DUMP_WARNING("WARNING: the center angle of the CIRCLE is too small, ignored in <CIRCLE:planTrajectory>\n");
	}
}

bool   CIRCLE::isInCruisingPhase(double t)
{
	// check acceleration // re-plan in cruising phase only
	if (ABS(_interpolater.acc(t - _startTime)) > ACCURACY_FACTOR)
	{
		return false;
	}
	else
	{
		return true;
	}
}

void   CIRCLE::_setCenterPoint(double cP[])
{
	memcpy(_center, cP, sizeof(_center));
}

void   CIRCLE::_setNormalVector(double nV[])
{
	if (normN(nV,TRANS_D) < ALMOST_ZERO)
	{
		DUMP_ERROR("ERR: the input normal vector is invalid in <CIRCLE::setNormalVector>\n");
	}
	memcpy(_normal, nV, sizeof(_normal));
}

void   CIRCLE::_setTarget(double targetPos[])
{
	DUMP_ERROR("ERR: the target of CIRCLE is calculated by centerPoint, centerAngle and normalVector!\n");
}

void   CIRCLE::getTarget(double targetPos[]) const
{
	if (_newParam)
	{
		DUMP_WARNING("WARNING: Please call <CIRCLE::planTrajectory> first!\n");
		return;
	}

	memcpy(targetPos, _targetPos, sizeof(double)*TRANS_D);
}

bool   CIRCLE::scaleToDuration(double newDuration)
{
	// deal with new set parameters
	if (_newParam)
	{
		_planTrajectory();
	}

	if (_isValid)
	{
		if (newDuration <= getDuration())
		{
			DUMP_ERROR("ERR: new duration must be longer in <CIRCLE::scaleToDuration>!\n");
			return false;
		}

		// clear flags and update results
		if (_interpolater.scaleTo(newDuration))
		{
			_isValid = true;
			_firstMax = true;
			_targetTime = _startTime + newDuration;
		}
		else
		{
			_isValid = false;
			DUMP_ERROR("ERR: trajectory is inValid in <CIRCLE::scaleToDuration>!\n");
		}

		return _isValid;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is inValid in <CIRCLE::scaleToDuration>!\n");
	}

	return _isValid;
}

double CIRCLE::pos(double t,double p[TRANS_D])const
{
	t = t - _startTime;
	if (t <= 0)
	{
		t = 0;
	}
	if (t >= getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <CIRCLE::pos>!\n");
	}

	if (_isValid)
	{
		double tp = _interpolater.pos(t)/_radius;
		for (int i=0; i<TRANS_D; i++)
		{
			p[i] = _center[i] 
			+ _matrix[i][0]*_radius*cos(tp) 
				+ _matrix[i][1]*_radius*sin(tp);
		}

		return (getDuration() > 0)?(t/getDuration()):1;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <CIRCLE::pos>!\n");
	}
	return -1;
}

double CIRCLE::vel(double t,double v[TRANS_D])const
{
	t = t - _startTime;
	if (t <= 0)
	{
		t = 0;
	}
	if (t >= getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <CIRCLE::vel>!\n");
	}

	if (_isValid)
	{
		double tp = _interpolater.pos(t)/_radius;
		double tv = _interpolater.vel(t)/_radius;

		for (int i=0; i<TRANS_D; i++)
		{
			v[i] = (- _matrix[i][0] * _radius*sin(tp)
				+ _matrix[i][1] * _radius*cos(tp))*tv;
		}

		return (getDuration() > 0) ? (t / getDuration()) : 1;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <CIRCLE::vel>!\n");
	}
	return -1;
}

double CIRCLE::acc(double t,double a[TRANS_D]) const
{
	t = t - _startTime;
	if (t <= 0)
	{
		t = 0;
	}
	if (t >= getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <CIRCLE::acc>!\n");
	}

	if (_isValid)
	{
		double tp = _interpolater.pos(t)/_radius;
		double tv = _interpolater.vel(t)/_radius;
		double ta = _interpolater.acc(t)/_radius;

		for (int i=0; i<TRANS_D; i++)
		{
			a[i] = (- _matrix[i][0] * _radius*cos(tp)
				- _matrix[i][1] * _radius*sin(tp))*tv*tv
				+ (- _matrix[i][0] * _radius*sin(tp)
				+ _matrix[i][1] * _radius*cos(tp))*ta;
		}

		return (getDuration() > 0) ? (t / getDuration()) : 1;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <CIRCLE::acc>!\n");
	}
	return -1;
}

double CIRCLE::jerk(double t,double j[TRANS_D]) const
{
	t = t - _startTime;
	if (t <= 0)
	{
		t = 0;
	}
	if (t >= getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <CIRCLE::jerk>!\n");
	}

	if (_isValid)
	{
		double tp = _interpolater.pos(t)/_radius;
		double tv = _interpolater.vel(t)/_radius;
		double ta = _interpolater.acc(t)/_radius;
		double tj = _interpolater.jerk(t)/_radius;

		for (int i=0; i<TRANS_D; i++)
		{
			double dA = (+_matrix[i][0] * sin(tp)
				- _matrix[i][1] * cos(tp))*_radius;
			double dB = (-_matrix[i][0] * cos(tp)
				- _matrix[i][1] * sin(tp))*_radius;
			double A = (-_matrix[i][0] * _radius*cos(tp)
				- _matrix[i][1] * _radius*sin(tp));
			double B = (-_matrix[i][0] * _radius*sin(tp)
				+ _matrix[i][1] * _radius*cos(tp));

			j[i] = 2*A*tv + dA*tv*tv + B*tj + dB*ta;
		}

		return (getDuration() > 0) ? (t / getDuration()) : 1;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <CIRCLE::jerk>!\n");
	}
	return -1;
}

double CIRCLE::tangent(double t, double tangent[]) const
{
	if (!_isValid)
	{
		DUMP_ERROR("ERR: trajectory is invalid in <CIRCLE::tangent>!\n");
		return -1;
	}

	double nowPoint[TRANS_D];
	double ret = this->pos(t, nowPoint);

	double cV[TRANS_D];
	for (int i = 0; i < TRANS_D; i++)
	{
		cV[i] = (nowPoint[i] - _center[i]);
	}

	double angle = PI / 2.0;
	double axis[3] = { _normal[0], _normal[1], _normal[2] };
	double matrix[3][3] = { { 0 } };
	rot2matrix(matrix, axis, &angle);

	M3p3(tangent, matrix, cV);

	double nV = normN(tangent, TRANS_D);
	if (nV > 0)
	{
		for (int i = 0; i < TRANS_D; i++)
		{
			tangent[i] = tangent[i] /nV;
		}
	}

	return ret;
}

int    CIRCLE::rePlanTrajectory(double t,double newVelLimit)
{
	// check new limits
	if (ABS(newVelLimit) <= 0 )
	{
		DUMP_ERROR("ERR: please check the new set limits in <LINE::rePlanTrajectory>!\n");
		return ERR_REP_FAILED;
	}

	if (ABS(_profile) < ALMOST_ZERO)
	{
		_targetTime = _startTime;
		return REP_SUCCEED;
	}

	// check acceleration
	if (!isInCruisingPhase(t))
	{
		return ERR_REP_NOTYET; // re-plan in cruising phase only
	}

	if (ABS(ABS(newVelLimit) - ABS(_interpolater.vel(t - _startTime))) < ALMOST_ZERO)
	{
		return REP_SUCCEED;
	}

	// re-planning
	DS_or_Trap tempInterp;
	tempInterp.initiate();
	double alimit = ABS(_accLimit);
	double jlimit = ABS(_jerkLimit);
	double p0 = _interpolater.pos(t - _startTime);
	double v0 = _interpolater.vel(t - _startTime);
	tempInterp.setLimit(ABS(newVelLimit),ABS(alimit),ABS(jlimit));
	if (tempInterp.planProfile(0,p0,_profile,v0))
	{	// re-Plan succeed, saving parameters
		_targetTime = t + tempInterp.getDuration();
		setLimit(newVelLimit,alimit,jlimit); 
		_interpolater = tempInterp;
		_newParam = false;
		_firstMax = true;
		_isValid = true;
		_startTime = t;
		return REP_SUCCEED;
	}
	else // re-Plan failed, recover
	{
		return ERR_REP_FAILED;
	}
}

double CIRCLE::move(double t,double pos[], double vel[], double acc[]) const
{
	this->pos(t,pos);
	this->vel(t,vel);
	return this->acc(t,acc);
}

double CIRCLE::getNewCenterAngleAndMiddlePointAfterStop(double midPoint[]) const
{
	if (midPoint == nullptr)
	{
		DUMP_ERROR("ERR: the input ptr is invalid in <CIRCLE::getNewCenterAngleAndMiddlePointAfterStop>\n");
		return _newTheta;
	}

	memcpy(midPoint,_middlePoint,sizeof(_middlePoint));

	return _newTheta;
}

bool   CIRCLE::planTrajectory(double t0, double sP[],double mP[],double tP[],int cycle)
{
	double cP[TRANS_D],nV[TRANS_D],cA;
	//cA = circleAngle(sP,mP,tP); cA += sign(cA)*cycle*2.0*PI;
	cA = circleAngle(sP,mP,tP) + cycle*2.0*PI; // more flexible
	circleCenter(cP,sP,mP,tP);
	circleVector(nV,sP,mP,tP);

	return planTrajectory(t0,sP,cP,nV,cA);
}

bool   CIRCLE::planTrajectory(double t0, double sP[],double cP[],double nV[], double cA)
{
	_setStart(sP);
	_setStartTime(t0);
	_setCenterAngle(cA);
	_setCenterPoint(cP);
	_setNormalVector(nV);

	return _planTrajectory();
}




BSPLINE::BSPLINE()
{
	initiate();
}

BSPLINE::~BSPLINE()
{
}

void   BSPLINE::initiate()
{
	_tNum = 2;
	_pNum = 2;
	_bNum = 0;
	_kNum = 0;
	_cpNum = 0;

	_velMax = 0.0;
	_accMax = 0.0;
	_jerkMax = 0.0;

	_velLimit = 0.0;
	_accLimit = 0.0;
	_jerkLimit = 0.0;

	_profile = 0.0;
	_startTime = 0.0;
	_targetTime = 0.0;
	_chordLength = 0.0;

	_stop = false;
	_isValid = false;
	_newParam = true;
	_firstArc = true;
	_firstMax = true;
	_isAxisMode = false;

	_dof = TRANS_D;
	_tracType = tracBspline;

	memset(_t, 0, sizeof(_t));
	memset(_kT, 0, sizeof(_kT));
	memset(_oP, 0, sizeof(_oP));
	memset(_cP, 0, sizeof(_cP));
	memset(_bP, 0, sizeof(_bP));
	memset(_bCP, 0, sizeof(_bCP));
	memset(_sVel, 0, sizeof(_sVel));
	memset(_tVel, 0, sizeof(_sVel));
	memset(_startPos, 0, sizeof(_startPos));
	memset(_targetPos, 0, sizeof(_startPos));
}

void   BSPLINE::_searchMax()
{
	double t0 = 0;
	double tp[TRANS_D] = {0.0};
	double lp[TRANS_D] = {0.0};
	double tv[TRANS_D] = {0.0};
	double ta[TRANS_D] = {0.0};
	memcpy(lp,_startPos,sizeof(double)*TRANS_D);

	while(t0 <= _targetTime-_startTime)
	{
		move(t0,tp,tv,ta);

		if (normN(tv,TRANS_D) > _velMax)
		{
			_velMax = normN(tv,TRANS_D);
		}

		if (normN(ta,TRANS_D) >_accMax)
		{
			_accMax = normN(ta,TRANS_D);
		}

		_profile += disN(lp,tp,TRANS_D);
		memcpy(lp,tp,sizeof(double)*TRANS_D);

		t0 += PLAN_CYCLE/1000.0;
	}

	_firstMax = false;
}

void   BSPLINE::_intArcLength()
{
	if (_firstMax)
	{
		_searchMax();
	}
	_firstMax = false;
	_firstArc = false;
}

double BSPLINE::getChordLength()
{
	double chord = 0;

	for (int i = 0; i < _pNum - 1; i++)
	{
		double temp = 0;
		for (int j = 0; j < 3; j++)
		{
			temp += (_oP[i + 1][j] - _oP[i][j])*(_oP[i + 1][j] - _oP[i][j]);
		}

		if (sqrt(temp) < ALMOST_ZERO)
		{
			DUMP_ERROR("ERR: points sequence is invalid (%dth and %dth points are too close) in <BSPLINE::getChordLength>!\n", i, i + 1);
		}

		chord = chord + sqrt(temp);
	}

	if (chord <= ALMOST_ZERO)
	{
		DUMP_ERROR("ERR: points sequence is invalid in <BSPLINE::getChordLength>!\n");
	}

	return chord;
}

bool   BSPLINE::_planTrajectory()
{
	// declare flags
	_dof = TRANS_D;
	_isAxisMode = false;
	_tracType = tracBspline;

	// avoiding non-necessary re-planning
	if (!_newParam)
	{
		return _isValid;
	}

	// check points
	if (_pNum < 4)
	{
		DUMP_ERROR("ERR: at least 4 points were needed in <BSPLINE::planTrajectory>!\n");
		_isValid = false;
		return _isValid;
	}

	// clear results
	_isValid = false;
	_firstArc = true;
	_firstMax = true;
	_newParam = false;

	// check time sequence
	if (_tNum != _pNum || _tNum == 2)
	{
		if (! _checkTimeSequence())
		{
			_isValid = false;
			return _isValid;
		}
	}
	else
	{
		for (int i=0; i<_tNum-1; i++)
		{
			if (_t[i+1] - _t[i] <= 0)
			{
				DUMP_ERROR("ERR: the user set time sequence is invalid(NOT mono-increasing!) in <BSPLINE::planTrajectory>!\n");
				_isValid = false;
				return _isValid;
			}
		}
	}

	// compute knot vector
	if (! _computeKnotVector())
	{
		DUMP_ERROR("ERR: can NOT generate knot vector in <BSPLINE::planTrajectory>!\n");
		_isValid = false;
		return _isValid;
	}

	// generate control points
	if (! _cubicControlPoint())
	{
		DUMP_ERROR("ERR: can NOT generate control points in <BSPLINE::planTrajectory>!\n");
		_isValid = false;
		return _isValid;
	}

	//generate b-spline points
	if (! _cubicBsplinePoint())
	{
		DUMP_ERROR("ERR: can NOT generate b-spline points in <BSPLINE::planTrajectory>!\n");
		_isValid = false;
		return _isValid;
	}

	_stop = false;
	_isValid = true;
	return _isValid;
}

bool   BSPLINE::_checkTimeSequence()
{
	double T = _targetTime - _startTime;

	if (T <= 0)
	{
		DUMP_ERROR("ERR: target time is invalid in <BSPLINE::planTrajectory>!\n");
		return false;
	}

	_tNum = _pNum;

	_chordLength = 0;
	for(int i=0;i<_pNum-1;i++)
	{
		double tmpD = disN(_oP[i+1],_oP[i],TRANS_D);
		if (tmpD <= ALMOST_ZERO)
		{
			DUMP_ERROR("ERR: points sequence is invalid (%dth and %dth points are too close) in <BSPLINE::planTrajectory>!\n",i,i+1);
			return false;
		}

		_chordLength = _chordLength + tmpD;
	}

	if (_chordLength <= ALMOST_ZERO)
	{
		DUMP_ERROR("ERR: points sequence is invalid in <BSPLINE::planTrajectory>!\n");
		return false;
	}

	for(int i=0;i<_pNum;i++)
	{
		if(i==0)
		{
			_t[i] = 0;
		}
		else if(i==_pNum-1)
		{
			_t[i] = 1;
		}
		else
		{
			_t[i] = _t[i-1] + disN(_oP[i],_oP[i-1],TRANS_D)/_chordLength;
		}
	}

	for(int i=0;i<_pNum;i++)
	{
		_t[i] = _t[i]*T;
	}

	return true;
}

bool   BSPLINE::_computeKnotVector()
{
	// only for odd BSPLINE_ORDER
	_kNum = (_pNum+2*BSPLINE_ORDER);	

	// re-compute the knot vector
	for(int i=0; i<_kNum; i++)
	{
		if(i<=BSPLINE_ORDER)
		{
			_kT[i] = _t[0];
		}
		else if(i>=_pNum+BSPLINE_ORDER-1)
		{
			_kT[i] = _t[_pNum-1];
		}
		else
		{
			_kT[i] = _t[i-BSPLINE_ORDER];
		}
	}

	return true;
}

bool   BSPLINE::_cubicControlPoint()
{
	// only for odd BSPLINE_ORDER
	_cpNum = _pNum + BSPLINE_ORDER - 1;

	// only for 3rd B-spline
	double tempP[4][TRANS_D];
	_3rdControlPoint(tempP);

	// save control points
	for(int i=0;i<_cpNum;i++)
	{
		if(i<2) // only for 3rd B-spline
		{
			memcpy(_cP[i],tempP[i],sizeof(double)*TRANS_D);
		}

		if(i > _cpNum - BSPLINE_ORDER)
		{
			memcpy(_cP[i],tempP[i + BSPLINE_ORDER - _pNum - 1],sizeof(double)*TRANS_D);
		}
	}

	if (_cpNum > 4) // only for 3rd B-spline
	{
		// calculate control point of the intermediate points
		double* up		= (double*)malloc(sizeof(double)*(_pNum-3));
		double*	mid		= (double*)malloc(sizeof(double)*(_pNum-2));
		double* down	= (double*)malloc(sizeof(double)*(_pNum-3));
		double* B_basis = (double*)malloc(sizeof(double)*(BSPLINE_ORDER+1));

		double**f		= (double**)malloc(sizeof(double*)*(TRANS_D));
		double**x		= (double**)malloc(sizeof(double*)*(TRANS_D));
		for (int i=0; i<TRANS_D; i++)
		{
			f[i] = (double*)malloc(sizeof(double)*(_pNum-2));
			x[i] = (double*)malloc(sizeof(double)*(_pNum-2));
		}

		for(int i=2;i<_pNum;i++)
		{
			int k = _iSpan(_t[i-1]);
			_basisfuncs_single(k,_t[i-1],B_basis);

			if(i==2)
			{
				up[i-2] = B_basis[2];
				mid[i-2] = B_basis[1];
				for (int j=0; j<TRANS_D; j++)
				{
					f[j][i-2] = _oP[i-1][j] - B_basis[0]*tempP[1][j];
				}
			}
			else if(i==_pNum-1)
			{
				mid[i-2] = B_basis[1];
				down[i-3] = B_basis[0];
				for (int j=0; j<TRANS_D; j++)
				{
					f[j][i-2] = _oP[i-1][j] - B_basis[2]*tempP[2][j];
				}
			}
			else
			{
				up[i-2] = B_basis[2];
				mid[i-2] = B_basis[1];
				down[i-3] = B_basis[0];

				for (int j=0; j<TRANS_D; j++)
				{
					f[j][i-2] = _oP[i-1][j];
				}
			}
		}

		for (int j=0; j<TRANS_D; j++)
		{
			crout(mid,up,down,f[j],x[j],_pNum-2);
		}

		// save control points
		for(int i=0;i<_cpNum;i++)
		{
			if( i>=2 && i<=_cpNum-BSPLINE_ORDER)
			{
				for (int j=0; j<TRANS_D; j++)
				{
					_cP[i][j] = x[j][i-2];
				}
			}
		}

		free(up);
		free(mid);
		free(down);
		free(B_basis);
		for (int i=0; i<TRANS_D; i++)
		{
			free(f[i]);
			free(x[i]);
		}
		free(f);
		free(x);
	}

	return true;
}

bool   BSPLINE::_cubicBsplinePoint()
{
	_bNum = _pNum-1;
	memset(_bP,0,sizeof(_bP));

	// compute basis function recursively
	for (int i = 0; i<_bNum; i++)
	{
		int i_span = _iSpan(_t[i]);
		_basisfuncs(i_span,_t[i],_B);

		_DS[0][0] = 0.0;
		for(int k=0;k<TRANS_D;k++)
		{
			for(int v=0;v<BSPLINE_ORDER+1;v++)
			{
				_DS[v][k] = 0.0;
				for(int j=0;j<=BSPLINE_ORDER;j++)
				{
					int row = (k*(_kNum-BSPLINE_ORDER-1)+i_span-BSPLINE_ORDER+j)%(_pNum+2);
					int col = (k*(_kNum-BSPLINE_ORDER-1)+i_span-BSPLINE_ORDER+j)/(_pNum+2);
					_DS[v][k] = _DS[v][k] + _cP[row][col]*_B[v*(BSPLINE_ORDER+1)+j];
				}
			}
		}

		for(int j=0;j<BSPLINE_ORDER+1;j++)
		{
			for(int k=0;k<TRANS_D;k++)
			{
				_bP[i][j*TRANS_D+k] = _DS[j][k]/factorial(j);
			}
		}	
	}
	return true;
}

int	   BSPLINE::getPointsNumber() const
{
	return _pNum;
}

bool   BSPLINE::stopTrajectory(double t)
{
	if (t > _targetTime)
	{
		return true;
	}

	if (t <= _startTime)
	{
		_startTime = t;
		_targetTime = t;
		return true;
	}

	double V0[TRANS_D] = {0};
	double A0[TRANS_D] = {0};
	this->vel(t,V0); this->acc(t,A0);

	Trapezoid tempTrap;
	tempTrap.initiate();
	double timeToFullyStop = 0;
	tempTrap.setLimit(ABS(_accLimit),ABS(_jerkLimit),ABS(_jerkLimit)*5.0);
	for (int i=0; i<TRANS_D; i++)
	{
		tempTrap.planProfile(0,norm3(V0),0,A0[i]);
		if (!tempTrap.isValid())
		{
			return false;
		}
		if (timeToFullyStop < tempTrap.getDuration())
		{
			timeToFullyStop = tempTrap.getDuration();
		}
	}

	if (timeToFullyStop > _targetTime - t || timeToFullyStop <= 0)
	{
		return true;
	}

	// update the first point (delete the finished points)
	_span = 0;
	for ( int i=0; i<_pNum-1; i++)
	{
		if (t > _t[i] && t<= _t[i+1])
		{
			_span = i;
			break;
		}
	}

	double tV[TRANS_D] = {0}; // stop, v=0
	double tA[TRANS_D] = {0}; // stop, a=0
	double tP[2][TRANS_D] = {{0}};
	this->pos(t,tP[0]); 
	this->pos(t+timeToFullyStop,tP[1]);

	_startTime = t;
	_ratio = 1.0/timeToFullyStop;
	_targetTime = t + timeToFullyStop;

	for (int i=0;i<TRANS_D;i++)
	{
		_bCP[0][i] =   tP[0][i];
		_bCP[1][i] = ( V0[i]/(5.0*_ratio))         + _bCP[0][i];
		_bCP[2][i] = ( A0[i]/(20.0*_ratio*_ratio)) - _bCP[0][i] + 2.0*_bCP[1][i];

		_bCP[5][i] =    tP[1][i];
		_bCP[4][i] = -( tV[i]/(5.0*_ratio))         + _bCP[5][i];
		_bCP[3][i] =  ( tA[i]/(20.0*_ratio*_ratio)) - _bCP[5][i] + 2.0*_bCP[4][i];
	}

	_stop = true;
	return true;
}

int    BSPLINE::_iSpan(double time)const
{
	int high,low,mid;

	low = BSPLINE_ORDER;
	high = _kNum - BSPLINE_ORDER -1;

	if(time ==_kT[high])
	{
		mid = high;
	}
	else
	{
		mid = (high+low)/2;
		while((time<_kT[mid])||(time>=_kT[mid+1]))
		{
			if(time==_kT[mid+1])
			{
				mid = mid+1;
			}
			else
			{
				if(time>_kT[mid])
				{
					low = mid;
				}
				else
				{
					high = mid;
				}
				mid = (high+low)/2;
			}
		}
	}

	return mid;
}

bool   BSPLINE::isInCruisingPhase(double t)
{
	return false; 
}

int    BSPLINE::_setTargetTime(double tTime)
{
	if (tTime < _startTime)
	{
		DUMP_ERROR("ERR: targetTime should be larger than startTime in <BSPLINE::_setTargetTime>!\n");
		return -1;
	}
	if (_targetTime != tTime)
	{
		_targetTime = tTime;
		_isValid = false;
		_newParam = true;
	}
	return 0;
}

int    BSPLINE::getNowPointIndexAfterStop() const
{
	if (_stop)
	{
		return _span;
	}
	else
	{
		return 0;
	}
}

bool   BSPLINE::scaleToDuration(double newDuration)
{
	// deal with new set parameters
	if (_newParam)
	{
		_planTrajectory();
	}

	if (_isValid)
	{
		// clear result flag of re-planning
		_firstMax = true;
		_firstArc =  true;

		double tempRatio = newDuration/getDuration();

		for (int i=0; i<_tNum; i++)
		{
			_t[i] = (_t[i] - _t[0])*tempRatio + _t[0];
		}

		if (_planTrajectory())
		{
			return _isValid;
		}
		else
		{
			DUMP_ERROR("ERR: scale to new duration failed in <BSPLINE::scaleToDuration>!\n");
			return false;
		}
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is inValid in <BSPLINE::scaleToDuration>!\n");
	}

	return false;
}

double BSPLINE::pos(double t,double p[TRANS_D]) const
{
	double temp1[TRANS_D],temp2[TRANS_D];

	return move(t,p,temp1,temp2);
}

double BSPLINE::vel(double t,double v[TRANS_D]) const
{
	double temp1[TRANS_D],temp2[TRANS_D];
	return move(t,temp1,v,temp2);
}

double BSPLINE::acc(double t,double a[TRANS_D]) const
{
	double temp1[TRANS_D],temp2[TRANS_D];
	return move(t,temp1,temp2,a);
}

double BSPLINE::jerk(double t,double j[TRANS_D]) const
{
	DUMP_WARNING("WARNING: in 3rd order B-spline, only pos/vel/acc are supported!\n");
	memset(j,-1,sizeof(double)*TRANS_D);
	return -1;
}

bool   BSPLINE::_3rdControlPoint(double cp[4][TRANS_D])
{
	for(int i=0;i<TRANS_D;i++)
	{
		cp[0][i] = _oP[0][i];
		cp[1][i] = _oP[0][i] + (_kT[BSPLINE_ORDER+1]-_kT[BSPLINE_ORDER])*_sVel[i]/3.0;
		cp[2][i] = _oP[_pNum-1][i] - (_kT[_pNum+BSPLINE_ORDER-1]-_kT[_pNum+BSPLINE_ORDER-2])*_tVel[i]/3.0;
		cp[3][i] = _oP[_pNum-1][i];
	}

	return true;
}

double BSPLINE::tangent(double t, double tangent[]) const
{
	if (!_isValid)
	{
		DUMP_ERROR("ERR: the trajectory is invalid in <BSPLINE::tangent>!\n");
		return -1;
	}

	t = t - _startTime;
	if (t <= PLAN_CYCLE/1000.0)
	{
		t = PLAN_CYCLE/1000.0;
	}
	if (t >= _targetTime - _startTime - PLAN_CYCLE/1000.0)
	{
		t = _targetTime - _startTime - PLAN_CYCLE/1000.0;
	}

	this->vel(t, tangent);

	double tmp = normN(tangent, TRANS_D);
	if (tmp > ALMOST_ZERO)
	{
		for (int i = 0; i < TRANS_D; i++)
		{
			tangent[i] = tangent[i] / tmp;
		}
	}

	return t / getDuration();
}

int	   BSPLINE::getTimeSequence(double timeSequence[]) const
{
	if (nullptr == timeSequence)
	{
		DUMP_ERROR("ERR: input parameter (timeSequence) is nullptr in <BSPLINE::getTimeSequence>!\n");
		return -1;
	}

	if (_tNum != _pNum)
	{
		DUMP_WARNING("WARNING: please process planTrajectory() first in <BSPLINE::getTimeSequence>!\n");
	}

	for (int i=0; i<_tNum; i++)
	{
		timeSequence[i] = _t[i] + _startTime;
	}

	return _tNum;
}

int    BSPLINE::rePlanTrajectory(double t,double newVelLimit)
{
	DUMP_WARNING("WARNING: <BSPLINE> trajectory is based on timeSequence, cannot re-plan on line!\n");
	return REP_SUCCEED;
}

int    BSPLINE::_setTimeSequence(double timeSequence[],int num)
{
	if (nullptr == timeSequence)
	{
		DUMP_ERROR("ERR: input parameter (timeSequence) is nullptr in <BSPLINE::setTimeSequence>!\n");
		return -1;
	}

	if (num < 4)
	{
		DUMP_ERROR("ERR: input parameter (num) is less than 4 in <BSPLINE::setTimeSequence>!\n");
		return -1;
	}

	if (_pNum != num && _pNum > 2)
	{
		DUMP_ERROR("ERR: number of timeSequence does NOT match pointsSequence in <BSPLINE::setTimeSequence>!\n");
		return -1;
	}

	for (int i=0; i<num-1; i++)
	{
		if (timeSequence[i+1] - timeSequence[i] <= 0)
		{
			DUMP_ERROR("ERR: the user set time sequence is invalid(NOT mono-increasing!) in <BSPLINE::setTimeSequence>!\n");
			return -1;
		}
	}

	// save
	_tNum = num;
	_newParam = true;
	memset(_t,0,sizeof(_t));
	for (int i=0; i<_tNum; i++)
	{	// _t starts at 0 and ends with duration
		_t[i] = timeSequence[i] - timeSequence[0];
	}

	// assign start/target time
	_startTime = timeSequence[0]; 
	_targetTime = timeSequence[_tNum-1];

	return _tNum;
}

int    BSPLINE::_basisfuncs(int i_span, double time, double* B)
{
	double acc_,temp,d;
	int j,r,k,s1,s2,rk,pk,j1,j2;

	_DU[0][0] = 1.0;
	for(j=1;j<=BSPLINE_ORDER;j++)
	{
		_DL[j] = time - _kT[i_span+1-j];
		_DR[j] = _kT[i_span+j] - time;
		acc_ = 0.0;
		for(r=0;r<j;r++)
		{
			_DU[j][r] = _DR[r+1] + _DL[j-r];
			temp = _DU[r][j-1]/_DU[j][r];

			_DU[r][j] = acc_ + _DR[r+1]*temp;
			acc_ = _DL[j-r]*temp;
		}
		_DU[j][j] = acc_;
	}

	for(j=0;j<=BSPLINE_ORDER;j++)
	{
		_DB[0][j] = _DU[j][BSPLINE_ORDER];
	}

	for(r=0;r<=BSPLINE_ORDER;r++)
	{
		s1 = 0;
		s2 = 1;
		_DA[0][0] = 1.0;

		for(k=1;k<=BSPLINE_ORDER;k++)
		{
			d = 0.0;
			rk = r - k;
			pk = BSPLINE_ORDER - k;

			if(r>=k)
			{
				_DA[s2][0] = _DA[s1][0]/_DU[pk+1][rk];
				d = _DA[s2][0]*_DU[rk][pk];
			}

			if(rk>=-1)
			{
				j1 = 1;
			}
			else
			{
				j1 = -rk;
			}
			if(r-1<=pk)
			{
				j2 = k-1;
			}
			else
			{
				j2 = BSPLINE_ORDER-r;
			}
			for(j=j1;j<=j2;j++)
			{
				_DA[s2][j] = (_DA[s1][j]-_DA[s1][j-1])/_DU[pk+1][rk+j];
				d += _DA[s2][j]*_DU[rk+j][pk];
			}
			if(r<=pk)
			{
				_DA[s2][k] = -_DA[s1][k-1]/_DU[pk+1][r];
				d += _DA[s2][k]*_DU[r][pk];
			}
			_DB[k][r] = d;
			j=s1;s1=s2;s2=j;
		}
	}

	r=BSPLINE_ORDER;
	for(k=1;k<=BSPLINE_ORDER;k++)
	{
		for(j=0;j<=BSPLINE_ORDER;j++)
		{
			_DB[k][j] *= r;
		}
		r *= (BSPLINE_ORDER-k);
	}

	for(j=0;j<=BSPLINE_ORDER;j++)
	{
		for(k=0;k<=BSPLINE_ORDER;k++)
		{
			B[j*(BSPLINE_ORDER+1)+k] = _DB[j][k];
		}
	}

	return 0;
}

int    BSPLINE::_basisfuncs_single(int i_span, double time, double* B)
{
	int j,r;
	double temp,acc_;

	B[0] = 1;
	for(j=1;j<=BSPLINE_ORDER;j++)
	{
		acc_ = 0.0;

		_DL[j] = time-_kT[i_span+1-j];
		_DR[j] = _kT[i_span+j] - time;

		for(r=0;r<=j-1;r++)
		{
			temp = B[r]/(_DR[r+1]+_DL[j-r]);
			B[r] = acc_ + _DR[r+1]*temp;
			acc_ = _DL[j-r]*temp;
		}
		B[j] = acc_;
	}

	return 0;
}

int	   BSPLINE::getPointsSequence(double pointsSequence[][TRANS_D]) const
{
	if (nullptr == pointsSequence)
	{
		DUMP_ERROR("ERR: input parameter (pointsSequence) is nullptr in <BSPLINE::getPointsSequence>!\n");
		return -1;
	}

	for (int i=0; i<_pNum; i++)
	{
		memcpy(pointsSequence[i],_oP[i],sizeof(double)*TRANS_D);
	}

	return _pNum;
}

int    BSPLINE::_setPointsSequence(double pointsSequence[][TRANS_D],int num)
{
	if (nullptr == pointsSequence)
	{
		DUMP_ERROR("ERR: input parameter (pointsSequence) is nullptr in <BSPLINE::setPointsSequence>!\n");
		return -1;
	}

	if (num < 4)
	{
		DUMP_ERROR("ERR: input parameter (num) is less than 4 in <BSPLINE::setPointsSequence>!\n");
		return -1;
	}

	// save
	_pNum = num;
	_newParam = true;
	memset(_oP,0,sizeof(_oP));
	for (int i=0; i<num; i++)
	{
		memcpy(_oP[i],pointsSequence[i],sizeof(double)*TRANS_D);
	}

	// assign start/target point
	_setStart(pointsSequence[0]);
	_setTarget(pointsSequence[num-1]);

	return _pNum;
}

double BSPLINE::move(double t,double pos[], double vel[], double acc[])const
{
	if (!_isValid)
	{
		DUMP_ERROR("ERR: the trajectory is invalid in <BSPLINE::move>!\n");
		return -1;
	}

	t = t - _startTime;
	if (t <= 0)
	{
		t = 0;
	}
	if ( t >= _targetTime - _startTime)
	{
		t = _targetTime - _startTime;
	}

	if (_stop)
	{
		double u = t/getDuration();
		for (int i=0; i<TRANS_D; i++)
		{
			pos[i] =    1.0*_bCP[0][i]*(1-u)*(1-u)*(1-u)*(1-u)*(1-u) 
				+    5.0*_bCP[1][i]*u    *(1-u)*(1-u)*(1-u)*(1-u)
				+   10.0*_bCP[2][i]*u    *u    *(1-u)*(1-u)*(1-u)
				+   10.0*_bCP[3][i]*u    *u    *u    *(1-u)*(1-u)
				+    5.0*_bCP[4][i]*u    *u    *u    *u    *(1-u)
				+    1.0*_bCP[5][i]*u    *u    *u    *u    *u;

			vel[i] = ( 25.0*_bCP[1][i] -  5.0*_bCP[0][i] 
			-50.0*_bCP[2][i] + 50.0*_bCP[3][i] 
			-25.0*_bCP[4][i] +  5.0*_bCP[5][i]
			)*_ratio*u*u*u*u 
				+ ( 20.0*_bCP[0][i] - 80.0*_bCP[1][i]+ 
				120.0*_bCP[2][i] - 80.0*_bCP[3][i] 
			+20.0*_bCP[4][i]
			)*_ratio*u*u*u 
				+ ( 90.0*_bCP[1][i] - 30.0*_bCP[0][i]
			-90.0*_bCP[2][i] + 30.0*_bCP[3][i]
			)*_ratio*u*u 
				+ ( 20.0*_bCP[0][i] - 40.0*_bCP[1][i] 
			+20.0*_bCP[2][i] 
			)*_ratio*u 
				-    5.0*_bCP[0][i]*_ratio 
				+    5.0*_bCP[1][i]*_ratio;

			acc[i] = (100.0*_bCP[1][i] -  20.0*_bCP[0][i]- 
				200.0*_bCP[2][i] + 200.0*_bCP[3][i]- 
				100.0*_bCP[4][i] +  20.0*_bCP[5][i]
			)*_ratio*_ratio*u*u*u 
				+ ( 60.0*_bCP[0][i] - 240.0*_bCP[1][i]+  
				360.0*_bCP[2][i] - 240.0*_bCP[3][i]+ 
				60.0*_bCP[4][i]
			)*_ratio*_ratio*u*u 
				+ (180.0*_bCP[1][i] -  60.0*_bCP[0][i]- 
				180.0*_bCP[2][i] +  60.0*_bCP[3][i]
			)*_ratio*_ratio*u 
				+   20.0*_bCP[0][i]*_ratio*_ratio 
				-   40.0*_bCP[1][i]*_ratio*_ratio 
				+   20.0*_bCP[2][i]*_ratio*_ratio;
		}

		return t/getDuration();
	}

	int i_span = 0;
	for ( int i=0; i<_pNum-1; i++)
	{
		if (t > _t[i] && t<= _t[i+1])
		{
			i_span = i;
			break;
		}
	}

	double u_p = t - _t[i_span];
	for(int k=0;k<TRANS_D;k++)
	{
		pos[k] = _bP[i_span][0*TRANS_D+k] + _bP[i_span][1*TRANS_D+k]*u_p +     _bP[i_span][2*TRANS_D+k]*u_p*u_p +     _bP[i_span][3*TRANS_D+k]*u_p*u_p*u_p;
		vel[k] =                            _bP[i_span][1*TRANS_D+k]     + 2.0*_bP[i_span][2*TRANS_D+k]*u_p     + 3.0*_bP[i_span][3*TRANS_D+k]*u_p*u_p;
		acc[k] =                                                           2.0*_bP[i_span][2*TRANS_D+k]         + 6.0*_bP[i_span][3*TRANS_D+k]*u_p;
	}

	return t/getDuration();
}

bool   BSPLINE::planTrajectory(double t0, double tf, double p[][TRANS_D], int num)
{
	if (nullptr == p)
	{
		DUMP_ERROR("ERR: input points are nullptr in <BSPLINE::planTrajectory>!\n");
		_isValid = false;
		return false;
	}

	if (num < 4)
	{
		DUMP_ERROR("ERR: input points number are less than 4 in <BSPLINE::planTrajectory>!\n");
		_isValid = false;
		return false;
	}

	// save
	_tNum = 2;
	_setStartTime(t0);
	_setTargetTime(tf);
	if (_setPointsSequence(p,num) != num)
	{
		_isValid = false;
		return _isValid;
	}

	return _planTrajectory();
}

bool   BSPLINE::planTrajectory(double timeSequence[], double pointsSequence[][TRANS_D], int num)
{
	if (nullptr == timeSequence)
	{
		DUMP_ERROR("ERR: input times are nullptr in <BSPLINE::planTrajectory>!\n");
		_isValid = false;
		return false;
	}

	if (nullptr == pointsSequence)
	{
		DUMP_ERROR("ERR: input points are nullptr in <BSPLINE::planTrajectory>!\n");
		_isValid = false;
		return false;
	}

	if (num < 4)
	{
		DUMP_ERROR("ERR: input points number are less than 4 in <BSPLINE::planTrajectory>!\n");
		_isValid = false;
		return false;
	}

	if (_setPointsSequence(pointsSequence,num) != num)
	{
		_isValid = false;
		return _isValid;
	}

	if(_setTimeSequence(timeSequence,num) != num)
	{
		_isValid = false;
		return _isValid;
	}

	return _planTrajectory();
}




BLENDER::BLENDER()
{	
	initiate();
}

BLENDER::~BLENDER()
{
}

void   BLENDER::initiate()
{
	_ratio = 0.0;
	_tTime = 0.0;
	_zArea = 0.0;

	_velMax = 0.0;
	_accMax = 0.0;
	_jerkMax = 0.0;

	_postTime = 0.0;
	_prevTime = 0.0;

	_velLimit = 0.0;
	_accLimit = 0.0;
	_jerkLimit = 0.0;

	_profile = 0.0;
	_startTime = 0.0;
	_targetTime = 0.0;
	_chordLength = 0.0;

	_isValid = false;
	_newParam = true;
	_firstArc = true;
	_firstMax = true;
	_isValid = false;
	_firstMax = true;
	_firstArc = true;
	_newParam = true;
	_isAxisMode = false;

	_dof = TRANS_D;
	_zType = ZONE_RATIO;
	_postTrac = nullptr;
	_prevTrac = nullptr;

	memset(_cP, 0, sizeof(_cP));
	memset(_mP, 0, sizeof(double)*TRANS_D);
	memset(_postP, 0, sizeof(double)*TRANS_D);
	memset(_prevP, 0, sizeof(double)*TRANS_D);
	memset(_postPos, 0, sizeof(double)*TRANS_D);
	memset(_postVel, 0, sizeof(double)*TRANS_D);
	memset(_postAcc, 0, sizeof(double)*TRANS_D);
	memset(_prevPos, 0, sizeof(double)*TRANS_D);
	memset(_prevVel, 0, sizeof(double)*TRANS_D);
	memset(_prevAcc, 0, sizeof(double)*TRANS_D);
}

void   BLENDER::_searchMax()
{
	_velMax = 0.0;
	_accMax = 0.0;
	_jerkMax = 0.0;

	double maxVel = 0.0;
	double maxAcc = 0.0;
	double maxJerk = 0.0;

	double tempVel[TRANS_D];
	double tempAcc[TRANS_D];
	double tempJerk[TRANS_D];

	double tempTime = 0.0;
	while(tempTime <= getDuration())
	{
		vel(tempTime+_startTime,tempVel);
		acc(tempTime+_startTime,tempAcc);
		jerk(tempTime+_startTime,tempJerk);

		maxVel = normN(tempVel,TRANS_D);
		maxAcc = normN(tempAcc,TRANS_D);
		maxJerk = normN(tempJerk,TRANS_D);

		if (maxVel > _velMax)
		{
			_velMax = maxVel;
		}

		if (maxAcc > _accMax)
		{
			_accMax = maxAcc;
		}

		if (maxJerk > _jerkMax)
		{
			_jerkMax = maxJerk;
		}

		tempTime += PLAN_CYCLE/1000.0;
	}

	return ;
}

void   BLENDER::_intArcLength()
{
	double t0 = 0.0;
	double Pos[TRANS_D] = {0.0};
	double oldPos[TRANS_D] = {0.0};

	pos(t0,oldPos);
	_profile = 0.0;
	while(t0 <= _tTime)
	{
		double u = _ratio*t0;
		for (int i=0; i<TRANS_D; i++)
		{
			Pos[i] =  1.0*_cP[0][i]*(1-u)*(1-u)*(1-u)*(1-u)*(1-u) 
				+  5.0*_cP[1][i]*u    *(1-u)*(1-u)*(1-u)*(1-u)
				+ 10.0*_cP[2][i]*u    *u    *(1-u)*(1-u)*(1-u)
				+ 10.0*_cP[3][i]*u    *u    *u    *(1-u)*(1-u)
				+  5.0*_cP[4][i]*u    *u    *u    *u    *(1-u)
				+  1.0*_cP[5][i]*u    *u    *u    *u    *u;
		}

		_profile += disN(oldPos,Pos,TRANS_D);

		memcpy(oldPos,Pos,sizeof(double)*TRANS_D);
		t0 += PLAN_CYCLE/1000.0;
	}
}

bool   BLENDER::planTrajectory()
{
	// declare flags
	_dof = TRANS_D;
	_isAxisMode = false;
	_tracType = tracBlender;

	// avoiding non-necessary re-planning
	if (!_newParam)
	{
		return _isValid;
	}

	// check blend type
	_isValid = _checkBlendType();
	if (!_isValid)
	{
		return _isValid;
	}

	// get enter and out points
	_isValid = _computeTracPoints();
	if (!_isValid)
	{
		return _isValid;
	}

	// get control points
	_isValid = _5thControlPoints();
	_newParam = false;

	return _isValid;
}

bool   BLENDER::_checkBlendType()
{
	if (_prevTrac == nullptr)
	{
		DUMP_ERROR("ERR: please check the prev trajectory in <BLENDER::_checkBlendType>!\n");
		_isValid = false;
		return _isValid;
	}

	if (_postTrac == nullptr)
	{
		DUMP_ERROR("ERR: please check the post trajectory in <BLENDER::_checkBlendType>!\n");
		_isValid = false;
		return _isValid;
	}

	if (!_postTrac->isValid())
	{
		DUMP_ERROR("ERR: the post trajectory is invalid in <BLENDER::_checkBlendType>!\n");
		_isValid = false;
		return false;
	}

	if (!_prevTrac->isValid())
	{
		DUMP_ERROR("ERR: the prev trajectory is invalid in <BLENDER::_checkBlendType>!\n");
		_isValid = false;
		return _isValid;
	}

	if (_zArea < 0)
	{
		DUMP_ERROR("ERR: the transition zone is negative in <BLENDER::_checkBlendType>!\n");
		_isValid = false;
		return false;
	}


	return true;
}

bool   BLENDER::_5thControlPoints()
{
	//--------------how-to-find-the-perfect-cTime-?-------------------------------------
	{ 
		double vf = normN(_postVel,TRANS_D);
		double v0 = normN(_prevVel,TRANS_D);
		double df = disN(_postPos,_mP,TRANS_D);
		double d0 = disN(_prevPos,_mP,TRANS_D);

		if ((v0+vf) == 0)
		{
			_tTime = 0;
			_targetTime = _startTime + _tTime;
			return true;
		}

		_tTime = 2.0*(d0+df)/(v0+vf);
	}
	//--------------how-to-find-the-perfect-cTime-?-------------------------------------

	_targetTime = _startTime + _tTime;

	if ( _tTime == 0)
	{
		return true;
	}

	_ratio = 1.0/_tTime;

	for (int i=0;i<TRANS_D;i++)
	{
		_cP[0][i] =    _prevPos[i];
		_cP[1][i] =  ( _prevVel[i]/(5.0*_ratio))         + _cP[0][i];
		_cP[2][i] =  ( _prevAcc[i]/(20.0*_ratio*_ratio)) - _cP[0][i] + 2.0*_cP[1][i];

		_cP[5][i] =    _postPos[i];
		_cP[4][i] = -( _postVel[i]/(5.0*_ratio))         + _cP[5][i];
		_cP[3][i] =  ( _postAcc[i]/(20.0*_ratio*_ratio)) - _cP[5][i] + 2.0*_cP[4][i];
	}

	return true;
}

TRAC*  BLENDER::getPostTrac() const
{
	return _postTrac;
}

TRAC*  BLENDER::getPrevTrac() const
{
	return _prevTrac;
}

double BLENDER::getPostTime() const
{
	/* get approximate point for time saving */

	double ret = 0.0;
	if (_zType == ZONE_RATIO)
	{
		double blend = _zArea;
		if (blend < 0)
		{
			blend = 0;
		}
		if (blend > 0.5)
		{
			blend = 0.5;
		}
		ret = blend*_postTrac->getDuration() + _postTrac->getStartTime();
	}

	if (_zType == ZONE_LENGTH)
	{
		if (_postTrac->getChordLength() > 0)
		{
			double blend = _zArea/_postTrac->getChordLength();
			if (blend < 0)
			{
				blend = 0;
			}
			if (blend > 0.5)
			{
				blend = 0.5;
			}
			ret = blend*_postTrac->getDuration() + _postTrac->getStartTime();
		}
		else
		{
			ret = _postTrac->getStartTime();
		}
	}

	return ret;
}

double BLENDER::getPrevTime() const
{
	/* get approximate point for time saving */

	double ret = 0.0;
	if (_zType == ZONE_RATIO)
	{
		double blend = _zArea;
		if (blend < 0)
		{
			blend = 0;
		}
		if (blend > 0.5)
		{
			blend = 0.5;
		}
		ret = (1.0 - blend)*_prevTrac->getDuration() + _prevTrac->getStartTime();
	}

	if (_zType == ZONE_LENGTH)
	{
		if (_prevTrac->getChordLength() > 0)
		{
			double blend = _zArea/_prevTrac->getChordLength();
			if (blend < 0)
			{
				blend = 0;
			}
			if (blend > 0.5)
			{
				blend = 0.5;
			}

			ret = (1.0 - blend)*_prevTrac->getDuration() + _prevTrac->getStartTime();
		}
		else
		{
			ret = _prevTrac->getTargetTime();
		}
	}

	return ret;
}

bool   BLENDER::_computeTracPoints()
{
	_postTime = getPostTime();
	_prevTime = getPrevTime();
	_startTime = _prevTime;

	_prevTrac->pos(_prevTime,_prevPos);
	_prevTrac->vel(_prevTime,_prevVel);
	_prevTrac->acc(_prevTime,_prevAcc);

	_postTrac->pos(_postTime,_postPos);
	_postTrac->vel(_postTime,_postVel);
	_postTrac->acc(_postTime,_postAcc);

	distanceOf2LinesIn3D(_prevP,_postP,_prevPos,_prevVel,_postPos,_postVel);

	for (int i=0; i<3; i++)
	{
		_mP[i] = (_prevP[i] + _postP[i])/2.0;
	}

	memcpy(_startPos,_prevPos,sizeof(_prevPos));
	memcpy(_targetPos,_postPos,sizeof(_postPos));

	return true;
}

bool   BLENDER::stopTrajectory(double t)
{
	if (t > _targetTime)
	{
		return true;
	}

	if (t <= _startTime)
	{
		_startTime = t;
		_targetTime = t;
		return true;
	}

	double V0[TRANS_D] = {0};
	double A0[TRANS_D] = {0};
	this->vel(t,V0);this->acc(t,A0);

	Trapezoid tempTrap;
	tempTrap.initiate();
	double timeToFullyStop = 0;
	_prevTrac->getLimit(_velLimit, _accLimit, _jerkLimit);
	tempTrap.setLimit(ABS(_accLimit),ABS(_jerkLimit),ABS(_jerkLimit)*5.0);
	for (int i=0; i<TRANS_D; i++)
	{
		tempTrap.planProfile(0,norm3(V0),0,A0[i],0);
		if (!tempTrap.isValid())
		{
			return false;
		}
		if (timeToFullyStop < tempTrap.getDuration())
		{
			timeToFullyStop = tempTrap.getDuration();
		}
	}

	if (timeToFullyStop > _targetTime - t || timeToFullyStop <= 0)
	{
		return true;
	}

	double tV[TRANS_D] = {0}; // stop, v=0
	double tA[TRANS_D] = {0}; // stop, a=0
	double tP[2][TRANS_D] = {{0}};
	this->pos(t,tP[0]);
	this->pos(t+timeToFullyStop,tP[1]);

	_startTime = t;
	_tTime = timeToFullyStop;
	_targetTime = t + _tTime;
	_ratio = 1.0/timeToFullyStop;

	for (int i=0;i<TRANS_D;i++)
	{
		_cP[0][i] =   tP[0][i];
		_cP[1][i] = ( V0[i]/(5.0*_ratio))         + _cP[0][i];
		_cP[2][i] = ( A0[i]/(20.0*_ratio*_ratio)) - _cP[0][i] + 2.0*_cP[1][i];

		_cP[5][i] =    tP[1][i];
		_cP[4][i] = -( tV[i]/(5.0*_ratio))         + _cP[5][i];
		_cP[3][i] =  ( tA[i]/(20.0*_ratio*_ratio)) - _cP[5][i] + 2.0*_cP[4][i];
	}

	return true;
}

void   BLENDER::setPostTrac(TRAC* postTrac)
{
	if (nullptr == postTrac)
	{
		DUMP_ERROR("ERR: input parameter is nullptr in <BLENDER::setPostTrac>!\n");
		return;
	}
	if (postTrac != _postTrac)
	{
		_isValid = false;
		_newParam = true;
		_postTrac = postTrac;
	}
}

void   BLENDER::setPrevTrac(TRAC* prevTrac)
{
	if (nullptr == prevTrac)
	{
		DUMP_ERROR("ERR: input parameter is nullptr in <BLENDER::setprevTrac>!\n");
		return ;
	}
	if (prevTrac != _prevTrac)
	{
		_isValid = false;
		_newParam = true;
		_prevTrac = prevTrac;
	}
}

bool   BLENDER::scaleToDuration(double newDuration)
{
	if (_newParam)
	{
		planTrajectory();
	}

	if (!_isValid)
	{
		DUMP_ERROR("ERR: the trajectory is inValid in <BLENDER::scaleToDuration>!\n");
		return false;
	}

	if (newDuration < _tTime)
	{
		DUMP_ERROR("ERR: new duration must be longer in <BLENDER::scaleToDuration>!\n");
		return false;
	}

	_targetTime = _startTime + newDuration;

	if (_tTime == 0)
	{
		return _isValid;
	}

	_tTime = newDuration;
	_ratio = 1.0/newDuration;

	for (int i=0;i<TRANS_D;i++)
	{
		_cP[0][i] =    _prevPos[i];
		_cP[1][i] =  ( _prevVel[i]/(5.0*_ratio))         + _cP[0][i];
		_cP[2][i] =  ( _prevAcc[i]/(20.0*_ratio*_ratio)) - _cP[0][i] + 2.0*_cP[1][i];

		_cP[5][i] =    _postPos[i];
		_cP[4][i] = -( _postVel[i]/(5.0*_ratio))         + _cP[5][i];
		_cP[3][i] =  ( _postAcc[i]/(20.0*_ratio*_ratio)) - _cP[5][i] + 2.0*_cP[4][i];
	}

	_isValid = true;

	return _isValid;
}

void   BLENDER::setBlendZone(int zType, double zArea)
{	
	if (_zType != zType || _zArea != ABS(zArea))
	{
		_isValid = false;
		_newParam = true;
		_zType = zType;
		_zArea = ABS(zArea);
	}
}

double BLENDER::pos(double t, double p[TRANS_D]) const
{
	if (isValid())
	{
		t = t - _startTime;
		if (t < 0)
		{
			t = 0;
		}
		if (t > getDuration())
		{
			t = getDuration();
		}

		if ( _tTime == 0)
		{
			memcpy(p,_startPos,sizeof(double)*TRANS_D);
			if (getDuration() > 0)
			{
				return t/getDuration();
			}
			else
			{
				return 1;
			}
		}

		double u = t/getDuration();
		for (int i=0; i<TRANS_D; i++)
		{
			p[i] =    1.0*_cP[0][i]*(1-u)*(1-u)*(1-u)*(1-u)*(1-u) 
				+  5.0*_cP[1][i]*u    *(1-u)*(1-u)*(1-u)*(1-u)
				+ 10.0*_cP[2][i]*u    *u    *(1-u)*(1-u)*(1-u)
				+ 10.0*_cP[3][i]*u    *u    *u    *(1-u)*(1-u)
				+  5.0*_cP[4][i]*u    *u    *u    *u    *(1-u)
				+  1.0*_cP[5][i]*u    *u    *u    *u    *u;
		}
		return t/getDuration();
	}
	else
	{
		DUMP_ERROR("ERR: Blender is invalid in <BLENDER::pos>!\n");
		return 0.0;
	}
}

double BLENDER::vel(double t, double v[TRANS_D]) const
{
	if (isValid())
	{
		t = t - _startTime;
		if (t < 0)
		{
			t = 0;
		}
		if (t > getDuration())
		{
			t = getDuration();
		}

		if ( _tTime == 0)
		{
			memset(v,0,sizeof(sizeof(double)*TRANS_D));
			if (getDuration() > 0)
			{
				return t/getDuration();
			}
			else
			{
				return 1;
			}
		}

		double u = t/getDuration();
		for (int i=0; i<TRANS_D; i++)
		{
			v[i] = (    25.0*_cP[1][i] -  5.0*_cP[0][i] 
			-  50.0*_cP[2][i] + 50.0*_cP[3][i] 
			-  25.0*_cP[4][i] +  5.0*_cP[5][i]
			)*_ratio*u*u*u*u 
				+ (  20.0*_cP[0][i] - 80.0*_cP[1][i] 
			+120.0*_cP[2][i] - 80.0*_cP[3][i] 
			+ 20.0*_cP[4][i]
			)*_ratio*u*u*u 
				+ (  90.0*_cP[1][i] - 30.0*_cP[0][i]
			-90.0*_cP[2][i] + 30.0*_cP[3][i]
			)*_ratio*u*u 
				+ (  20.0*_cP[0][i] - 40.0*_cP[1][i] 
			+20.0*_cP[2][i] 
			)*_ratio*u 
				-     5.0*_cP[0][i]*_ratio 
				+     5.0*_cP[1][i]*_ratio;
		}
		return t/getDuration();
	}
	else
	{
		DUMP_ERROR("ERR: Blender is invalid in <BLENDER::vel>!\n");
		return 0.0;
	}
}

double BLENDER::acc(double t, double a[TRANS_D]) const
{
	if (isValid())
	{
		t = t - _startTime;
		if (t < 0)
		{
			t = 0;
		}
		if (t > getDuration())
		{
			t = getDuration();
		}

		if (_tTime == 0)
		{
			memset(a,0,sizeof(sizeof(double)*TRANS_D));
			if (getDuration() > 0)
			{
				return t/getDuration();
			}
			else
			{
				return 1;
			}
		}

		double u = t/getDuration();
		for (int i=0; i<TRANS_D; i++)
		{
			a[i] = ( 100.0*_cP[1][i] -  20.0*_cP[0][i] 
			- 200.0*_cP[2][i] + 200.0*_cP[3][i] 
			- 100.0*_cP[4][i] +  20.0*_cP[5][i]
			)*_ratio*_ratio*u*u*u 
				+ (   60.0*_cP[0][i] - 240.0*_cP[1][i]  
			+ 360.0*_cP[2][i] - 240.0*_cP[3][i] 
			+  60.0*_cP[4][i]
			)*_ratio*_ratio*u*u 
				+ (   180.0*_cP[1][i] -  60.0*_cP[0][i] 
			- 180.0*_cP[2][i] +  60.0*_cP[3][i]
			)*_ratio*_ratio*u 
				+      20.0*_cP[0][i]*_ratio*_ratio 
				-      40.0*_cP[1][i]*_ratio*_ratio 
				+      20.0*_cP[2][i]*_ratio*_ratio;
		}
		return t/getDuration();
	}
	else
	{
		DUMP_ERROR("ERR: Blender is invalid in <BLENDER::acc>!\n");
		return 0.0;
	}
}

double BLENDER::jerk(double t, double j[TRANS_D]) const
{
	if (isValid())
	{
		t = t - _startTime;
		if (t < 0)
		{
			t = 0;
		}
		if (t > getDuration())
		{
			t = getDuration();
		}

		if (_tTime == 0)
		{
			memset(j,0,sizeof(sizeof(double)*TRANS_D));
			if (getDuration() > 0)
			{
				return t/getDuration();
			}
			else
			{
				return 1;
			}
		}

		double u = t/getDuration();
		for (int i=0; i<TRANS_D; i++)
		{
			j[i] = (  300.0*_cP[1][i] -  60.0*_cP[0][i] 
			- 600.0*_cP[2][i] + 600.0*_cP[3][i] 
			- 300.0*_cP[4][i] +  60.0*_cP[5][i])*_ratio*_ratio*_ratio*u*u 
				+ (   120.0*_cP[0][i] - 480.0*_cP[1][i] 
			+  720.0*_cP[2][i] - 480.0*_cP[3][i] 
			+  120.0*_cP[4][i])*_ratio*_ratio*_ratio*u 
				+     180.0*_cP[1][i] *_ratio*_ratio*_ratio 
				-      60.0*_cP[0][i] *_ratio*_ratio*_ratio 
				-     180.0*_cP[2][i] *_ratio*_ratio*_ratio 
				+      60.0*_cP[3][i] *_ratio*_ratio*_ratio;
		}
		return t/getDuration();
	}
	else
	{
		DUMP_ERROR("ERR: Blender is invalid in <BLENDER::jerk>!\n");
		return 0.0;
	}
}

double BLENDER::tangent(double t, double tangent[]) const
{
	double ret = this->vel(t,tangent);
	double tmp = normN(tangent,TRANS_D);

	if (tmp < ALMOST_ZERO)
	{
		DUMP_WARNING("WARNING: the tangent is invalid in <BLENDER::tangent>!\n");
		return 0;
	}
	else
	{
		tangent[0] /= tmp;
		tangent[1] /= tmp;
		tangent[2] /= tmp;
	}

	return ret;
}

void   BLENDER::getBlendZone(int& zType, double& zArea) const
{
	zType = _zType;
	zArea = _zArea;
}

int    BLENDER::rePlanTrajectory(double t,double newVelLimit)
{
	DUMP_WARNING("WARNING: <BLENDER> is automated planned, can not re-plan on line!\n");
	return REP_SUCCEED;
}

double BLENDER::move(double t,double pos[], double vel[], double acc[]) const
{
	int k = 0;
	if (isValid())
	{
		this->pos(t,pos);
		this->vel(t,vel);
		return this->acc(t,acc);
	}
	else
	{
		DUMP_ERROR("ERR: Blender is invalid in <BLENDER::move>!\n");
		return 0.0;
	}
}





ROTATION::ROTATION()
{
	initiate();
}

ROTATION::~ROTATION()
{
}

void   ROTATION::initiate()
{
	_isValid = false;
	_newParam = true;
	_firstArc = true;
	_firstMax = true;

	_velMax = 0.0;
	_accMax = 0.0;
	_jerkMax = 0.0;

	_velLimit = 0.0;
	_accLimit = 0.0;
	_jerkLimit = 0.0;

	_startTime = 0.0;
	_targetTime = 0.0;

	_profile = 0.0;
	_chordLength = 0.0;

	_dof = ROTATE_D;
	_isAxisMode = false;
	_tracType = tracRotate;

	memset(_axis,0,sizeof(_axis));
	_axis[ROT_INDEX-TRANS_D] = 1.0;

	memset(_startPos, 0, sizeof(_startPos));
	memset(_targetPos, 0, sizeof(_startPos));

	memset(_axis, 0, sizeof(double) * ROTATE_D);
	memset(_startPos, 0, sizeof(double)*ROTATE_D);
	memset(_targetPos, 0, sizeof(double)*ROTATE_D);
}

void   ROTATION::_searchMax()
{
	if (_interpolater.isValid())
	{
		_velMax = ABS(_interpolater.getMaxVel());
		_accMax = ABS(_interpolater.getMaxAcc());
		_jerkMax = ABS(_interpolater.getMaxJerk());
	}
	else
	{
		_velMax = -0;
		_accMax = -0;
		_jerkMax = -0;
	}
}

void   ROTATION::_intArcLength()
{
	// for rotation, do nothing
}

bool   ROTATION::_planTrajectory()
{
	// declare flags
	_dof = TRANS_D;
	_isAxisMode = false;
	_tracType = tracRotate;

	// avoiding non-necessary re-planning
	if (!_newParam)
	{
		DUMP_WARNING("WARNING: no new parameter was set for <ROTATION>!\n");
		return _isValid;
	}

	// check limits
	if (_velLimit <= 0 || _accLimit <= 0 || _jerkLimit <= 0)
	{
		DUMP_ERROR("ERR: please check/set the limit in <ROTATION::planTrajectory>!\n");
		_isValid = false;
		return false;
	}

	// clear results
	_isValid = false;
	_firstArc = true;
	_firstMax = true;
	_newParam = false;

	// prepare and check
	rpy2matrix(_sR,_startPos);
	if (normN(_axis,ROTATE_D) < ALMOST_ZERO)
	{
		DUMP_ERROR("ERR: the reference axis of the ROTATION is invalid, please check in <ROTATION:planTrajectory>\n");
		_isValid = false;
		return _isValid;
	}

	// planning
	_interpolater.setLimit(_velLimit,_accLimit,_jerkLimit);
	_isValid = _interpolater.planProfile(0,0,_profile);
	if (_isValid)
	{
		_targetTime = _startTime + _interpolater.getDuration();
	}

	return _isValid;

}

bool   ROTATION::stopTrajectory(double t)
{
	if (t > _targetTime)
	{
		return true;
	}

	if (ABS(_profile) <= 0 || t <= _startTime)
	{
		_startTime = t;
		_targetTime = t;
		return true;
	}

	// current status of the profile
	double a_limit = ABS(_accLimit);
	double j_limit = ABS(_jerkLimit);
	double v  = _interpolater.vel(t - _startTime);
	double a  = _interpolater.acc(t - _startTime);

	double timeToVelMax = a/j_limit;
	double newVelLimit = ABS(v) + ABS(0.5*a*timeToVelMax);

	// new profile without cruising phase
	double Ta;   
	if ((newVelLimit - 0) * j_limit < a_limit * a_limit)
	{
		Ta = 2.0*sqrt((newVelLimit - 0)/j_limit);
	}
	else
	{
		Ta = a_limit / j_limit + (newVelLimit - 0)/a_limit;
	}
	
	DS_or_Trap tempInterP; tempInterP.initiate();
	tempInterP.setLimit(newVelLimit,a_limit,j_limit);
	double deltaP = sign(_profile)*(Ta + PLAN_CYCLE/1000.0)*newVelLimit;

	if (tempInterP.planProfile(0,0,deltaP))
	{
		double p0 = _interpolater.pos(t-_startTime) - tempInterP.pos(Ta-timeToVelMax);
		tempInterP.planProfile(0,p0,p0+deltaP);
		if (tempInterP.isValid())
		{
			_targetTime = t + timeToVelMax - Ta + tempInterP.getDuration();
			_startTime = t + timeToVelMax - Ta;
			_interpolater = tempInterP;
			_profile  = p0 + deltaP;
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool   ROTATION::isInCruisingPhase(double t)
{
	// check acceleration
	if (ABS(_interpolater.acc(t - _startTime)) > ALMOST_ZERO)
	{
		return false; // only in cruising phase can re-plan
	}
	else
	{
		return true;
	}
}

int    ROTATION::_setTarget(double targetPos[])
{
	double R0[3][3] = {{0.0}};
	double sR[3][3] = {{0.0}};
	double tR[3][3] = {{0.0}};
	double dR[3][3] = {{0.0}};

	memset(_axis,0,sizeof(_axis));
	_axis[ROT_INDEX-TRANS_D] = 1.0;

	rpy2matrix(sR,_startPos);

	rpy2matrix(tR,targetPos);

	Trp3(R0,sR);

	M3p3(dR,R0,tR);

	matrix2rot(_axis,&_profile,dR);

	if (normN(_axis,ROTATE_D) < ALMOST_ZERO )
	{
		_profile = 0.0;
		memset(_axis,0,sizeof(_axis));
		_axis[ROT_INDEX-TRANS_D] = 1.0;

		DUMP_WARNING("WARNING: the target is too close to the start, ignored in <ROTATION>\n");
	}

	for (int i=0; i<ROTATE_D; i++)
	{
		if (_targetPos[i] != targetPos[i])
		{
			_isValid = false;
			_newParam = true;
			_targetPos[i] = targetPos[i];
		}
	}

	return 0;
}

bool   ROTATION::scaleToDuration(double newDuration)
{
	// deal with new set parameters
	if (_newParam)
	{
		_planTrajectory();
	}

	if (_isValid)
	{
		if (newDuration <= getDuration())
		{
			DUMP_ERROR("ERR: new duration must be longer in <ROTATION::scaleToDuration>!\n");
			return false;
		}

		// clear flags of re-planning results
		if (_interpolater.scaleTo(newDuration))
		{
			_isValid = true;
			_firstMax = true;
			_targetTime = _startTime + newDuration;
		}
		else
		{
			_isValid = false;
			DUMP_ERROR("ERR: trajectory is inValid in <ROTATION::scaleToDuration>!\n");
		}

		return _isValid;
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is inValid in <ROTATION::scaleToDuration>!\n");
	}

	return _isValid;
}

double ROTATION::pos(double t, double p[TRANS_D]) const
{
	t = t - _startTime;
	if (t <= 0)
	{
		t = 0;
	}
	if (t >= getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <ROTATION::pos>!\n");
	}

	if (_isValid)
	{
		double tempP = _interpolater.pos(t);

		double sR[3][3] = { { 0.0 } };
		double ret[3][3] = { { 0.0 } };
		double axis[3] = { 0.0, 0, 0 };
		double deltaR[3][3] = { { 0.0 } };
		axis[0] = _axis[0]; axis[1] = _axis[1]; axis[2] = _axis[2];

		for (int i=0; i<3; i++)
		{
			memcpy(sR[i],_sR[i],sizeof(double)*ROTATE_D);
		}

		rot2matrix(deltaR,axis,&tempP);

		M3p3(ret,sR,deltaR);

		matrix2rpy(p,ret);

		return (getDuration()>0?t/getDuration():1);
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <ROTATION::pos>!\n");
	}
	return -1;
}

double ROTATION::vel(double t, double v[TRANS_D]) const
{
	t = t - _startTime;
	if (t <= 0)
	{
		t = 0;
	}
	if (t >= getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <ROTATION::vel>!\n");
	}

	if (_isValid)
	{
		double tempV = _interpolater.vel(t);
		for (int i=0; i<ROTATE_D; i++)
		{
			v[i] = _axis[i]*tempV;
		}

		// it is not in RPY form, because RPY has singular points

		return (getDuration()>0?t/getDuration():1);
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <ROTATION::vel>!\n");
	}
	return -1;
}

double ROTATION::acc(double t, double a[TRANS_D]) const
{
	t = t - _startTime;
	if (t <= 0)
	{
		t = 0;
	}
	if (t >= getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <ROTATION::acc>!\n");
	}

	if (_isValid)
	{
		double tempA = _interpolater.acc(t);
		for (int i=0; i<ROTATE_D; i++)
		{
			a[i] = _axis[i]*tempA;
		}

		// it is not in RPY form, because RPY has singular points

		return (getDuration()>0?t/getDuration():1);
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <ROTATION::acc>!\n");
	}
	return -1;
}

double ROTATION::jerk(double t, double j[TRANS_D]) const
{
	t = t - _startTime;
	if (t <= 0)
	{
		t = 0;
	}
	if (t >= getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <ROTATION::jerk>!\n");
	}

	if (_isValid)
	{
		double tempJ = _interpolater.jerk(t);
		for (int i=0; i<ROTATE_D; i++)
		{
			j[i] = _axis[i]*tempJ;
		}

		// it is not in RPY form, because RPY has singular points

		return (getDuration()>0?t/getDuration():1);
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <ROTATION::jerk>!\n");
	}
	return -1;
}

double ROTATION::tangent(double t, double tangent[]) const
{
	t = t - _startTime;
	if (t <= 0)
	{
		t = 0;
	}
	if (t >= getDuration())
	{
		t = getDuration();
	}

	if (_newParam)
	{
		DUMP_WARNING("WARNING: new parameters have NOT been processed in <ROTATION::tangent>!\n");
	}

	if (_isValid)
	{
		double tempV = _interpolater.vel(t);
		for (int i=0; i<ROTATE_D; i++)
		{
			tangent[i] = _axis[i]*tempV;
		}

		return (getDuration()>0?t/getDuration():1);
	}
	else
	{
		DUMP_ERROR("ERR: trajectory is invalid in <ROTATION::tangent>!\n");
	}
	return -1;
}

int    ROTATION::_setAxisAngle(double axis[], double angle)
{
	double tmp = normN(axis,ROTATE_D);
	if (tmp < ALMOST_ZERO)
	{
		DUMP_ERROR("ERR: the set reference axis of ROTATION is invalid \n");
		return -1;
	}

	for (int i=0; i<ROTATE_D; i++)
	{
		_axis[i] = axis[i]/tmp;
	}

	_profile = angle;

	double sR[3][3];
	rpy2matrix(sR,_startPos);

	double dR[3][3];
	rot2matrix(dR,_axis,&_profile);

	double tR[3][3];
	M3p3(tR,sR,dR);
	matrix2rpy(_targetPos,tR);

	return 0;
}

int    ROTATION::rePlanTrajectory(double t,double newVelLimit)
{
	// check new limits
	if (ABS(newVelLimit) <= 0 )
	{
		DUMP_ERROR("ERR: please check the new set limits in <ROTATION::rePlanTrajectory>!\n");
		return ERR_REP_FAILED;
	}

	if (ABS(_profile) < ALMOST_ZERO)
	{
		_targetTime = _startTime;
		return REP_SUCCEED;
	}

	// check acceleration
	if (!isInCruisingPhase(t))
	{
		return ERR_REP_NOTYET; // only in cruising phase can re-plan
	}

	if (ABS(ABS(newVelLimit) - ABS(_interpolater.vel(t - _startTime))) < ALMOST_ZERO)
	{
		return REP_SUCCEED;
	}

	// re-planning
	DS_or_Trap tempInterp;
	tempInterp.initiate();
	double alimit = ABS(_accLimit);
	double jlimit = ABS(_jerkLimit);
	double p0 = _interpolater.pos(t - _startTime);
	double v0 = _interpolater.vel(t - _startTime);
	tempInterp.setLimit(ABS(newVelLimit),ABS(alimit),ABS(jlimit));
	if (tempInterp.planProfile(0,p0,_profile,v0))
	{	// re-Plan succeed, saving parameters
		_targetTime = t + tempInterp.getDuration();
		setLimit(newVelLimit,alimit,jlimit); 
		_interpolater = tempInterp;
		_newParam = false;
		_firstMax = true;
		_isValid = true;
		_startTime = t;
		return REP_SUCCEED;
	}
	else // re-Plan failed, recover
	{
		return ERR_REP_FAILED;
	}
}

bool   ROTATION::planTrajectory(double t0, double p0[], double pf[])
{
	_setStart(p0);
	_setTarget(pf);
	_setStartTime(t0);

	return _planTrajectory();
}

double ROTATION::move(double t,double pos[], double vel[], double acc[]) const
{
	this->pos(t,pos);
	this->vel(t,vel);
	return this->acc(t,acc);
}

bool   ROTATION::planTrajectory(double t0, double p0[], double axis[], double angle)
{
	_setStart(p0);
	_setStartTime(t0);
	_setAxisAngle(axis,angle);

	return _planTrajectory();
}







MULTI_ROT::MULTI_ROT()
{
	initiate();
};

MULTI_ROT::~MULTI_ROT()
{

}

void   MULTI_ROT::initiate()
{
	_isValid = false;
	_newParam = true;
	_firstArc = true;
	_firstMax = true;

	_velMax = 0.0;
	_accMax = 0.0;
	_jerkMax = 0.0;

	_profile = 0.0;
	_chordLength = 0.0;

	_velLimit = 0.0;
	_accLimit = 0.0;
	_jerkLimit = 0.0;

	_rePlanN = 0;
	_startTime = 0.0;
	_targetTime = 0.0;

	_dof = TRANS_D;
	_tChange = false;
	_isAxisMode = false;
	_tracType = tracRotate;

	for (int i = 0; i < POINTS_MAX_NUM; i++)
	{
		_rotate[i].initiate();
	}

	memset(_startPos, 0, sizeof(_startPos));
	memset(_targetPos, 0, sizeof(_startPos));
}

void   MULTI_ROT::_searchMax()
{
	_firstMax = false;

	for (int i=0; i<_pNum-1; i++)
	{
		if(abs(_rotate[i].getMaxVel()) > _velMax)
		{
			_velMax = abs(_rotate[i].getMaxVel());
		}

		if(abs(_rotate[i].getMaxAcc()) > _accMax)
		{
			_accMax = abs(_rotate[i].getMaxAcc());
		}

		if(abs(_rotate[i].getMaxJerk()) > _jerkMax)
		{
			_jerkMax = abs(_rotate[i].getMaxJerk());
		}
	}

	return ;
}

void   MULTI_ROT::_intArcLength()
{
	for (int i=0; i<_pNum-1; i++)
	{
		_profile += ABS(_angle[i]);
	}
}

bool   MULTI_ROT::_planTrajectory()
{
	// declare flags
	_dof = TRANS_D;
	_isAxisMode = false;
	_tracType = tracRotate;

	// avoiding non-necessary re-planning
	if (!_newParam)
	{
		DUMP_WARNING("WARNING: no new parameter was set for <MULTI_ROT>!\n");
		return _isValid;
	}

	// check limits
	if (_velLimit <= 0 || _accLimit <= 0 || _jerkLimit <= 0)
	{
		DUMP_ERROR("ERR: please check/set the limit in <MULTI_ROT::planTrajectory>!\n");
		_isValid = false;
		return false;
	}

	if ( _pNum < 1)
	{
		DUMP_ERROR("ERR: at least one rpy is needed in MULTI_ROT::planTrajectory>!\n");
		_isValid = false;
		return false;
	}

	// clear flags	
	_isValid = false;
	_firstArc = true;
	_firstMax = true;
	_newParam = false;

	// check t
	if (!_checkTimeT(_t))
	{
		_isValid = false;
		return _isValid;
	}

	// planning
	_rePlanN = 0;
	_startTime = _t[0];
	_targetTime = _t[_pNum-1];
	return _synchronizePlanning();
}

bool   MULTI_ROT::isTimeChanged() const
{
	return _tChange;
}

bool   MULTI_ROT::_synchronizePlanning()
{
	// planning
	_isValid = true;
	for (int i=0; i<_pNum-1; i++)
	{
		_rotate[i].setLimit(_velLimit,_accLimit,_jerkLimit);
		while (true)
		{
			_isValid = _rotate[i].planTrajectory(_t[i],_rpy[i],_axis[i],_angle[i]);
			if (_isValid)
			{
				if ((_t[i+1] - _t[i]) - _rotate[i].getDuration() > PLAN_CYCLE/1000.0)
				{
					_isValid = _rotate[i].scaleToDuration(_t[i+1] - _t[i]);
					break;
				}
				else
				{
					double deltaT = _rotate[i].getDuration() - (_t[i+1] - _t[i]);
					if (deltaT <= PLAN_CYCLE/1000.0)
					{
						break;
					}

					// re-plan time t
					_tChange = true;
					for (int j = i+1; j < _pNum; j++)
					{
						_t[j] = _t[j] + deltaT;
					}
				}
			}
			else
			{
				return _isValid;
			}
		}

		if (!_isValid)
		{
			DUMP_ERROR("ERR: the %d rotation is invalid in <MULTI_ROT::planTrajectory>!\n",i+1);
			return _isValid;
		}
	}

	return _isValid;
}

int    MULTI_ROT::_iSpan(double t) const
{
	if (t <= _t[0])
	{
		t = _t[0];
	}
	if (t >= _t[_pNum-1])
	{
		t = _t[_pNum-1];
	}

	for (int i=0; i<_pNum-1; i++)
	{
		if (t>= _t[i] && t <= _t[i+1])
		{
			return i;
		}
	}
	return _pNum-1;
}

int    MULTI_ROT::getPointsNumber() const
{
	return _pNum;
}

bool   MULTI_ROT::stopTrajectory(double t)
{
	if (t <= _t[0])
	{
		t = _t[0];
	}
	if (t >= _t[_pNum-1])
	{
		t = _t[_pNum-1];
	}
	int index = _iSpan(t);

	bool ret = _rotate[index].stopTrajectory(t);
	if (ret)
	{
		_pNum = index+2;
		_targetTime = _rotate[index].getTargetTime();
		return true;
	}

	return false;
}

bool   MULTI_ROT::isInCruisingPhase(double t)
{
	// re-plan profile on line
	int index = _iSpan(t);

	return _rotate[index].isInCruisingPhase(t);
}

bool   MULTI_ROT::_checkTimeT(double* t) const
{
	for (int i=0; i<_pNum-1; i++)
	{
		if (t[i+1] < t[i])
		{
			return false;
		}
	}

	return true;
}

bool   MULTI_ROT::scaleToDuration(double newDuration)
{
	if (!_isValid)
	{
		DUMP_ERROR("ERR: the spline profile is invalid in <MULTI_ROT::scaleToDuration>!\n");
		return _isValid;
	}

	if (newDuration <= getDuration())
	{
		DUMP_ERROR("ERR: new duration must be longer in <MULTI_ROT::scaleToDuration>!\n");
	}

	// re-plan time t
	_t[_pNum-1] = _t[0] + newDuration;
	if (ABS(_profile) <= 0)
	{
		_isValid = true;
		return true;
	}
	for (int i=1; i<_pNum; i++)
	{
		_t[i] = _t[i-1] + ABS(_angle[i])/ABS(_profile) * (_t[_pNum-1] - _t[0]);
	}

	// re-planning
	for (int i=MAX(_rePlanN-1,0); i<_pNum-1; i++)
	{
		if (i == _rePlanN-1)
		{
			_rotate[i].scaleToDuration(_t[i+1] - _t[i]);
		}
		else
		{
			_rotate[i].scaleToDuration(_t[i+1] - _t[i]);
		}
	}

	return _isValid;
}

int    MULTI_ROT::_setTimeSequence(double t[],int num)
{
	if (num < 2)
	{
		DUMP_ERROR("ERR: input parameter (num) is less than 2 in <MULTI_ROT::setTimeSequence>!\n");
		return -1;
	}

	if (nullptr == t)
	{
		DUMP_ERROR("ERR: input parameter (timeSequence) is nullptr in <MULTI_ROT::setTimeSequence>!\n");
		return -1;
	}

	if (_pNum != num && _pNum > 2)
	{
		DUMP_ERROR("ERR: timeSequence does NOT match pointsSequence in <MULTI_ROT::setTimeSequence>!\n");
		return -1;
	}

	// check t
	if (!_checkTimeT(t))
	{
		_isValid = false;
		return -1;
	}

	// save time sequence
	_newParam = true;
	memcpy(_t,t,sizeof(double)*num);

	return num;
}

int    MULTI_ROT::getTimeSequence(double time[]) const
{
	if (nullptr == time)
	{
		DUMP_ERROR("ERR: input parameter (timeSequence) is nullptr in <MULTI_ROT::getTimeSequence>!\n");
		return -1;
	}

	memcpy(time,_t,sizeof(double)*_pNum);

	return _pNum;
}

double MULTI_ROT::pos(double t, double p[ROTATE_D]) const
{
	if (t <= _t[0])
	{
		t = _t[0];
	}
	if (t >= _t[_pNum-1])
	{
		t = _t[_pNum-1];
	}
	int index = _iSpan(t);

	return _rotate[index].pos(t,p);
}

double MULTI_ROT::vel(double t, double v[ROTATE_D]) const
{
	if (t <= _t[0])
	{
		t = _t[0];
	}
	if (t >= _t[_pNum-1])
	{
		t = _t[_pNum-1];
	}
	int index = _iSpan(t);
	return _rotate[index].vel(t,v);
};

double MULTI_ROT::acc(double t, double a[ROTATE_D]) const 
{
	if (t <= _t[0])
	{
		t = _t[0];
	}
	if (t >= _t[_pNum-1])
	{
		t = _t[_pNum-1];
	}
	int index = _iSpan(t);
	return _rotate[index].acc(t,a);
}

double MULTI_ROT::jerk(double t, double j[ROTATE_D]) const
{
	if (t <= _t[0])
	{
		t = _t[0];
	}
	if (t >= _t[_pNum-1])
	{
		t = _t[_pNum-1];
	}
	int index = _iSpan(t);
	return _rotate[index].jerk(t,j);
}

double MULTI_ROT::tangent(double t, double tangent[]) const
{
	DUMP_WARNING("WARNING: it is invalid in MULTI_ROT::tangent()\n");
	return vel(t,tangent);
}

int    MULTI_ROT::getRpySequence(double rpy[][ROTATE_D]) const
{
	if (_newParam)
	{
		DUMP_WARNING("WARNING: please call <MULTI_ROT::planTrajectory> first!\n");
	}

	memcpy(rpy,_rpy,sizeof(double)*ROTATE_D*_pNum);

	return _pNum;
}

int    MULTI_ROT::rePlanTrajectory(double t, double newVelLimit)
{
	// check new limits
	if (ABS(newVelLimit) <=0)
	{
		DUMP_ERROR("ERR: new limit is invalid for being zeros in <MULTI_ROT::rePlanProfile>!\n");
		return ERR_REP_FAILED;
	}

	// avoid non-necessary re-plan
	if (  ABS(ABS(newVelLimit) - ABS(_velLimit)) < ALMOST_ZERO)
	{
		return REP_SUCCEED;
	}

	// backup time sequence
	double ot[POINTS_MAX_NUM] = {0};
	this->getTimeSequence(ot);

	// re-plan profile on line
	int index = _iSpan(t);
	if (REP_SUCCEED == _rotate[index].rePlanTrajectory(t,newVelLimit))
	{
		int ret = REP_SUCCEED; // re-plan the reset and update
		_t[index+1] = _t[index] + _rotate[index].getDuration();
		for (int i=index+1; i<_pNum-1; i++)
		{
			_rotate[i].setLimit(newVelLimit,_accLimit,_jerkLimit);
			if (REP_SUCCEED != _rotate[i].planTrajectory(_t[i],_rpy[i],_axis[i],_angle[i]))
			{
				ret = ERR_REP_NOTYET;
				_rePlanN = 0;
				break;
			}
			_t[i+1] = _t[i] + _rotate[i].getDuration();
		}

		// one failed, reset everything
		if (REP_SUCCEED != ret)
		{
			memcpy(_t,ot,sizeof(double)*_pNum);
			_synchronizePlanning();
			_rePlanN = 0;
			return ERR_REP_NOTYET;
		}
		else // succeed, update
		{
			_rePlanN = index+1;
			setLimit(newVelLimit,_accLimit,_jerkLimit);
			return REP_SUCCEED;
		}
	}
	else // failed
	{
		memcpy(_t,ot,sizeof(double)*_pNum);
		_synchronizePlanning();
		_rePlanN = 0;
		return ERR_REP_NOTYET;
	}
}

int    MULTI_ROT::_setRpySequence(double rpy[][ROTATE_D],int num)
{
	if (nullptr == rpy)
	{
		DUMP_ERROR("ERR: input parameter (rpy) is nullptr in <MULTI_ROT::_setRpySequence>!\n");
		return -1;
	}

	if (num < 2)
	{
		DUMP_ERROR("ERR: input parameter (num) is less than 2 in <MULTI_ROT::_setRpySequence>!\n");
		return -1;
	}

	// save
	_pNum = num;

	for (int i=0; i<num-1; i++)
	{
		if(_distanceOfRpy(_axis[i],_angle[i],rpy[i],rpy[i+1]))
		{
			DUMP_ERROR("ERR: the input %dth rpy is invalid in <MULTI_ROT::_setRpySequence>!\n",i+1);
			return -1;
		}
	}

	// assign start/target
	_setStart(rpy[0]);
	_setTarget(rpy[num-1]);
	memcpy(_rpy,rpy,sizeof(double)*ROTATE_D*num);

	return _pNum;
}

bool   MULTI_ROT::planTrajectory(double t[], int num, double rpy[][ROTATE_D])
{
	if (_setRpySequence(rpy,num) != num)
	{
		_isValid = false;
		return _isValid;
	}

	if(_setTimeSequence(t,num) != num)
	{
		_isValid = false;
		return _isValid;
	}

	return _planTrajectory();
}

int    MULTI_ROT::getAxisAngleSequence(double axis[][ROTATE_D], double angle[]) const
{
	if (_newParam)
	{
		DUMP_WARNING("WARNING: please call <MULTI_ROT::planTrajectory> first!\n");
	}

	memcpy(axis,_axis,sizeof(double)*ROTATE_D*(_pNum-1));

	memcpy(angle,_angle,sizeof(double)*(_pNum-1));

	return _pNum-1;
}

int    MULTI_ROT::_distanceOfRpy(double axis[3],double&angle,double E1[3], double E2[3])
{
	angle = 0.0;
	double R0[3][3] = {{0.0}};
	double sR[3][3] = {{0.0}};
	double tR[3][3] = {{0.0}};
	double dR[3][3] = {{0.0}};

	rpy2matrix(sR,E1);
	rpy2matrix(tR,E2);
	Trp3(R0,sR);
	M3p3(dR,R0,tR);

	axis[0] = 0; axis[1] = 0;
	axis[ROT_INDEX-TRANS_D] = 1.0;

	return matrix2rot(axis,&angle,dR);
}

int    MULTI_ROT::_setAxisAngleSequence(double axis[][ROTATE_D], double angle[], int num)
{
	if (num < 1)
	{
		DUMP_ERROR("ERR: at least 1 axis and angle is needed in <MULTI_ROT::_setAxisAngleSequence>\n");
		return -1;
	}

	_pNum = num + 1;
	_newParam = true;
	memcpy(_angle,angle,sizeof(double)*(num));
	memcpy(_axis,axis,sizeof(double)*ROTATE_D*(num));
	memcpy(_rpy[0],_startPos,sizeof(double)*ROTATE_D);

	for (int i=1; i<_pNum; i++)
	{
		double sR[3][3];
		rpy2matrix(sR,_rpy[i-1]);

		double dR[3][3];
		double tmp = _angle[i-1];
		double Axis[3] = {_axis[i-1][0],_axis[i-1][1],_axis[i-1][2]};
		
		rot2matrix(dR,Axis,&tmp);

		double tR[3][3];
		M3p3(tR,sR,dR);

		matrix2rpy(_rpy[i],tR);
	}

	_setTarget(_rpy[_pNum-1]);

	return num;
}

double MULTI_ROT::move(double t, double pos[ROTATE_D], double vel[ROTATE_D], double acc[ROTATE_D]) const
{
	this->pos(t,pos);
	this->vel(t,vel);
	return this->acc(t,acc);
}

bool   MULTI_ROT::planTrajectory(double t[], double sR[ROTATE_D], int num, double axis[][ROTATE_D], double angle[])
{
	_setStart(sR);
	
	if ((num-1) != _setAxisAngleSequence(axis,angle,num-1))
	{
		_isValid = false;
		return _isValid;
	}

	if (num != _setTimeSequence(t,num))
	{
		_isValid = false;
		return _isValid;
	}

	return _planTrajectory();
}

