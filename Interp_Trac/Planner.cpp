



#pragma once
#include "Planner.h"


Planner6D::Planner6D()
{
	initiate();
}


Planner6D::~Planner6D()
{

}


int Planner6D::stop()
{
	switch(_plannerState)
	{
	case PLANNER_DONE:
		return PLANNER_SUCCEED;
		break;

	case PLANNER_ZONE_IN:
	case PLANNER_ZONEOUT:
		return PLANNER_NOTYET;
		break;

	case PLANNER_WAITING:
		_plannerStartTime = _plannerSwitchTime;
		_blenderState = PLANNER_NONEXT;
		return PLANNER_SUCCEED;
		break;

	case PLANNER_ZONING:
	case PLANNER_MOVING:
		if (_isNowInJointSpace)
		{
			if (!_tracPtr->stopTrajectory(_plannerStartTime))
			{
				return ERR_PLAN_FAILED;
			}
			_plannerSwitchTime = _tracPtr->getTargetTime();
			return PLANNER_SUCCEED;
		}
		else
		{
			if (!_tracPtr->stopTrajectory(_plannerStartTime))
			{
				return ERR_PLAN_FAILED;
			}
			if(!_rotPtr->stopTrajectory(_plannerStartTime))
			{
				return ERR_PLAN_FAILED;
			}

			_plannerSwitchTime = MAX(_tracPtr->getTargetTime(),_rotPtr->getTargetTime());

			if(tracBspline == _tracPtr->getType())
			{
				_newPointIndexAfterStop = _prevBsplinePlanner.getNowPointIndexAfterStop();
			}

			if (tracCircle == _tracPtr->getType())
			{
				_newCenterAngleAfterStop = _prevCirclePlanner.getNewCenterAngleAndMiddlePointAfterStop(_newMiddlePointAfterStop);
			}

			return PLANNER_SUCCEED;
		}
		break;

	case ERR_PLAN_FAILED:
		return ERR_PLAN_FAILED;
		break;
	}

	return PLANNER_SUCCEED;
}


int Planner6D::_moveZ()
{
	if (_plannerState != PLANNER_ZONE_IN)
	{
		return PLANNER_NOTYET;
	}

	if (_blendPlanner.getPostTrac() != nullptr && nullptr != _blendPlanner.getPrevTrac() )
	{
		if (!_blendPlanner.planTrajectory())
		{
			_blenderState = ERR_PLAN_FAILED;
		}
		else
		{
			double duration = _prevRotPlanner.getTargetTime() - _blendPlanner.getPrevTime();
			if (_blendPlanner.getDuration() < duration)
			{
				_blendPlanner.scaleToDuration(duration);
			}
			_tracPtr = &_blendPlanner;
			_blenderState = PLANNER_SUCCEED;
			DUMP_INFORM("Moving Blender ... \n");
		}
	}
	else
	{
		_blenderState = PLANNER_NONEXT;
		_blendPlanner.initiate();
	}

	_plannerSwitchTime = _tracPtr->getTargetTime();

	return _blenderState;
}


int Planner6D::initiate()
{
	_rotPtr = nullptr;
	_tracPtr = nullptr;

	_plannerStartTime = 0;
	_plannerSwitchTime = 0;

	_isNowInJointSpace = false;
	_isPostPlannerPlanned = false;

	_plannerState = PLANNER_DONE;
	_blenderState = PLANNER_NOZONE;

	_lastTracRefToolInv.cartesianZero();

	memset(_lastTracRefTool,0,sizeof(_lastTracRefTool));

	return PLANNER_SUCCEED;
}


int Planner6D::wait(double wTime)
{
	if (_plannerState != PLANNER_DONE)
	{
		return PLANNER_NOTYET;
	}

	_plannerStartTime = 0;
	_plannerSwitchTime = wTime;

	_plannerState = PLANNER_WAITING;

	return PLANNER_SUCCEED;
}


int Planner6D::rePlan(double refVel)
{
	if (_plannerState != PLANNER_MOVING)
	{
		return PLANNER_NOTYET;
	}

	int zType; double zBlend;
	double refVels[MAX_DOF] = {0};
	for (int i=0; i<MAX_DOF; i++)
	{
		refVels[i] = refVel;
	}

	if (_isNowInJointSpace)
	{
		switch(_axisPtpPlanner.rePlanAxisTrajectory(_plannerStartTime,refVels))
		{
		case ERR_REP_NOTYET:
			return PLANNER_NOTYET;
			break;

		case REP_SUCCEED:
			_plannerSwitchTime = _axisPtpPlanner.getTargetTime();
			return PLANNER_SUCCEED;
			break;

		case ERR_REP_FAILED:
			return ERR_PLAN_FAILED;
			break;
		}
	}
	else
	{
		switch(_tracPtr->rePlanTrajectory(_plannerStartTime,refVel))
		{
		case ERR_REP_NOTYET:
			return PLANNER_NOTYET;
			break;

		case REP_SUCCEED:
			switch(_rotPtr->rePlanTrajectory(_plannerStartTime,refVel))
			{
			case ERR_REP_NOTYET:
				return PLANNER_NOTYET;
				break;

			case REP_SUCCEED:

				if (_tracPtr->getTargetTime() < _rotPtr->getTargetTime())
				{
					if (!_tracPtr->scaleToDuration(_rotPtr->getTargetTime() - _tracPtr->getStartTime()))
					{
						return ERR_PLAN_FAILED;
					}
				}

				switch(_tracPtr->getType())
				{
				case tracLine:
				case tracCircle:
				case tracBspline:
					_blendPlanner.getBlendZone(zType,zBlend);
					if (zBlend > 0)
					{
						_plannerSwitchTime = _blendPlanner.getPrevTime();
					}
					else
					{
						_plannerSwitchTime = _tracPtr->getTargetTime();
					}
					break;

				default:
					_plannerSwitchTime = _tracPtr->getTargetTime();
					break;
				}
				return PLANNER_SUCCEED;
				break;
					
			case ERR_REP_FAILED:
				return ERR_PLAN_FAILED;
				break;
			}
			break;

		case ERR_REP_FAILED:
			return ERR_PLAN_FAILED;
			break;
		}
	}

	return PLANNER_NOTYET;
}


int Planner6D::getUnfinishedBsplineIndex()
{
	return _newPointIndexAfterStop;
}


int Planner6D::_transferVelToFlange(double nextVel[6])
{
	double direc[3] = {0};
	double tracVelInTool[3] = {0};
	_lastTracRefToolInv.getPos(direc);

	cross3(tracVelInTool,nextVel+TRANS_D ,direc);

	double matrix[3][3] = {{0}};
	rpy2matrix(matrix,_lastTracTargetTcpPos+TRANS_D);

	double tracVelInWord[3] = {0};
	M3p3(tracVelInWord,matrix,tracVelInTool);

	nextVel[0] += tracVelInWord[0];
	nextVel[1] += tracVelInWord[1];
	nextVel[2] += tracVelInWord[2];

	return 0;
}


int Planner6D::getUnfinishedCircleCenterAngle(double& angle)
{
	angle = _newCenterAngleAfterStop;

	return PLANNER_SUCCEED;
}


int Planner6D::getUnfinishedCircleMiddlePoint(double midPoint[])
{
	if (midPoint == nullptr)
	{
		DUMP_ERROR("ERR: the input ptr is invalid in <Planner6D::getUnfinishedCircleMiddlePoint>\n");

		return -1;
	}

	memcpy(midPoint,_newMiddlePointAfterStop,sizeof(_newMiddlePointAfterStop));

	return PLANNER_SUCCEED;
}


int Planner6D::updateCurrentJoint(int dof, double currentJointAngle[])
{
	if (PLANNER_DONE != _plannerState && PLANNER_WAITING != _plannerState)
	{
		return PLANNER_NOTYET;
	}

	_dof = dof;

	memcpy(_lastTracTargetJoint,currentJointAngle,sizeof(double)*dof);

	return PLANNER_SUCCEED;
}


int Planner6D::updateCurrentPos(double currentPos[], double refTool[])
{
	switch(_plannerState)
	{
	default:
		return PLANNER_NOTYET;
		break;

	case PLANNER_DONE:
	case PLANNER_WAITING:
	case PLANNER_ZONE_IN:
		break;
	}

	// check tool
	bool sameTool = true;
	if (refTool != nullptr)
	{
		for (int i = 0; i < 6; i++)
		{
			if (ABS(refTool[i] - _lastTracRefTool[i]) > ALMOST_ZERO)
			{
				sameTool = false;
				break;
			}
		}

		if ((!sameTool) && (PLANNER_ZONE_IN == _plannerState))
		{
			return PLANNER_NOTYET;
		}

		if (!sameTool)
		{
			Point6D tmpTool; 
			tmpTool.cartesianZero();
			tmpTool.setWrench(refTool);
			_lastTracRefToolInv = tmpTool.getInverse();
			memcpy(_lastTracRefTool,refTool,sizeof(_lastTracRefTool));
	
			// flange --> tcp
			Point6D flange;
			flange.cartesianZero();
			flange.setWrench(currentPos);
			(flange*tmpTool).getWrench(currentPos);
		}
	}

	// update current pos of TCP
	memcpy(_lastTracTargetTcpPos,currentPos,sizeof(_lastTracTargetTcpPos));

	return PLANNER_SUCCEED;
};


int Planner6D::_transferAccToFlange(double nextVel[6], double nextAcc[6])
{
	double direc[3] = {0};
	double tracAccInTool[3] = {0};
	_lastTracRefToolInv.getPos(direc);

	cross3(tracAccInTool,nextAcc+TRANS_D ,direc);

	double tmpOmega = normN(nextVel+TRANS_D,ROTATE_D);
	double tmpNorm = dotN(nextVel+TRANS_D,direc,ROTATE_D);
	tracAccInTool[0] += tmpNorm*nextVel[0+TRANS_D] - tmpOmega*tmpOmega*direc[0];
	tracAccInTool[1] += tmpNorm*nextVel[1+TRANS_D] - tmpOmega*tmpOmega*direc[1];
	tracAccInTool[2] += tmpNorm*nextVel[2+TRANS_D] - tmpOmega*tmpOmega*direc[2];

	double matrix[3][3] = {{0}};
	rpy2matrix(matrix,_lastTracTargetTcpPos+TRANS_D);

	double tracAccInWord[3] = {0};
	M3p3(tracAccInWord,matrix,tracAccInTool);

	nextAcc[0] += tracAccInWord[0];
	nextAcc[1] += tracAccInWord[1];
	nextAcc[2] += tracAccInWord[2];

	return 0;
}


int Planner6D::setJointLimits(int dof, double jvl[],double jal[],double jjl[])
{
	_dof = dof;
	for (int i=0; i<dof; i++)
	{
		_jointVelLimit[i] = ABS(jvl[i]);
		_jointAccLimit[i] = ABS(jal[i]);
		_jointJerkLimit[i] = ABS(jjl[i]);
	}
	return PLANNER_SUCCEED;
}


int Planner6D::_getTarget(double nextPos[], double nextVel[], double nextAcc[])
{
	// joint space
	if (_isNowInJointSpace)
	{
		if (_axisPtpPlanner.isValid())
		{
			_axisPtpPlanner.move(_plannerStartTime,nextPos,nextVel,nextAcc);
		}
		else
		{
			return ERR_PLAN_FAILED;
		}
		memcpy(_lastTracTargetJoint,nextPos,sizeof(double)*_dof);

		return _plannerState;
	}

	//----------------------------------
	if (_tracPtr != nullptr)
	{
		if (_tracPtr->isValid())
		{
			_tracPtr->move(_plannerStartTime,nextPos,nextVel,nextAcc);
		}
		else
		{
			_plannerState = ERR_PLAN_FAILED;
			return _plannerState;
		}
	}
	else
	{
		_plannerState = ERR_PLAN_FAILED;
		return _plannerState;
	}

	//----------------------------------
	if (_rotPtr != nullptr)
	{
		if (_rotPtr->isValid())
		{
			_rotPtr->move(_plannerStartTime,nextPos+TRANS_D,nextVel+TRANS_D,nextAcc+TRANS_D);
		}
		else
		{
			_plannerState = ERR_PLAN_FAILED;
			return _plannerState;
		}
	}
	else
	{
		_plannerState = ERR_PLAN_FAILED;
		return _plannerState;
	}

	// save to last target
	memcpy(_lastTracTargetTcpPos,nextPos,sizeof(_lastTracTargetTcpPos));

	// pos transfer to flange
	Point6D tmpToolTarget; 
	tmpToolTarget.setWrench(nextPos);
	(tmpToolTarget*_lastTracRefToolInv).getWrench(nextPos);

	//vel transfer to flange
	_transferVelToFlange(nextVel);

	// acc transfer to flange
	_transferAccToFlange(nextVel,nextAcc);

	return _plannerState;
}


int Planner6D::move(int step, double nextPos[], double nextVel[], double nextAcc[])
{
	switch(_plannerState)
	{
	case PLANNER_DONE:
		break;

	case PLANNER_MOVING:
		_plannerState = _getTarget(nextPos,nextVel,nextAcc);
		_plannerStartTime += step/1000.0;
		if (_plannerStartTime >= _plannerSwitchTime)
		{
			_plannerState = PLANNER_ZONE_IN;
		}
		break;

	case PLANNER_ZONE_IN:
		switch(_moveZ())
		{
		case PLANNER_NOZONE:
			_plannerState = PLANNER_DONE;
			break;

		case PLANNER_NONEXT:
		case PLANNER_SUCCEED:
			_plannerState = PLANNER_ZONING;
			goto ZONING;
			break;

		case ERR_PLAN_FAILED:
			DUMP_INFORM("ERR: blender failed!\n");
			_plannerState = PLANNER_ZONING;
			break;
		}
		break;

	case PLANNER_ZONING:
ZONING: _plannerState = _getTarget(nextPos,nextVel,nextAcc);
		_plannerStartTime += step/1000.0;
		if (_plannerStartTime >= _plannerSwitchTime)
		{
			switch(_blenderState)
			{
			case PLANNER_NONEXT:
				_plannerState = PLANNER_DONE;
				break;

			case PLANNER_SUCCEED:
				_plannerState = PLANNER_ZONEOUT;
				break;
			}
		}
		break;

	case PLANNER_ZONEOUT:
		DUMP_ERROR("ERR: something is wrong with in moveJ/moveL/moveC/moveR/moveB!\n");
		_plannerState = ERR_PLAN_FAILED;
		break;

	case PLANNER_WAITING:
		_plannerStartTime += step/1000.0;
		if (_plannerStartTime >= _plannerSwitchTime)
		{
			_plannerState = PLANNER_DONE;
		}
		break;

	case ERR_PLAN_FAILED:
		break;
	}

	return _plannerState;
}


int Planner6D::setCartesianLimits(double tvl,double tal,double tjl,double rvl,double ral,double rjl)
{
	_rotVelLimit = ABS(rvl);
	_rotAccLimit = ABS(ral);
	_rotJerkLimit = ABS(rjl);
	_tracVelLimit = ABS(tvl);
	_tracAccLimit = ABS(tal);
	_tracJerkLimit = ABS(tjl);
	return PLANNER_SUCCEED;
}


int Planner6D::moveR(double targetPos[], double refVel, double refAcc, double refJerk, bool rel, double refTime)
{
	if (_plannerState != PLANNER_DONE)
	{   
		return PLANNER_NOTYET;
	}

	// plan rotate
	double sR[ROTATE_D] = {0};
	double tR[ROTATE_D] = {0};
	memcpy(tR,targetPos+TRANS_D,sizeof(sR));
	memcpy(sR,_lastTracTargetTcpPos+TRANS_D,sizeof(tR));

	if (rel)
	{
		double sM[ROTATE_D][ROTATE_D] = {{0}};
		double dM[ROTATE_D][ROTATE_D] = {{0}};
		double tM[ROTATE_D][ROTATE_D] = {{0}};
		rpy2matrix(sM,sR);
		rpy2matrix(dM,tR);
		M3p3(tM,sM,dM);
		matrix2rpy(tR,tM);
	}

	_prevRotPlanner.initiate();
	_prevRotPlanner.setLimit(refVel*_rotVelLimit,refAcc*_rotAccLimit,refJerk*_rotJerkLimit);
	_prevRotPlanner.planTrajectory(0,sR,tR);
	if (!_prevRotPlanner.isValid())
	{
		return ERR_PLAN_FAILED;
	}

	// plan translation
	double sP[TRANS_D] = {0};
	double tP[TRANS_D] = {0};
	memcpy(sP,_lastTracTargetTcpPos,sizeof(sP));
	memcpy(tP,_lastTracTargetTcpPos,sizeof(sP));	// target translation ignored

	_prevLinePlanner.initiate();
	_prevLinePlanner.setLimit(refVel*_tracVelLimit,refAcc*_tracAccLimit,refJerk*_tracJerkLimit);
	_prevLinePlanner.planTrajectory(0,sP,tP);
	if (!_prevLinePlanner.isValid())
	{
		return ERR_PLAN_FAILED;
	}

	// synchronize
	if (_prevRotPlanner.getDuration() > _prevLinePlanner.getDuration())
	{
		_prevLinePlanner.scaleToDuration(_prevRotPlanner.getDuration());
	}

	// update
	_plannerStartTime = 0;
	_blendPlanner.initiate();
	_isNowInJointSpace = false;
	_rotPtr = &_prevRotPlanner;
	_tracPtr = &_prevLinePlanner;
	_plannerState = PLANNER_MOVING;
	DUMP_INFORM("Moving rotate ... \n");
	_plannerSwitchTime = _prevRotPlanner.getTargetTime();
	return PLANNER_SUCCEED;
}


int Planner6D::moveJ(double targetJoint[], double refVel, double refAcc, double refJerk, bool rel,double refTime)
{
	if (_plannerState != PLANNER_DONE)
	{   
		return PLANNER_NOTYET;
	}

	// prepare
	double sJ[MAX_DOF] = {0},tJ[MAX_DOF] = {0};
	memcpy(sJ,_lastTracTargetJoint,sizeof(sJ));
	memcpy(tJ,targetJoint,sizeof(double)*_dof);

	if (rel)
	{
		for (int i=0; i<_dof; i++)
		{
			tJ[i] += sJ[i];
		}
	}

	// plan
	double vl[MAX_DOF],al[MAX_DOF],jl[MAX_DOF];
	for (int i=0; i<_dof; i++)
	{
		vl[i] = refVel*_jointVelLimit[i];
		al[i] = refAcc*_jointAccLimit[i];
		jl[i] = refJerk*_jointJerkLimit[i];
	}
	_axisPtpPlanner.setAxisLimits(_dof,vl,al,jl);
	if (!_axisPtpPlanner.planTrajectory(0,sJ,tJ))
	{
		return ERR_PLAN_FAILED;
	}

	if (_axisPtpPlanner.getDuration() < refTime)
	{
		if (!_axisPtpPlanner.scaleToDuration(refTime))
		{
			return ERR_PLAN_FAILED;
		}
	}

	// update
	_plannerSwitchTime = _axisPtpPlanner.getTargetTime();
	DUMP_INFORM("Moving PTP ... \n");
	_plannerState = PLANNER_MOVING;
	_tracPtr = &_axisPtpPlanner;
	_isNowInJointSpace = true;
	_blendPlanner.initiate();
	_plannerStartTime = 0;
	
	return PLANNER_SUCCEED;
}


int Planner6D::moveB(double targetPos[][6], int pointsNum, double refVel, double zArea, int zType,bool rel, double refTime)
{
	switch(_plannerState)
	{
	case PLANNER_DONE:
	case PLANNER_ZONE_IN:
		goto PLAN_NEXT_MOTION;
		break;

	case PLANNER_ZONEOUT:
		goto SWITCH_PLANNERS;
		break;
		
	default:
		return PLANNER_NOTYET;
		break;
	}


PLAN_NEXT_MOTION://------------------------------------------------
	if (!_isPostPlannerPlanned)
	{
		// plan translation
		double targetP[POINTS_MAX_NUM][TRANS_D] = {{0}};
		if (nullptr == _blendPlanner.getPrevTrac())
		{
			memcpy(targetP[0],_lastTracTargetTcpPos,sizeof(double)*TRANS_D);
		}
		else
		{
			_blendPlanner.getPrevTrac()->getTarget(targetP[0]);
		}
		
		for (int i=1; i<= pointsNum; i++)
		{
			memcpy(targetP[i],targetPos[i-1],sizeof(double)*TRANS_D);
			if (rel)
			{
				for (int j=0; j<TRANS_D; j++)
				{
					targetP[i][j] += targetP[0][j];
				}
			}
		}
		_postBsplinePlanner.initiate();
		if (refTime <= 0)
		{
			if(!_postBsplinePlanner.planTrajectory(0,100,targetP,pointsNum+1))
			{
				return ERR_PLAN_FAILED;
			}

			refTime = 1.5 * _postBsplinePlanner.getChordLength() / (refVel*_tracVelLimit);
		}

		if(!_postBsplinePlanner.planTrajectory(0,refTime,targetP,pointsNum+1))
		{
			return ERR_PLAN_FAILED;
		}

		// update blender zone if necessary
		_blendPlanner.setPostTrac(&_postBsplinePlanner);
		_rotTimeOffset = _blendPlanner.getPostTime();

		// plan rotate
		double time[POINTS_MAX_NUM] = {0};
		double targetR[POINTS_MAX_NUM][ROTATE_D] = {{0}};
		if (_rotPtr != nullptr)
		{
			_rotPtr->getTarget(targetR[0]);
		}
		else
		{
			memcpy(targetR[0],_lastTracTargetTcpPos+TRANS_D,sizeof(double)*ROTATE_D);
		}
		_postBsplinePlanner.getTimeSequence(time);
		for (int i=1; i<=pointsNum; i++)
		{
			time[i] = time[i] + _rotTimeOffset;
			memcpy(targetR[i],targetPos[i-1]+TRANS_D,sizeof(double)*ROTATE_D);
			if (rel)
			{
				double sM[ROTATE_D][ROTATE_D] = {{0}};
				double dM[ROTATE_D][ROTATE_D] = {{0}};
				double tM[ROTATE_D][ROTATE_D] = {{0}};
				rpy2matrix(sM,targetR[0]);
				rpy2matrix(dM,targetR[i]);
				M3p3(tM,sM,dM);
				matrix2rpy(targetR[i],tM);
			}
		}
		time[0] += _rotTimeOffset;
		_postMultiRotPlanner.initiate();
		_postMultiRotPlanner.setLimit(refVel*_rotVelLimit,_rotAccLimit,_rotJerkLimit);
		_postMultiRotPlanner.planTrajectory(time,pointsNum+1,targetR);
		if (!_postMultiRotPlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// synchronize
		if (_postMultiRotPlanner.isTimeChanged())
		{
			_postMultiRotPlanner.getTimeSequence(time);
			for (int i=0; i<pointsNum+1; i++)
			{
				time[i] -= _rotTimeOffset;
			}
			_postBsplinePlanner.planTrajectory(time,targetP,pointsNum+1);
			if (!_postBsplinePlanner.isValid())
			{
				return ERR_PLAN_FAILED;
			}
		}
		_isPostPlannerPlanned = true;
	}

	switch(_plannerState)
	{
	case PLANNER_DONE:
		goto SWITCH_PLANNERS;
		break;

	case PLANNER_ZONE_IN:
		return PLANNER_NOTYET;
		break;
	}


SWITCH_PLANNERS://-------------------------------------------------
	if (_blendPlanner.getPostTrac() != nullptr && nullptr != _blendPlanner.getPrevTrac())
	{
		double burr = _plannerStartTime - _blendPlanner.getTargetTime();
		_plannerStartTime = _rotTimeOffset + burr;
	}
	else
	{
		_plannerStartTime = 0;
	}
	_prevMultiRotPlanner = _postMultiRotPlanner;
	_prevBsplinePlanner = _postBsplinePlanner;
	_blendPlanner.initiate();

	if (zArea > 0)
	{
		_blendPlanner.setBlendZone(zType,zArea);
		_blendPlanner.setPrevTrac(&_prevBsplinePlanner);
		_plannerSwitchTime = _blendPlanner.getPrevTime();
	}
	else
	{
		_plannerSwitchTime = _prevBsplinePlanner.getTargetTime();
	}


//UPDATE_STATE:----------------------------------------------------
	_isNowInJointSpace = false;
	_isPostPlannerPlanned = false;
	_plannerState = PLANNER_MOVING;
	_rotPtr = &_prevMultiRotPlanner;
	_tracPtr = &_prevBsplinePlanner;
	DUMP_INFORM("Moving B-spline ... \n");
	return PLANNER_SUCCEED;
}


int Planner6D::moveL(double targetPos[], double refVel, double refAcc, double refJerk, double zArea, int zType, bool rel,double refTime)
{
	switch(_plannerState)
	{
	case PLANNER_DONE:
	case PLANNER_ZONE_IN:
		goto PLAN_NEXT_MOTION;
		break;

	case PLANNER_ZONEOUT:
		goto SWITCH_PLANNERS;
		break;

	default:
		return PLANNER_NOTYET;
		break;
	}


PLAN_NEXT_MOTION://------------------------------------------------
	if (!_isPostPlannerPlanned)
	{
		// plan translation
		double sP[TRANS_D] = {0}, tP[TRANS_D] = {0};
		if (nullptr == _blendPlanner.getPrevTrac())
		{
			memcpy(sP,_lastTracTargetTcpPos,sizeof(sP));
		}
		else
		{
			_blendPlanner.getPrevTrac()->getTarget(sP);
		}
		memcpy(tP,targetPos,sizeof(tP));
		if (rel)
		{
			for (int i=0; i<TRANS_D; i++)
			{
				tP[i] += sP[i];
			}
		}
		_postLinePlanner.initiate();
		_postLinePlanner.setLimit(refVel*_tracVelLimit,refAcc*_tracAccLimit,refJerk*_tracJerkLimit);
		_postLinePlanner.planTrajectory(0,sP,tP);
		if (!_postLinePlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// update blender zone if necessary
		_blendPlanner.setPostTrac(&_postLinePlanner);
		_rotTimeOffset = _blendPlanner.getPostTime();

		// plan rotate
		double sR[ROTATE_D] = {0}, tR[ROTATE_D] = {0};
		
		if (_rotPtr != nullptr)
		{
			_rotPtr->getTarget(sR);
		}
		else
		{
			memcpy(sR,_lastTracTargetTcpPos+TRANS_D,sizeof(sR));
		}

		memcpy(tR,targetPos+TRANS_D,sizeof(tR));
		if (rel)
		{
			double sM[ROTATE_D][ROTATE_D] = {{0}};
			double dM[ROTATE_D][ROTATE_D] = {{0}};
			double tM[ROTATE_D][ROTATE_D] = {{0}};
			rpy2matrix(sM,sR);
			rpy2matrix(dM,tR);
			M3p3(tM,sM,dM);
			matrix2rpy(tR,tM);
		}
		_postRotPlanner.initiate();
		_postRotPlanner.setLimit(refVel*_rotVelLimit,refAcc*_rotAccLimit,refJerk*_rotJerkLimit);
		_postRotPlanner.planTrajectory(_rotTimeOffset,sR,tR);
		if (!_postRotPlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// synchronize
		if (_postRotPlanner.getTargetTime() > _postLinePlanner.getTargetTime())
		{
			_postLinePlanner.scaleToDuration(_postRotPlanner.getDuration()+_rotTimeOffset);
		}
		if (_postLinePlanner.getTargetTime() > _postRotPlanner.getTargetTime())
		{
			_postRotPlanner.scaleToDuration(_postLinePlanner.getDuration() - _rotTimeOffset);
		}
		_isPostPlannerPlanned = true;
	}

	if(PLANNER_ZONE_IN == _plannerState)
	{
		return PLANNER_NOTYET;
	}


SWITCH_PLANNERS://-------------------------------------------------
	if (_blendPlanner.getPostTrac() != nullptr && nullptr != _blendPlanner.getPrevTrac() )
	{
		double burr = _plannerStartTime - _blendPlanner.getTargetTime();
		_plannerStartTime = _rotTimeOffset + burr;
	}
	else
	{
		_plannerStartTime = 0;
	}
	_prevLinePlanner = _postLinePlanner;
	_prevRotPlanner = _postRotPlanner;
	_blendPlanner.initiate();
	if (zArea > 0)
	{
		_blendPlanner.setBlendZone(zType,zArea);
		_blendPlanner.setPrevTrac(&_prevLinePlanner);
		_plannerSwitchTime = _blendPlanner.getPrevTime();
	}
	else
	{
		_plannerSwitchTime = _prevLinePlanner.getTargetTime();
	}


//UPDATE_STATE:-------------------------------------------------
	_isNowInJointSpace = false;
	_rotPtr = &_prevRotPlanner;
	_tracPtr = &_prevLinePlanner;
	_isPostPlannerPlanned = false;
	_plannerState = PLANNER_MOVING;
	DUMP_INFORM("Moving Line ... \n");
	return PLANNER_SUCCEED;
}


int Planner6D::moveC(double targetPos[], double middlePos[], double refVel, double refAcc, double refJerk, double zArea, int zType, bool rel, double refTime, int cycle)
{
	switch(_plannerState)
	{
	case PLANNER_DONE:
	case PLANNER_ZONE_IN:
		goto PLAN_NEXT_MOTION;
		break;

	case PLANNER_ZONEOUT:
		goto SWITCH_PLANNERS;
		break;

	default:
		return PLANNER_NOTYET;
		break;
	}


PLAN_NEXT_MOTION://-------------------------------------------------
	if (!_isPostPlannerPlanned)
	{
		// plan translation
		double sP[TRANS_D] = {0}, mP[TRANS_D] = {0}, tP[TRANS_D] ={0};
		if (nullptr == _blendPlanner.getPrevTrac())
		{
			memcpy(sP,_lastTracTargetTcpPos,sizeof(sP));
		}
		else
		{
			_blendPlanner.getPrevTrac()->getTarget(sP);
		}
		memcpy(mP,middlePos,sizeof(mP));
		memcpy(tP,targetPos,sizeof(tP));
		if (rel)
		{
			for (int i=0; i<TRANS_D; i++)
			{
				mP[i] += sP[i];
				tP[i] += sP[i];
			}
		}

		_postCirclePlanner.initiate();
		_postCirclePlanner.setLimit(refVel*_tracVelLimit,refAcc*_tracAccLimit,refJerk*_tracJerkLimit);
		_postCirclePlanner.planTrajectory(0,sP,mP,tP,cycle);
		if (!_postCirclePlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// update blender zone if necessary
		_blendPlanner.setPostTrac(&_postCirclePlanner);
		_rotTimeOffset = _blendPlanner.getPostTime();

		// plan rotate
		double sR[ROTATE_D] = {0}, tR[ROTATE_D] = {0};
		if (_rotPtr != nullptr)
		{
			_rotPtr->getTarget(sR);
		}
		else
		{
			memcpy(sR,_lastTracTargetTcpPos+TRANS_D,sizeof(sR));
		}
		memcpy(tR,targetPos+TRANS_D,sizeof(tR));

		if (rel)
		{
			double sM[ROTATE_D][ROTATE_D] = {{0}};
			double dM[ROTATE_D][ROTATE_D] = {{0}};
			double tM[ROTATE_D][ROTATE_D] = {{0}};
			rpy2matrix(sM,sR);
			rpy2matrix(dM,tR);
			M3p3(tM,sM,dM);
			matrix2rpy(tR,tM);
		}
		_postRotPlanner.initiate();
		_postRotPlanner.setLimit(refVel*_rotVelLimit,refAcc*_rotAccLimit,refJerk*_rotJerkLimit);
		_postRotPlanner.planTrajectory(_rotTimeOffset,sR,tR);
		if (!_postRotPlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// synchronize
		if (_postRotPlanner.getTargetTime() > _postCirclePlanner.getTargetTime())
		{
			_postCirclePlanner.scaleToDuration(_postRotPlanner.getDuration()+_rotTimeOffset);
		}
		if (_postCirclePlanner.getTargetTime() > _postRotPlanner.getTargetTime())
		{
			_postRotPlanner.scaleToDuration(_postCirclePlanner.getTargetTime()-_rotTimeOffset);
		}
		_isPostPlannerPlanned = true;
	}

	if(PLANNER_ZONE_IN == _plannerState)
	{
		return PLANNER_NOTYET;
	}


SWITCH_PLANNERS://-------------------------------------------------
	if (_blendPlanner.getPostTrac() != nullptr && nullptr != _blendPlanner.getPrevTrac())
	{
		double burr = _plannerStartTime - _blendPlanner.getTargetTime();
		_plannerStartTime = _rotTimeOffset + burr;
	}
	else
	{
		_plannerStartTime = 0;
	}
	_prevCirclePlanner = _postCirclePlanner;
	_prevRotPlanner = _postRotPlanner;
	_blendPlanner.initiate();
	if (zArea > 0)
	{
		_blendPlanner.setBlendZone(zType,zArea);
		_blendPlanner.setPrevTrac(&_prevCirclePlanner);
		_plannerSwitchTime = _blendPlanner.getPrevTime();
	}
	else
	{
		_plannerSwitchTime = _prevCirclePlanner.getTargetTime();
	}


//UPDATE_STATE:-------------------------------------------------
	_isNowInJointSpace = false;
	_rotPtr = &_prevRotPlanner;
	_isPostPlannerPlanned = false;
	_tracPtr = &_prevCirclePlanner;
	_plannerState = PLANNER_MOVING;
	DUMP_INFORM("Moving Circle ... \n");

	return PLANNER_SUCCEED;
}


int Planner6D::moveC(double centerPoint[],double centerAngle, double normVec[], double targetRot[], double refVel, double refAcc,double refJerk, double zArea, int zType, double refTime)
{
	switch(_plannerState)
	{
	case PLANNER_DONE:
	case PLANNER_ZONE_IN:
		goto PLAN_NEXT_MOTION;
		break;

	case PLANNER_ZONEOUT:
		goto SWITCH_PLANNERS;
		break;

	default:
		return PLANNER_NOTYET;
		break;
	}


PLAN_NEXT_MOTION://-------------------------------------------------
	if (!_isPostPlannerPlanned)
	{
		// plan translation
		double sP[TRANS_D] = {0}, cP[TRANS_D] = {0}, nV[TRANS_D] = {0},cA = 0;
		if (nullptr == _blendPlanner.getPrevTrac())
		{
			memcpy(sP,_lastTracTargetTcpPos,sizeof(sP));
		}
		else
		{
			_blendPlanner.getPrevTrac()->getTarget(sP);
		}
		memcpy(cP,centerPoint,sizeof(cP));
		memcpy(nV,normVec,sizeof(nV));
		cA = centerAngle;

		_postCirclePlanner.initiate();
		_postCirclePlanner.setLimit(refVel*_tracVelLimit,refAcc*_tracAccLimit,refJerk*_tracJerkLimit);
		_postCirclePlanner.planTrajectory(0,sP,cP,nV,cA);
		if (!_postCirclePlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// update blender zone if necessary
		_blendPlanner.setPostTrac(&_postCirclePlanner);
		double timeOffset = _blendPlanner.getPostTime();

		// plan rotate
		double sR[ROTATE_D] = {0}, tR[ROTATE_D] = {0};
		if (_rotPtr != nullptr)
		{
			_rotPtr->getTarget(sR);
		}
		else
		{
			memcpy(sR,_lastTracTargetTcpPos+TRANS_D,sizeof(sR));
		}
		memcpy(tR,targetRot,sizeof(tR));

		_postRotPlanner.initiate();
		_postRotPlanner.setLimit(refVel*_rotVelLimit,refAcc*_rotAccLimit,refJerk*_rotJerkLimit);
		_postRotPlanner.planTrajectory(timeOffset,sR,tR);
		if (!_postRotPlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// synchronize
		if (_postRotPlanner.getTargetTime() > _postCirclePlanner.getTargetTime())
		{
			_postCirclePlanner.scaleToDuration(_postRotPlanner.getDuration()+timeOffset);
		}
		if (_postCirclePlanner.getTargetTime() > _postRotPlanner.getTargetTime())
		{
			_postRotPlanner.scaleToDuration(_postCirclePlanner.getTargetTime()-timeOffset);
		}
		_isPostPlannerPlanned = true;
	}

	if(PLANNER_ZONE_IN == _plannerState)
	{
		return PLANNER_NOTYET;
	}


SWITCH_PLANNERS://-------------------------------------------------
	if (_blendPlanner.getPostTrac() != nullptr && nullptr != _blendPlanner.getPrevTrac())
	{
		double burr = _plannerStartTime - _blendPlanner.getTargetTime();
		_plannerStartTime = _blendPlanner.getPostTime() + burr;
	}
	else
	{
		_plannerStartTime = 0;
	}
	_prevCirclePlanner = _postCirclePlanner;
	_prevRotPlanner = _postRotPlanner;
	_blendPlanner.initiate();
	if (zArea > 0)
	{
		_blendPlanner.setBlendZone(zType,zArea);
		_blendPlanner.setPrevTrac(&_prevCirclePlanner);
		_plannerSwitchTime = _blendPlanner.getPrevTime();
	}
	else
	{
		_plannerSwitchTime = _prevCirclePlanner.getTargetTime();
	}


//UPDATE_STATE:-------------------------------------------------
	_isNowInJointSpace = false;
	_rotPtr = &_prevRotPlanner;
	_isPostPlannerPlanned = false;
	_tracPtr = &_prevCirclePlanner;
	_plannerState = PLANNER_MOVING;
	DUMP_INFORM("Moving Circle ... \n");

	return PLANNER_SUCCEED;
}





#define	TARGET_X		(0)
#define TARGET_Y		(1)
#define TARGET_A		(2)
#define TARGET_W		(3)

#define NORMAL_TRAC		(0)
#define PURE_TURNING	(1)
#define PURE_BLENDING	(2)
#define PURE_ROTATING	(3)

#define TURNING_VEL		(5)
#define TURNING_ACC		(25)
#define TURNING_JERK	(250)

#define TURNING_TIME	(1.0)
#define TIME_SCALE_MS	(100)


PlannerAgv::PlannerAgv()
{
	initiate();
}


PlannerAgv::~PlannerAgv()
{

}


int PlannerAgv::stop()
{
	switch(_plannerState)
	{
	case PLANNER_DONE:
	case PLANNER_ZONE_IN:
	case PLANNER_ZONEOUT:
		return PLANNER_NOTYET;
		break;

	case PLANNER_WAITING:
		_plannerStartTime = _plannerSwitchTime;
		_blenderState = PLANNER_NONEXT;
		return PLANNER_SUCCEED;
		break;

	case PLANNER_ZONING:
		if (_tracType == PURE_TURNING)
		{
			if (_stopTurningPlanner())
			{
				return ERR_PLAN_FAILED;
			}
			_blenderState = PLANNER_NONEXT;
			return PLANNER_SUCCEED;
		}
		goto STOP_TRAC_PLANNER;
		break;

	case PLANNER_MOVING:
		goto STOP_TRAC_PLANNER;
		break;

	case ERR_PLAN_FAILED:
		return ERR_PLAN_FAILED;
		break;
	}

STOP_TRAC_PLANNER:
	if (!_tracPtr->stopTrajectory(_plannerStartTime))
	{
		return ERR_PLAN_FAILED;
	}
	if(!_rotPtr->stopTrajectory(_plannerStartTime))
	{
		return ERR_PLAN_FAILED;
	}

	if (tracBspline == _tracPtr->getType())
	{
		_newPointIndexAfterStop = _prevBsplinePlanner.getNowPointIndexAfterStop();
	}

	if (tracCircle == _tracPtr->getType())
	{
		_newCenterAngleAfterStop = _prevCirclePlanner.getNewCenterAngleAndMiddlePointAfterStop(_newMiddlePointAfterStop);
	}

	if (tracBlender == _tracPtr->getType())
	{
		_prevRotPlanner.getTarget(_newBlenderPointAfterStop+TRANS_D);
		(_blendPlanner.getPrevTrac())->getTarget(_newBlenderPointAfterStop);
	}

	_plannerSwitchTime = MAX(_tracPtr->getTargetTime(),_rotPtr->getTargetTime());

	_blenderState = PLANNER_NONEXT;

	return PLANNER_SUCCEED;
}


int PlannerAgv::_moveZ()
{
	if (_plannerState != PLANNER_ZONE_IN)
	{
		return PLANNER_NOTYET;
	}

	int zType = 0;
	double zBlend = 0;
	double rotDuration = 0;
	double tracDuration = 0;
	double blendDuration = 0;

	// if this is NO next motion
	if (nullptr == _blendPlanner.getPostTrac())
	{
		_plannerSwitchTime = MAX(_tracPtr->getTargetTime(), _rotPtr->getTargetTime());
		_blenderState = PLANNER_NONEXT;
		_blendPlanner.initiate();
		_tracType = NORMAL_TRAC;
		return PLANNER_NONEXT;
	}

	// skip the invalid blender
	if (nullptr != _blendPlanner.getPrevTrac())
	{
		if (!_blendPlanner.planTrajectory())
		{
			_blenderState = ERR_PLAN_FAILED;
			return ERR_PLAN_FAILED;
		}
		_blendPlanner.getBlendZone(zType,zBlend);
		blendDuration = _blendPlanner.getDuration();
		rotDuration = _rotPtr->getTargetTime() - _blendPlanner.getPrevTime();
		tracDuration = MAX(_blendPlanner.getPrevTrac()->getDuration(), _blendPlanner.getPostTrac()->getDuration());
	}

	// turning direction
	if (zBlend <= MIN_ZONE || blendDuration < TIME_SCALE_MS/1000.0 || tracDuration < 2.0*TIME_SCALE_MS/1000.0)
	{
		double p0 = _lastTracTargetTurnOfFlange;
		double pf = p0 + radDistance(p0, _targetTurnOfNextMotionOfFlange);

		_turningPlanner.setLimit(TURNING_VEL, TURNING_ACC, TURNING_JERK);

		if (!_turningPlanner.planProfile(_plannerStartTime,p0,pf))
		{
			_blenderState = ERR_PLAN_FAILED;
			return ERR_PLAN_FAILED;
		}

		if (_turningPlanner.getDuration() < rotDuration)
		{
			if (!_turningPlanner.scaleTo(rotDuration))
			{
				_blenderState = ERR_PLAN_FAILED;
				return ERR_PLAN_FAILED;
			}
		}

		_blendPlanner.initiate();
		_tracType = PURE_TURNING;
		_blenderState = PLANNER_SUCCEED;
		DUMP_INFORM("Turning Direction ... \n");

		_plannerSwitchTime = _plannerStartTime + _turningPlanner.getDuration();
		_plannerSwitchTime = MAX(_plannerSwitchTime, _plannerStartTime + TURNING_TIME);
		_plannerSwitchTime = MAX(_plannerSwitchTime, _refTurnTimeOfNextMotionOfFlange);
	}
	else
	{
		if (_blendPlanner.getDuration() < rotDuration)
		{
			if(!_blendPlanner.scaleToDuration(rotDuration))
			{
				_blenderState = ERR_PLAN_FAILED;
				return ERR_PLAN_FAILED;
			}
		}

		_tracPtr = &_blendPlanner;
		_tracType = PURE_BLENDING;
		_turningPlanner.initiate();
		_blenderState = PLANNER_SUCCEED;
		DUMP_INFORM("Moving Blender ... \n");
		_plannerSwitchTime = _tracPtr->getTargetTime();
	}

	return _blenderState;
}


int PlannerAgv::initiate()
{
	_rotPtr = nullptr;
	_tracPtr = nullptr;

	_rotTimeOffset = 0;
	_plannerStartTime = 0;
	_plannerSwitchTime = 0;

	_tracType = NORMAL_TRAC;
	_plannerState = PLANNER_DONE;
	_blenderState = PLANNER_NOZONE;

	_isPostPlannerPlanned = false;

	_lastTracTargetTurnOfFlange = 0;
	_targetTurnOfNextMotionOfFlange = 0;
	_refTurnTimeOfNextMotionOfFlange = 0;

	_lastTracRefToolInv.cartesianZero();

	memset(_lastTracRefTool,0,sizeof(_lastTracRefTool));

	return PLANNER_SUCCEED;
}


int PlannerAgv::deInitiate()
{
	return PLANNER_SUCCEED;
}


int PlannerAgv::getTracType()
{
	return _tracType;
}


int PlannerAgv::_prepareTurning()
{
	// plan NULL translation
	_prevLinePlanner.initiate();
	_prevLinePlanner.setLimit(_tracVelLimit, _tracAccLimit, _tracJerkLimit);

	double sP[TRANS_D] = {_lastTracTargetTcpPos[0],_lastTracTargetTcpPos[1], 0};
	double tP[TRANS_D] = {_lastTracTargetTcpPos[0],_lastTracTargetTcpPos[1], 0};

	if (!_prevLinePlanner.planTrajectory(0,sP,tP))
	{
		return ERR_PLAN_FAILED;
	}

	// plan NULL rotation
	_prevRotPlanner.initiate();
	_prevRotPlanner.setLimit(_rotVelLimit, _rotAccLimit, _rotJerkLimit);

	double nV[ROTATE_D] = {0};nV[ROT_INDEX - TRANS_D] = 1.0;
	double sA[ROTATE_D] = {0};sA[ROT_INDEX - TRANS_D] = _lastTracTargetTcpPos[ROT_INDEX];

	if (!_prevRotPlanner.planTrajectory(0,sA,nV,0))
	{
		return ERR_PLAN_FAILED;
	}

	// update
	_plannerStartTime = 0;
	_blendPlanner.initiate();
	_tracType = PURE_TURNING;
	_rotPtr = &_prevRotPlanner;
	_tracPtr = &_prevLinePlanner;
	_plannerState = PLANNER_ZONE_IN;
	_blendPlanner.setBlendZone(0,0);
	_blendPlanner.setPostTrac(_tracPtr);

	return PLANNER_NOTYET;
}


int PlannerAgv::wait(double wTime)
{
	if (_plannerState != PLANNER_DONE)
	{
		return PLANNER_NOTYET;
	}

	_plannerStartTime = 0;
	_plannerSwitchTime = wTime;
	_plannerState = PLANNER_WAITING;

	return PLANNER_SUCCEED;
}


int PlannerAgv::_stopTurningPlanner()
{
	// current status of the profile
	double a_limit = ABS(_rotAccLimit)*2.0;
	double j_limit = ABS(_rotJerkLimit)*2.0;
	double v = _turningPlanner.vel(_plannerStartTime);
	double a = _turningPlanner.acc(_plannerStartTime);
	double dP= _turningPlanner.pos(_turningPlanner.getDuration())-_turningPlanner.pos(0);

	if (a < 0)
	{
		return PLANNER_SUCCEED;
	}

	double timeToVelMax = a / j_limit;
	double newVelLimit = ABS(v) + ABS(0.5*a*timeToVelMax);

	// new profile without cruising phase
	double Ta;   DS_or_Trap tempInterP; tempInterP.initiate();
	if ((newVelLimit - 0) * j_limit < a_limit * a_limit)
	{
		Ta = 2.0*sqrt((newVelLimit - 0)/j_limit);
	}
	else
	{
		Ta = a_limit / j_limit + (newVelLimit - 0)/a_limit;
	}
	tempInterP.setLimit(newVelLimit,a_limit,j_limit);
	double deltaP = sign(dP)*(Ta + PLAN_CYCLE/1000.0)*newVelLimit;

	if (tempInterP.planProfile(0,0,deltaP))
	{
		double p0 = _turningPlanner.pos(_plannerStartTime) - tempInterP.pos(Ta-timeToVelMax);
		tempInterP.planProfile(0,p0,p0+deltaP);
		if (tempInterP.isValid())
		{
			_plannerSwitchTime = tempInterP.getDuration();
			_plannerStartTime = Ta - timeToVelMax;
			_turningPlanner = tempInterP;
			return PLANNER_SUCCEED;
		}
		else
		{
			return ERR_PLAN_FAILED;
		}
	}
	else
	{
		return ERR_PLAN_FAILED;
	}
}


int PlannerAgv::rePlan(double refVel)
{
	int ret = PLANNER_SUCCEED;
	if (_plannerState == PLANNER_MOVING)
	{
		if (   !_tracPtr->isInCruisingPhase(_plannerStartTime) 
			|| !_rotPtr->isInCruisingPhase(_plannerStartTime))
		{   
			return PLANNER_NOTYET;// rePlan in cruising phase only
		}

		switch(_tracPtr->rePlanTrajectory(_plannerStartTime,refVel))
		{
		case ERR_REP_NOTYET:
			return PLANNER_NOTYET;
			break;

		case REP_SUCCEED:
			switch(_rotPtr->rePlanTrajectory(_plannerStartTime,refVel))
			{
			case ERR_REP_NOTYET:
				return PLANNER_NOTYET;
				break;

			case REP_SUCCEED:
				// synchronize
				if (   _rotPtr->getTargetTime() - _plannerStartTime > TIME_SCALE_MS / 1000.0 
					|| _tracPtr->getTargetTime() - _plannerStartTime > 2.0*TIME_SCALE_MS / 1000.0)
				{
					if (_rotPtr->getTargetTime() + TIME_SCALE_MS / 1000.0 >  _tracPtr->getTargetTime())
					{
						_tracPtr->scaleToDuration(_rotPtr->getTargetTime() + TIME_SCALE_MS / 1000.0 - _tracPtr->getStartTime());
					}

					if (_tracPtr->getTargetTime() - TIME_SCALE_MS / 1000.0 - _rotPtr->getStartTime() > _rotPtr->getDuration())
					{
						//pRot.scaleToDuration(pTrac.getTargetTime() - TIME_SCALE_MS / 1000.0 - pRot.getStartTime());  /// ???? something is wrong !!!
						DUMP_INFORM("rotDuration:%.4f    newDuration:%.4f\n", _rotPtr->getDuration(), _tracPtr->getTargetTime() - TIME_SCALE_MS / 1000.0 - _rotPtr->getStartTime());
					}
				}

				// update time axis
				switch(_tracPtr->getType())
				{
				case tracLine:
				case tracCircle:
				case tracBspline:
					_plannerSwitchTime = _blendPlanner.getPrevTime();
					break;

				default:
					_plannerSwitchTime = _tracPtr->getTargetTime();
					break;
				}
				return PLANNER_SUCCEED;
				break;

			case ERR_REP_FAILED:
				return ERR_PLAN_FAILED;
				break;
			}
			break;

		case ERR_REP_FAILED:
			return ERR_PLAN_FAILED;
			break;
		}
	}
	else
	{
		ret = PLANNER_NOTYET;
	}

	return ret;
}


int PlannerAgv::getUnfinishedBsplineIndex()
{
	return _newPointIndexAfterStop;
}


int PlannerAgv::skipToPlannerTime(double time)
{
	_plannerStartTime = time;
	return PLANNER_SUCCEED;
}


int PlannerAgv::_transferVelToFlange(double nextVel[6])
{
	double direc[3] = {0};
	double tracVelInTool[3] = {0};
	_lastTracRefToolInv.getPos(direc);

	cross3(tracVelInTool,nextVel+TRANS_D ,direc);

	double matrix[3][3] = {{0}};
	rpy2matrix(matrix,_lastTracTargetTcpPos+TRANS_D);

	double tracVelInWord[3] = {0};
	M3p3(tracVelInWord,matrix,tracVelInTool);

	nextVel[0] += tracVelInWord[0];
	nextVel[1] += tracVelInWord[1];
	nextVel[2] += tracVelInWord[2];

	return 0;
}


int PlannerAgv::getUnfinishedCircleCenterAngle(double& angle)
{
	angle = _newCenterAngleAfterStop;

	return PLANNER_SUCCEED;
}


int PlannerAgv::moveT(double targetW,bool rel, double refTime)
{
	if (PLANNER_DONE == _plannerState)
	{
		if (rel)
		{
			_targetTurnOfNextMotionOfFlange = _lastTracTargetTurnOfFlange + targetW;
		}
		else
		{
			_targetTurnOfNextMotionOfFlange = targetW;
		}

		_prepareTurning();
		return PLANNER_SUCCEED;
	}
	else
	{
		return PLANNER_NOTYET;
	}
}


int PlannerAgv::getTargetTurnOfNextMotionOfFlange(double& turn)
{
	turn = _targetTurnOfNextMotionOfFlange;

	return PLANNER_SUCCEED;
}


int PlannerAgv::getUnfinishedCircleMiddlePoint(double midPoint[])
{
	if (midPoint == nullptr)
	{
		DUMP_ERROR("ERR: the input ptr is invalid in <PlannerAgv::getUnfinishedCircleMiddlePoint>\n");

		return -1;
	}

	midPoint[TARGET_X] = _newMiddlePointAfterStop[TARGET_X];
	midPoint[TARGET_Y] = _newMiddlePointAfterStop[TARGET_Y];

	return PLANNER_SUCCEED;
}


int PlannerAgv::getUnfinishedBlenderMiddlePoint(double midPoint[])
{
	if (midPoint == nullptr)
	{
		DUMP_ERROR("ERR: the input ptr is invalid in <PlannerAgv::getUnfinishedBlenderMiddlePoint>\n");

		return -1;
	}

	midPoint[TARGET_X] = _newBlenderPointAfterStop[TARGET_X];
	midPoint[TARGET_Y] = _newBlenderPointAfterStop[TARGET_Y];
	midPoint[TARGET_A] = _newBlenderPointAfterStop[ROT_INDEX-TRANS_D];

	return PLANNER_SUCCEED;
}


int PlannerAgv::updateCurrentPos(double currentPos[], double refTool[])
{
	switch(_plannerState)
	{
	default:
		return PLANNER_NOTYET;
		break;

	case PLANNER_DONE:
	case PLANNER_WAITING:
	case PLANNER_ZONE_IN:
		break;
	}

	double refCurrentPos[6] = {0};
	refCurrentPos[TARGET_X] = currentPos[TARGET_X];
	refCurrentPos[TARGET_Y] = currentPos[TARGET_Y];
	refCurrentPos[ROT_INDEX] = currentPos[TARGET_A];

	// check tool
	bool sameTool = true;
	if (refTool != nullptr)
	{
		double tool[6] = {0};
		tool[TARGET_X] = refTool[TARGET_X];
		tool[TARGET_Y] = refTool[TARGET_Y];
		tool[ROT_INDEX] = refTool[TARGET_A];

		for (int i = 0; i < 6; i++)
		{
			if (ABS(tool[i] - _lastTracRefTool[i]) > ALMOST_ZERO)
			{
				sameTool = false;
				break;
			}
		}

		if ((!sameTool) && (PLANNER_ZONE_IN == _plannerState))
		{
			return PLANNER_NOTYET;
		}

		if (!sameTool)
		{
			Point6D tmpTool; 
			tmpTool.cartesianZero();
			tmpTool.setWrench(tool);
			_lastTracRefToolInv = tmpTool.getInverse();
			memcpy(_lastTracRefTool,tool,sizeof(_lastTracRefTool));

			// flange --> tool
			Point6D flange;
			flange.cartesianZero();
			flange.setWrench(refCurrentPos);
			(flange*tmpTool).getWrench(refCurrentPos);
		}
	}

	// update current pos of TCP
	_lastTracTargetTurnOfFlange = currentPos[TARGET_W];
	memcpy(_lastTracTargetTcpPos,refCurrentPos,sizeof(_lastTracTargetTcpPos));

	return PLANNER_SUCCEED;
};


int PlannerAgv::_transferAccToFlange(double nextVel[6], double nextAcc[6])
{
	double direc[3] = {0};
	double tracAccInTool[3] = {0};
	_lastTracRefToolInv.getPos(direc);

	cross3(tracAccInTool,nextAcc+TRANS_D ,direc);

	double tmpOmega = normN(nextVel+TRANS_D,ROTATE_D);
	double tmpNorm = dotN(nextVel+TRANS_D,direc,ROTATE_D);
	tracAccInTool[0] += tmpNorm*nextVel[0+TRANS_D] - tmpOmega*tmpOmega*direc[0];
	tracAccInTool[1] += tmpNorm*nextVel[1+TRANS_D] - tmpOmega*tmpOmega*direc[1];
	tracAccInTool[2] += tmpNorm*nextVel[2+TRANS_D] - tmpOmega*tmpOmega*direc[2];

	double matrix[3][3] = {{0}};
	rpy2matrix(matrix,_lastTracTargetTcpPos+TRANS_D);

	double tracAccInWord[3] = {0};
	M3p3(tracAccInWord,matrix,tracAccInTool);

	nextAcc[0] += tracAccInWord[0];
	nextAcc[1] += tracAccInWord[1];
	nextAcc[2] += tracAccInWord[2];

	return 0;
}


int PlannerAgv::getStartAndSwitchTime(double& startTime, double& switchTime)
{
	startTime = _plannerStartTime;
	switchTime = _plannerSwitchTime;

	return PLANNER_SUCCEED;
}


int PlannerAgv::_getTarget(double nextPos[], double nextVel[], double nextAcc[])
{
	double tmpPos[6] = {0};
	double tmpVel[6] = {0};
	double tmpAcc[6] = {0};

	//----------------------------------
	if (_tracPtr != nullptr)
	{
		if (_tracPtr->isValid())
		{
			_tracPtr->move(_plannerStartTime,tmpPos,tmpVel,tmpAcc);
		}
		else
		{
			_plannerState = ERR_PLAN_FAILED;
			return _plannerState;
		}
	}
	else
	{
		_plannerState = ERR_PLAN_FAILED;
		return _plannerState;
	}

	//----------------------------------
	if (_rotPtr != nullptr)
	{
		if (_rotPtr->isValid())
		{
			_rotPtr->move(_plannerStartTime,tmpPos+TRANS_D,tmpVel+TRANS_D,tmpAcc+TRANS_D);
		}
		else
		{
			_plannerState = ERR_PLAN_FAILED;
			return _plannerState;
		}
	}
	else
	{
		_plannerState = ERR_PLAN_FAILED;
		return _plannerState;
	}

	// save last target of TCP
	memcpy(_lastTracTargetTcpPos,tmpPos,sizeof(_lastTracTargetTcpPos));

	// pos transfer to flange
	Point6D tmpToolTarget; 
	tmpToolTarget.setWrench(tmpPos);
	(tmpToolTarget*_lastTracRefToolInv).getWrench(tmpPos);

	//vel transfer to flange
	_transferVelToFlange(tmpVel);

	// acc transfer to flange
	_transferAccToFlange(tmpVel,tmpAcc);

	// flange output
	nextPos[TARGET_X] = tmpPos[0]; nextPos[TARGET_Y] = tmpPos[1];nextPos[TARGET_A] = tmpPos[ROT_INDEX];
	nextVel[TARGET_X] = tmpVel[0]; nextVel[TARGET_Y] = tmpVel[1];nextVel[TARGET_A] = tmpVel[ROT_INDEX];
	nextAcc[TARGET_X] = tmpAcc[0]; nextAcc[TARGET_Y] = tmpAcc[1];nextAcc[TARGET_A] = tmpAcc[ROT_INDEX];
	
	nextVel[TARGET_W] = headingAngularVelIn2D(nextVel,nextAcc);
	nextAcc[TARGET_W] = 0.0;// ignored ...
	
	switch(_tracType)
	{
	case PURE_TURNING:
		nextVel[TARGET_W] = _turningPlanner.vel(_plannerStartTime);
		nextPos[TARGET_W] = radDistance(0,_turningPlanner.pos(_plannerStartTime));
		break;

	case PURE_BLENDING:
		nextPos[TARGET_W] = headingDirectionIn2D(nextVel,_lastTracTargetTurnOfFlange);
		break;

	case NORMAL_TRAC:
	case PURE_ROTATING:
		nextPos[TARGET_W] = headingDirectionIn2D(nextVel,_lastTracTargetTurnOfFlange);
		break;
	}

	// save last target turn of flange
	_lastTracTargetTurnOfFlange = nextPos[TARGET_W];

	return _plannerState;
}


int PlannerAgv::_getTargetTurnOfFlange(double&targetTurn, TRAC* tPtr, TRAC* rPtr)
{
	double nextPos[6] = {0};
	double nextVel[6] = {0};

	tPtr->pos(TIME_SCALE_MS/1000.0,nextPos);
	tPtr->vel(TIME_SCALE_MS/1000.0,nextVel);

	rPtr->pos(TIME_SCALE_MS/1000.0,nextPos+TRANS_D);
	rPtr->vel(TIME_SCALE_MS/1000.0,nextVel+TRANS_D);

	double direc[3] = {0};
	double tracVelInTool[3] = {0};
	_lastTracRefToolInv.getPos(direc);

	cross3(tracVelInTool,nextVel+TRANS_D ,direc);

	double matrix[3][3] = {{0}};
	rpy2matrix(matrix,nextPos+TRANS_D);

	double tracVelInWord[3] = {0};
	M3p3(tracVelInWord,matrix,tracVelInTool);

	nextVel[0] += tracVelInWord[0];
	nextVel[1] += tracVelInWord[1];
	nextVel[2] += tracVelInWord[2];

	targetTurn = headingDirectionIn2D(nextVel,_lastTracTargetTurnOfFlange);

	return PLANNER_SUCCEED;
}


int PlannerAgv::move(int step, double nextPos[], double nextVel[], double nextAcc[])
{
	switch(_plannerState)
	{
	case PLANNER_DONE:
		break;

	case PLANNER_MOVING:
		_plannerState = _getTarget(nextPos,nextVel,nextAcc);
		_plannerStartTime += step/1000.0;
		if (_plannerStartTime > _plannerSwitchTime)
		{
			_plannerState = PLANNER_ZONE_IN;
		}
		break;

	case PLANNER_ZONE_IN:
		switch(_moveZ())
		{
		case PLANNER_NOZONE:
			_plannerState = PLANNER_DONE;
			break;

		case PLANNER_NONEXT:
		case PLANNER_SUCCEED:
			_plannerState = PLANNER_ZONING;
			goto ZONING;
			break;

		case ERR_PLAN_FAILED:
			DUMP_INFORM("ERR: blender failed!\n");
			_plannerState = PLANNER_ZONING;
			break;
		}
		break;

	case PLANNER_ZONING:
ZONING:
		_plannerState = _getTarget(nextPos,nextVel,nextAcc);
		_plannerStartTime += step/1000.0;
		if (_plannerStartTime > _plannerSwitchTime)
		{
			switch(_blenderState)
			{
			case PLANNER_NONEXT:
				_plannerState = PLANNER_DONE;
				break;

			case PLANNER_SUCCEED:
				_plannerState = PLANNER_ZONEOUT;
				break;
			}
		}
		break;

	case PLANNER_ZONEOUT:
		DUMP_ERROR("ERR: something is wrong with in moveJ/moveL/moveC/moveR/moveB!\n");
		_plannerState = ERR_PLAN_FAILED;
		break;

	case PLANNER_WAITING:
		_plannerStartTime += step/1000.0;
		if (_plannerStartTime > _plannerSwitchTime)
		{
			_plannerState = PLANNER_DONE;
		}
		break;

	case ERR_PLAN_FAILED:
		break;
	}

	return _plannerState;
}


int PlannerAgv::setCartesianLimits(double tvl,double tal,double tjl,double rvl,double ral,double rjl)
{
	_rotVelLimit = ABS(rvl);
	_rotAccLimit = ABS(ral);
	_rotJerkLimit = ABS(rjl);
	_tracVelLimit = ABS(tvl);
	_tracAccLimit = ABS(tal);
	_tracJerkLimit = ABS(tjl);
	return PLANNER_SUCCEED;
}


int PlannerAgv::moveB(double targetPos[][3], int pNum, double refVel, double zArea, int zType,bool rel,double refTime,double turnTime)
{
	switch(_plannerState)
	{
	case PLANNER_DONE:
	case PLANNER_ZONE_IN:
		goto PLAN_NEXT_MOTION;
		break;

	case PLANNER_ZONEOUT:
		goto SWITCH_PLANNERS;
		break;

	default:
		return PLANNER_NOTYET;
		break;
	}


PLAN_NEXT_MOTION://-------------------------------------------------
	// plan trajectory
	if (!_isPostPlannerPlanned)
	{
		// plan translation
		double targetP[POINTS_MAX_NUM][TRANS_D] = {{0}};
		if (nullptr == _blendPlanner.getPrevTrac())
		{
			targetP[0][TARGET_X] = _lastTracTargetTcpPos[TARGET_X];
			targetP[0][TARGET_Y] = _lastTracTargetTcpPos[TARGET_Y];
		}
		else
		{
			_blendPlanner.getPrevTrac()->getTarget(targetP[0]);
		}

		for (int i=1; i<= pNum; i++)
		{
			memcpy(targetP[i],targetPos[i-1],sizeof(double)*2);
			if (rel)
			{
				for (int j=0; j<2; j++)
				{
					targetP[i][j] += targetP[0][j];
				}
			}
		}
		_postBsplinePlanner.initiate();
		if (refTime <= 0)
		{
			if (!_postBsplinePlanner.planTrajectory(0,100,targetP,pNum+1))
			{
				return ERR_PLAN_FAILED;
			}

			refTime = 1.5 * _postBsplinePlanner.getChordLength()/(refVel*_tracVelLimit);
		}

		_postBsplinePlanner.planTrajectory(0,refTime,targetP,pNum+1);
		if (!_postBsplinePlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// update blender zone if necessary
		_blendPlanner.setPostTrac(&_postBsplinePlanner);
		_rotTimeOffset = _blendPlanner.getPostTime();

		// plan rotate
		double sA[ROTATE_D] = {0};
		double time[POINTS_MAX_NUM] = {0};
		double deltaA[POINTS_MAX_NUM] = {0};
		double rotAxis[POINTS_MAX_NUM][ROTATE_D] = {{0}};
		if (_rotPtr != nullptr)
		{
			_rotPtr->getTarget(sA);
		}
		else
		{
			sA[ROT_INDEX-TRANS_D] = _lastTracTargetTcpPos[ROT_INDEX];
		}
		_postBsplinePlanner.getTimeSequence(time);

		time[0] = time[0] + _rotTimeOffset;
		rotAxis[0][ROT_INDEX-TRANS_D] = 1.0;
		deltaA[0] = radDistance(sA[ROT_INDEX-TRANS_D],targetPos[0][TARGET_A]);
		for (int i=1; i<pNum; i++)
		{
			time[i] = time[i] + _rotTimeOffset;
			rotAxis[i][ROT_INDEX-TRANS_D] = 1.0;
			deltaA[i] = radDistance(targetPos[i-1][TARGET_A], targetPos[i][TARGET_A]);

			if (rel)
			{
				deltaA[i] = targetPos[i][TARGET_A];
			}
		}
		time[0] += _rotTimeOffset;
		_postMultiRotPlanner.initiate();
		_postMultiRotPlanner.setLimit(refVel*_rotVelLimit,_rotAccLimit,_rotJerkLimit);

		_postMultiRotPlanner.planTrajectory(time,sA,pNum+1,rotAxis,deltaA);
		if (!_postMultiRotPlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// synchronize
		if (_postMultiRotPlanner.isTimeChanged())
		{
			_postMultiRotPlanner.getTimeSequence(time);
			for (int i=0; i<pNum+1; i++)
			{
				time[i] -= _rotTimeOffset;
			}
			_postBsplinePlanner.planTrajectory(time,targetP,pNum+1);
			if (!_postBsplinePlanner.isValid())
			{
				return ERR_PLAN_FAILED;
			}
		}
		_isPostPlannerPlanned = true;
	}


//TURN_DIRECTION:-------------------------------------------------
	_refTurnTimeOfNextMotionOfFlange = turnTime;
	_getTargetTurnOfFlange(_targetTurnOfNextMotionOfFlange,&_postBsplinePlanner,&_postMultiRotPlanner);

	switch(_plannerState)
	{
	case PLANNER_DONE:
		return _prepareTurning();
		break;

	case PLANNER_ZONE_IN:
		return PLANNER_NOTYET;
		break;
	}


SWITCH_PLANNERS://-------------------------------------------------
	if (_blendPlanner.getPostTrac() != nullptr && nullptr != _blendPlanner.getPrevTrac())
	{
		double burr = _plannerStartTime - _blendPlanner.getTargetTime();
		_plannerStartTime = _blendPlanner.getPostTime() + burr;
	}
	else
	{
		_plannerStartTime = 0;
	}
	_prevMultiRotPlanner = _postMultiRotPlanner;
	_prevBsplinePlanner = _postBsplinePlanner;
	_blendPlanner.initiate();

	if (zArea > 0)
	{
		_blendPlanner.setBlendZone(zType,zArea);
		_blendPlanner.setPrevTrac(&_prevBsplinePlanner);
		_plannerSwitchTime = _blendPlanner.getPrevTime();
	}
	else
	{
		_plannerSwitchTime = _prevBsplinePlanner.getTargetTime();
	}


//UPDATE_STATE:-------------------------------------------------
	_tracType = NORMAL_TRAC;
	_isPostPlannerPlanned = false;
	_plannerState = PLANNER_MOVING;
	_rotPtr = &_prevMultiRotPlanner;
	_tracPtr = &_prevBsplinePlanner;
	DUMP_INFORM("Moving B-spline ... \n");
	return PLANNER_SUCCEED;
}


int PlannerAgv::moveR(double targetPos[], double refDir, double refVel, double refAcc, double refJerk, int rCycle, bool rel, double refTime,double turnTime)
{
	switch(_plannerState)
	{
	case PLANNER_DONE:
		goto PLAN_NEXT_MOTION;
		break;

	case PLANNER_ZONEOUT:
		goto SWITCH_PLANNERS;
		break;

	default:
		return PLANNER_NOTYET;
		break;
	}


PLAN_NEXT_MOTION://-------------------------------------------------
	if (!_isPostPlannerPlanned)
	{
		// plan translation
		double sP[TRANS_D] = {0};
		sP[TARGET_X] = _lastTracTargetTcpPos[TARGET_X];
		sP[TARGET_Y] = _lastTracTargetTcpPos[TARGET_Y];

		double tP[TRANS_D] = {0};
		tP[TARGET_X] = _lastTracTargetTcpPos[TARGET_X];
		tP[TARGET_Y] = _lastTracTargetTcpPos[TARGET_Y];

		_postLinePlanner.initiate();
		_postLinePlanner.setLimit(refVel*_tracVelLimit,refAcc*_tracAccLimit,refJerk*_tracJerkLimit);
		_postLinePlanner.planTrajectory(0,sP,tP);
		if (!_postLinePlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// plan rotation
		double sR[ROTATE_D] = {0};
		double axis[ROTATE_D] = {0};
		axis[ROT_INDEX-TRANS_D] = 1.0;
		sR[ROT_INDEX-TRANS_D] = _lastTracTargetTcpPos[ROT_INDEX];

		double dR = (rel?targetPos[TARGET_A]:radDistance(sR[ROT_INDEX-TRANS_D], targetPos[TARGET_A]));
		dR += 2.0*PI*(rCycle+((dR*rCycle>=0)?(-sign(rCycle)):0));

		_postRotPlanner.initiate();
		_postRotPlanner.setLimit(refVel*_rotVelLimit,refAcc*_rotAccLimit,refJerk*_rotJerkLimit);
		_postRotPlanner.planTrajectory(0,sR,axis,dR);
		if (!_postRotPlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// synchronize
		if (_postRotPlanner.getDuration() > _postLinePlanner.getDuration())
		{
			_postLinePlanner.scaleToDuration(_postRotPlanner.getDuration());
		}
		
		_isPostPlannerPlanned = true;
	}


//TURN_DIRECTION:-------------------------------------------------
	double tmp = 0;
	tmp = _lastTracTargetTurnOfFlange;
	_lastTracTargetTurnOfFlange = refDir;
	_refTurnTimeOfNextMotionOfFlange = turnTime;
	_getTargetTurnOfFlange(_targetTurnOfNextMotionOfFlange,&_postLinePlanner,&_postRotPlanner);
	_lastTracTargetTurnOfFlange = tmp;

	switch(_plannerState)
	{
	case PLANNER_DONE:
		return _prepareTurning();
		break;

	case PLANNER_ZONE_IN:
		return PLANNER_NOTYET;
		break;
	}


SWITCH_PLANNERS://-------------------------------------------------
	_plannerSwitchTime = _prevRotPlanner.getTargetTime();
	_prevLinePlanner = _postLinePlanner;
	_prevRotPlanner = _postRotPlanner;
	_blendPlanner.initiate();
	_plannerStartTime = 0;


//UPDATE_STATE:-------------------------------------------------
	_plannerStartTime = 0;
	_blendPlanner.initiate();
	_tracType = PURE_ROTATING;
	_rotPtr = &_prevRotPlanner;
	_tracPtr = &_prevLinePlanner;
	_isPostPlannerPlanned = false;
	_plannerState = PLANNER_MOVING;
	DUMP_INFORM("Moving rotate ... \n");
	return PLANNER_SUCCEED;
}


int PlannerAgv::moveL(double targetPos[], double refVel, double refAcc, double refJerk, double zArea, int zType, int rCycle, bool rel,double refTime,double turnTime)
{
	switch(_plannerState)
	{
	case PLANNER_DONE:
	case PLANNER_ZONE_IN:
		goto PLAN_NEXT_MOTION;
		break;

	case PLANNER_ZONEOUT:
		goto SWITCH_PLANNERS;
		break;

	default:
		return PLANNER_NOTYET;
		break;
	}


PLAN_NEXT_MOTION://-------------------------------------------------
	if (!_isPostPlannerPlanned)
	{
		// plan translation
		double sP[TRANS_D] = {0};
		sP[TARGET_X] = _lastTracTargetTcpPos[TARGET_X];
		sP[TARGET_Y] = _lastTracTargetTcpPos[TARGET_Y];

		if (nullptr != _blendPlanner.getPrevTrac())
		{
			_blendPlanner.getPrevTrac()->getTarget(sP);
		}

		double tP[TRANS_D] = {0};
		tP[TARGET_X] = targetPos[TARGET_X];
		tP[TARGET_Y] = targetPos[TARGET_Y];

		if (rel)
		{
			tP[TARGET_X] += sP[TARGET_X];
			tP[TARGET_Y] += sP[TARGET_Y];
		}

		_postLinePlanner.initiate();
		_postLinePlanner.setLimit(refVel*_tracVelLimit,refAcc*_tracAccLimit,refJerk*_tracJerkLimit);
		_postLinePlanner.planTrajectory(0,sP,tP);
		if (!_postLinePlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// update blender if necessary
		_blendPlanner.setPostTrac(&_postLinePlanner);
		_rotTimeOffset = _blendPlanner.getPostTime();

		// plan rotate
		double sR[ROTATE_D] = {0};
		double axis[ROTATE_D] = {0};
		axis[ROT_INDEX-TRANS_D] = 1.0;
		sR[ROT_INDEX-TRANS_D] = _lastTracTargetTcpPos[ROT_INDEX];

		if (nullptr != _rotPtr)
		{
			_rotPtr->getTarget(sR);
		}

		double dR = (rel?targetPos[TARGET_A]:radDistance(sR[ROT_INDEX-TRANS_D], targetPos[TARGET_A]));
		dR += 2.0*PI*(rCycle+((dR*rCycle>=0)?(-sign(rCycle)):0));

		_postRotPlanner.initiate();
		_postRotPlanner.setLimit(refVel*_rotVelLimit,refAcc*_rotAccLimit,refJerk*_rotJerkLimit);
		_postRotPlanner.planTrajectory(_rotTimeOffset,sR,axis,dR);
		if (!_postRotPlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// synchronize
		if (_postRotPlanner.getTargetTime() > _postLinePlanner.getTargetTime())
		{
			_postLinePlanner.scaleToDuration(_postRotPlanner.getDuration()+_rotTimeOffset);
		}
		if (_postLinePlanner.getTargetTime() > _postRotPlanner.getTargetTime())
		{
			_postRotPlanner.scaleToDuration(_postLinePlanner.getDuration() - _rotTimeOffset);
		}
		_isPostPlannerPlanned = true;
	}


//TURN_DIRECTION:-------------------------------------------------
	_refTurnTimeOfNextMotionOfFlange = turnTime;
	_getTargetTurnOfFlange(_targetTurnOfNextMotionOfFlange,&_postLinePlanner,&_postRotPlanner);

	switch(_plannerState)
	{
	case PLANNER_DONE:
		return _prepareTurning();
		break;

	case PLANNER_ZONE_IN:
		return PLANNER_NOTYET;
		break;
	}


SWITCH_PLANNERS://-------------------------------------------------
	if (_blendPlanner.getPostTrac() != nullptr && nullptr != _blendPlanner.getPrevTrac() )
	{
		double burr = _plannerStartTime - _blendPlanner.getTargetTime();
		_plannerStartTime = _blendPlanner.getPostTime() + burr;
	}
	else
	{
		_plannerStartTime = 0;
	}
	_prevLinePlanner = _postLinePlanner;
	_prevRotPlanner = _postRotPlanner;
	_blendPlanner.initiate();
	if (zArea > 0)
	{
		_blendPlanner.setBlendZone(zType,zArea);
		_blendPlanner.setPrevTrac(&_prevLinePlanner);
		_plannerSwitchTime = _blendPlanner.getPrevTime();
	}
	else
	{
		_plannerSwitchTime = _prevLinePlanner.getTargetTime();
	}


//UPDATE_STATE:-------------------------------------------------
	_tracType = NORMAL_TRAC;
	_rotPtr = &_prevRotPlanner;
	_tracPtr = &_prevLinePlanner;
	_plannerState = PLANNER_MOVING;
	_isPostPlannerPlanned = false;
	DUMP_INFORM("Moving Line ... \n");

	return PLANNER_SUCCEED;
}


int PlannerAgv::moveC(double targetPos[], double middlePos[], double refVel, double refAcc, double refJerk, double zArea, int zType, int cCycle,int rCycle,bool rel, double refTime,double turnTime)
{
	switch(_plannerState)
	{
	case PLANNER_DONE:
	case PLANNER_ZONE_IN:
		goto PLAN_NEXT_MOTION;
		break;

	case PLANNER_ZONEOUT:
		goto SWITCH_PLANNERS;
		break;

	default:
		return PLANNER_NOTYET;
		break;
	}


PLAN_NEXT_MOTION://-------------------------------------------------
	if (!_isPostPlannerPlanned)
	{
		// plan translation
		double sP[TRANS_D] = {0};
		sP[TARGET_X] = _lastTracTargetTcpPos[TARGET_X];
		sP[TARGET_Y] = _lastTracTargetTcpPos[TARGET_Y];

		if (nullptr != _blendPlanner.getPrevTrac())
		{
			_blendPlanner.getPrevTrac()->getTarget(sP);
		}

		double mP[TRANS_D] = {0};
		mP[TARGET_X] = middlePos[TARGET_X];
		mP[TARGET_Y] = middlePos[TARGET_Y];

		double tP[TRANS_D] ={0};
		tP[TARGET_X] = targetPos[TARGET_X];
		tP[TARGET_Y] = targetPos[TARGET_Y];

		if (rel)
		{
			mP[TARGET_X] += sP[TARGET_X];
			mP[TARGET_Y] += sP[TARGET_Y];

			tP[TARGET_X] += sP[TARGET_X];
			tP[TARGET_Y] += sP[TARGET_Y];
		}

		_postCirclePlanner.initiate();
		_postCirclePlanner.setLimit(refVel*_tracVelLimit,refAcc*_tracAccLimit,refJerk*_tracJerkLimit);
		_postCirclePlanner.planTrajectory(0,sP,mP,tP,cCycle);
		if (!_postCirclePlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// update blender if necessary
		_blendPlanner.setPostTrac(&_postCirclePlanner);
		double timeOffset = _blendPlanner.getPostTime();

		// plan rotate
		double sR[ROTATE_D] = {0};
		double axis[ROTATE_D] = {0};
		axis[ROT_INDEX-TRANS_D] = 1.0;
		sR[ROT_INDEX-TRANS_D] = _lastTracTargetTcpPos[ROT_INDEX];

		if (nullptr != _rotPtr)
		{
			_rotPtr->getTarget(sR);
		}

		double dR = (rel?targetPos[TARGET_A]:radDistance(sR[ROT_INDEX-TRANS_D], targetPos[TARGET_A]));
		dR += 2.0*PI*(rCycle+((dR*rCycle>=0)?(-sign(rCycle)):0));

		_postRotPlanner.initiate();
		_postRotPlanner.setLimit(refVel*_rotVelLimit,refAcc*_rotAccLimit,refJerk*_rotJerkLimit);
		_postRotPlanner.planTrajectory(timeOffset,sR,axis,dR);
		if (!_postRotPlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// synchronize
		if (_postRotPlanner.getTargetTime() > _postCirclePlanner.getTargetTime())
		{
			_postCirclePlanner.scaleToDuration(_postRotPlanner.getDuration()+timeOffset);
		}
		if (_postCirclePlanner.getTargetTime() > _postRotPlanner.getTargetTime())
		{
			_postRotPlanner.scaleToDuration(_postCirclePlanner.getTargetTime()-timeOffset);
		}
		_isPostPlannerPlanned = true;
	}


//TURN_DIRECTION:-------------------------------------------------
	_refTurnTimeOfNextMotionOfFlange = turnTime;
	_getTargetTurnOfFlange(_targetTurnOfNextMotionOfFlange,&_postCirclePlanner,&_postRotPlanner);

	switch(_plannerState)
	{
	case PLANNER_DONE:
		return _prepareTurning();
		break;

	case PLANNER_ZONE_IN:
		return PLANNER_NOTYET;
		break;
	}


SWITCH_PLANNERS://-------------------------------------------------
	if (_blendPlanner.getPostTrac() != nullptr && nullptr != _blendPlanner.getPrevTrac())
	{
		double burr = _plannerStartTime - _blendPlanner.getTargetTime();
		_plannerStartTime = _blendPlanner.getPostTime() + burr;
	}
	else
	{
		_plannerStartTime = 0;
	}
	_prevCirclePlanner = _postCirclePlanner;
	_prevRotPlanner = _postRotPlanner;
	_blendPlanner.initiate();
	if (zArea > 0)
	{
		_blendPlanner.setBlendZone(zType,zArea);
		_blendPlanner.setPrevTrac(&_prevCirclePlanner);
		_plannerSwitchTime = _blendPlanner.getPrevTime();
	}
	else
	{
		_plannerSwitchTime = _prevCirclePlanner.getTargetTime();
	}


//UPDATE_STATE:-------------------------------------------------
	_tracType = NORMAL_TRAC;
	_rotPtr = &_prevRotPlanner;
	_tracPtr = &_prevCirclePlanner;
	_plannerState = PLANNER_MOVING;
	_isPostPlannerPlanned = false;
	DUMP_INFORM("Moving Circle ... \n");

	return PLANNER_SUCCEED;
}


int PlannerAgv::moveC(double centerPoint[],double centerAngle, double targetA, double refVel, double refAcc,double refJerk, double zArea, int zType, int rCycle,bool rel, double refTime,double turnTime)
{
	switch(_plannerState)
	{
	case PLANNER_DONE:
	case PLANNER_ZONE_IN:
		goto PLAN_NEXT_MOTION;
		break;

	case PLANNER_ZONEOUT:
		goto SWITCH_PLANNERS;
		break;

	default:
		return PLANNER_NOTYET;
		break;
	}


PLAN_NEXT_MOTION://-------------------------------------------------
	if (!_isPostPlannerPlanned)
	{
		// plan translation
		double sP[TRANS_D] = {0};
		sP[TARGET_X] = _lastTracTargetTcpPos[TARGET_X];
		sP[TARGET_Y] = _lastTracTargetTcpPos[TARGET_Y];

		if (nullptr != _blendPlanner.getPrevTrac())
		{
			_blendPlanner.getPrevTrac()->getTarget(sP);
		}

		double cP[TRANS_D] = {0};
		cP[TARGET_X] = centerPoint[TARGET_X];
		cP[TARGET_Y] = centerPoint[TARGET_Y];

		if (rel)
		{
			cP[TARGET_X] += sP[TARGET_X];
			cP[TARGET_Y] += sP[TARGET_Y];
		}

		double nV[TRANS_D] = {0}; 
		nV[ROT_INDEX-TRANS_D] = 1.0;
		
		double cA = centerAngle;

		_postCirclePlanner.initiate();
		_postCirclePlanner.setLimit(refVel*_tracVelLimit,refAcc*_tracAccLimit,refJerk*_tracJerkLimit);
		_postCirclePlanner.planTrajectory(0,sP,cP,nV,cA);
		if (!_postCirclePlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// update blender zone if necessary
		_blendPlanner.setPostTrac(&_postCirclePlanner);
		double timeOffset = _blendPlanner.getPostTime();

		// plan rotate
		double sR[ROTATE_D] = {0};
		double axis[ROTATE_D] = {0};
		axis[ROT_INDEX-TRANS_D] = 1.0;
		sR[ROT_INDEX-TRANS_D] = _lastTracTargetTcpPos[ROT_INDEX];

		if (nullptr != _rotPtr)
		{
			_rotPtr->getTarget(sR);
		}

		double dR = (rel?targetA:radDistance(sR[ROT_INDEX-TRANS_D], targetA));
		dR += 2.0*PI*(rCycle+((dR*rCycle>=0)?(-sign(rCycle)):0));

		_postRotPlanner.initiate();
		_postRotPlanner.setLimit(refVel*_rotVelLimit,refAcc*_rotAccLimit,refJerk*_rotJerkLimit);
		_postRotPlanner.planTrajectory(timeOffset,sR,axis,dR);
		if (!_postRotPlanner.isValid())
		{
			return ERR_PLAN_FAILED;
		}

		// synchronize
		if (_postRotPlanner.getTargetTime() > _postCirclePlanner.getTargetTime())
		{
			_postCirclePlanner.scaleToDuration(_postRotPlanner.getDuration()+timeOffset);
		}
		if (_postCirclePlanner.getTargetTime() > _postRotPlanner.getTargetTime())
		{
			_postRotPlanner.scaleToDuration(_postCirclePlanner.getTargetTime()-timeOffset);
		}
		_isPostPlannerPlanned = true;
	}


//TURN_DIRECTION:-------------------------------------------------
	_refTurnTimeOfNextMotionOfFlange = turnTime;
	_getTargetTurnOfFlange(_targetTurnOfNextMotionOfFlange,&_postCirclePlanner,&_postRotPlanner);

	switch(_plannerState)
	{
	case PLANNER_DONE:
		return _prepareTurning();
		break;

	case PLANNER_ZONE_IN:
		return PLANNER_NOTYET;
		break;
	}


SWITCH_PLANNERS://-------------------------------------------------
	if (_blendPlanner.getPostTrac() != nullptr && nullptr != _blendPlanner.getPrevTrac())
	{
		double burr = _plannerStartTime - _blendPlanner.getTargetTime();
		_plannerStartTime = _blendPlanner.getPostTime() + burr;
	}
	else
	{
		_plannerStartTime = 0;
	}
	_prevCirclePlanner = _postCirclePlanner;
	_prevRotPlanner = _postRotPlanner;
	_blendPlanner.initiate();
	if (zArea > 0)
	{
		_blendPlanner.setBlendZone(zType,zArea);
		_blendPlanner.setPrevTrac(&_prevCirclePlanner);
		_plannerSwitchTime = _blendPlanner.getPrevTime();
	}
	else
	{
		_plannerSwitchTime = _prevCirclePlanner.getTargetTime();
	}


//UPDATE_STATE:-------------------------------------------------
	_tracType = NORMAL_TRAC;
	_rotPtr = &_prevRotPlanner;
	_tracPtr = &_prevCirclePlanner;
	_plannerState = PLANNER_MOVING;
	_isPostPlannerPlanned = false;
	DUMP_INFORM("Moving Circle ... \n");

	return PLANNER_SUCCEED;
}







PlannerOpt::PlannerOpt()
{
	_step = PLAN_CYCLE;
	memset(_jointUpBound,3,sizeof(_jointUpBound));
	memset(_jointLowBound,-3,sizeof(_jointLowBound));
	memset(_lastTargetJointVel,0,sizeof(_lastTargetJointVel));
	memset(_lastTargetFlangePos,0,sizeof(_lastTargetFlangePos));
	memset(_lastTargetFlangeVel,0,sizeof(_lastTargetFlangeVel));

	_flangeMask[0] = 1;
	_flangeMask[1] = 1;
	_flangeMask[2] = 1;
	_flangeMask[3] = 1;
	_flangeMask[4] = 1;
	_flangeMask[5] = 1;
}


PlannerOpt::~PlannerOpt()
{

}


int PlannerOpt::setKine(Kine kine)
{
	_kine = kine;

	return 0;
}


int PlannerOpt::setFlangeMask(int flangeMask[6])
{
	memcpy(_flangeMask,flangeMask,sizeof(int)*6);

	return PLANNER_SUCCEED;
}


int PlannerOpt::updateCurrentJoint(int dof, double currentJointAngle[])
{
	if (PLANNER_DONE != _plannerState && PLANNER_WAITING != _plannerState)
	{
		return PLANNER_NOTYET;
	}

	_dof = dof;

	memcpy(_lastTracTargetJoint,currentJointAngle,sizeof(double)*dof);

	_kine.Fkine(_lastTargetFlangePos,_lastTracTargetJoint);

	return PLANNER_SUCCEED;
}


int PlannerOpt::_getJointVelLimit(double velUpLim[],double velLowLim[])
{
	for(int i=0; i<_dof; i++)
	{
		velUpLim[i] = 1.0;
		velLowLim[i] = 1.0;

		if (_lastTracTargetJoint[i] - _jointLowBound[i] < 0)
		{
			velLowLim[i] = 0;
		}
		else
		{
			double tmp2 = -_jointVelLimit[i];
			double tmp1 = (_jointLowBound[i]-_lastTracTargetJoint[i])/(_step/1000.0);
			double tmp3 = -sqrt(2.0*_jointAccLimit[i]*(_lastTracTargetJoint[i]-_jointLowBound[i]));

			velLowLim[i] = MAX(MAX(tmp1,tmp2),tmp3);
		}

		if (_jointUpBound[i] - _lastTracTargetJoint[i] < 0)
		{
			velUpLim[i] = 0;
		}
		else
		{
			double tmp2 = _jointVelLimit[i];
			double tmp1 = (_jointUpBound[i]-_lastTracTargetJoint[i])/(_step/1000.0);
			double tmp3 = sqrt(2.0*_jointAccLimit[i]*(_jointUpBound[i]-_lastTracTargetJoint[i]));

			velUpLim[i] = MIN(MIN(tmp1,tmp2),tmp3);    
		}

		velUpLim[i] = MIN(velUpLim[i],_lastTargetJointVel[i] + _jointAccLimit[i]*_step*2.0);
		velLowLim[i] = MAX(velLowLim[i],_lastTargetJointVel[i] - _jointAccLimit[i]*_step*2.0);
	}

	return PLANNER_SUCCEED;
}


int PlannerOpt::updateCurrentPos(double currentPos[], double refTool[])
{
	switch(_plannerState)
	{
	default:
		return PLANNER_NOTYET;
		break;

	case PLANNER_DONE:
	case PLANNER_WAITING:
	case PLANNER_ZONE_IN:
		break;
	}

	memcpy(_lastTargetFlangePos,currentPos,sizeof(_lastTargetFlangePos));

	// check tool
	bool sameTool = true;
	if (refTool != nullptr)
	{
		for (int i = 0; i < 6; i++)
		{
			if (ABS(refTool[i] - _lastTracRefTool[i]) > ALMOST_ZERO)
			{
				sameTool = false;
				break;
			}
		}

		if ((!sameTool) && (PLANNER_ZONE_IN == _plannerState))
		{
			return PLANNER_NOTYET;
		}

		if (!sameTool)
		{
			Point6D tmpTool; 
			tmpTool.cartesianZero();
			tmpTool.setWrench(refTool);
			_lastTracRefToolInv = tmpTool.getInverse();
			memcpy(_lastTracRefTool,refTool,sizeof(_lastTracRefTool));

			// flange --> tcp
			Point6D flange;
			flange.cartesianZero();
			flange.setWrench(currentPos);
			(flange*tmpTool).getWrench(currentPos);
		}
	}

	// update current pos of TCP
	memcpy(_lastTracTargetTcpPos,currentPos,sizeof(_lastTracTargetTcpPos));

	return PLANNER_SUCCEED;
};


int PlannerOpt::_getTarget(double nextPos[], double nextVel[], double nextAcc[])
{
	// joint space
	if (_isNowInJointSpace)
	{
		if (_axisPtpPlanner.isValid())
		{
			_axisPtpPlanner.move(_plannerStartTime,nextPos,nextVel,nextAcc);
		}
		else
		{
			_plannerState = ERR_PLAN_FAILED;
			return _plannerState;
		}
		memcpy(_lastTracTargetJoint,nextPos,sizeof(double)*_dof);

		// update flange pos/vel/acc		
		double Jacob[6][MAX_DOF];
		_kine.Jacob0(Jacob,nextPos);
		_kine.Fkine(_lastTargetFlangePos,nextPos);
		M6pN(_lastTargetFlangeVel,Jacob,nextVel,_dof);
		return _plannerState;
	}

	//----------------------------------
	if (_tracPtr != nullptr)
	{
		if (_tracPtr->isValid())
		{
			_tracPtr->move(_plannerStartTime,nextPos,nextVel,nextAcc);
		}
		else
		{
			_plannerState = ERR_PLAN_FAILED;
			return _plannerState;
		}
	}
	else
	{
		_plannerState = ERR_PLAN_FAILED;
		return _plannerState;
	}

	//----------------------------------
	if (_rotPtr != nullptr)
	{
		if (_rotPtr->isValid())
		{
			_rotPtr->move(_plannerStartTime,nextPos+TRANS_D,nextVel+TRANS_D,nextAcc+TRANS_D);
		}
		else
		{
			_plannerState = ERR_PLAN_FAILED;
			return _plannerState;
		}
	}
	else
	{
		_plannerState = ERR_PLAN_FAILED;
		return _plannerState;
	}

	// save to last target
	memcpy(_lastTracTargetTcpPos,nextPos,sizeof(_lastTracTargetTcpPos));

	// pos transfer to flange
	Point6D tmpToolTarget; 
	tmpToolTarget.setWrench(nextPos);
	(tmpToolTarget*_lastTracRefToolInv).getWrench(nextPos);

	//vel transfer to flange
	_transferVelToFlange(nextVel);

	// acc transfer to flange
	_transferAccToFlange(nextVel,nextAcc);

	// motion projection
	return _projectMotion(nextPos,nextVel,nextAcc);
}


int PlannerOpt::_projectMotion(double nextPos[], double nextVel[],double nextAcc[])
{
	int sumMask = 0;
	double scales = 1.0;
	double jointVel[MAX_DOF];
	for (int i=0; i<6; i++)
	{
		if (_flangeMask[i] != 0)
		{
			sumMask++;
		}
	}
	if (_dof <= sumMask)
	{
		if(_scaleOptForNonRedundant(scales,jointVel,nextVel))
		{
			_plannerState = ERR_PLAN_FAILED;
			return _plannerState;
		}
	}
	else
	{
		if (_scaleOptForRedundant(scales,jointVel,nextVel))
		{
			_plannerState = ERR_PLAN_FAILED;
			return _plannerState;
		}
	}

	// update
	for (int i=0; i<6; i++)
	{
		nextVel[i] = nextVel[i]*scales;
		nextAcc[i] = (nextVel[i] - _lastTargetFlangeVel[i])/(_step/1000.0);
		nextPos[i] = _lastTargetFlangePos[i] + nextVel[i]*(_step/1000.0) + 0.5*nextAcc[i]*(_step/1000.0)*(_step/1000.0);
	}

	memcpy(_lastTargetJointVel,jointVel,sizeof(jointVel));
	_kine.Ikine(_lastTracTargetJoint,nextPos,_lastTracTargetJoint);
	_plannerStartTime -= (1.0 + ALMOST_ZERO - scales)*(_step/1000.0);
	memcpy(_lastTargetFlangePos,nextPos,sizeof(_lastTargetFlangePos));
	memcpy(_lastTargetFlangeVel,nextVel,sizeof(_lastTargetFlangeVel));

	return _plannerState;
}


int PlannerOpt::move(int step, double nextPos[], double nextVel[], double nextAcc[])
{
	_step = step;
	switch(_plannerState)
	{
	case PLANNER_DONE:
		break;

	case PLANNER_MOVING:
		_plannerState = _getTarget(nextPos,nextVel,nextAcc);
		_plannerStartTime += step/1000.0;
		if (_plannerStartTime >= _plannerSwitchTime)
		{
			_plannerState = PLANNER_ZONE_IN;
		}
		break;

	case PLANNER_ZONE_IN:
		switch(_moveZ())
		{
		case PLANNER_NOZONE:
			_plannerState = PLANNER_DONE;
			break;

		case PLANNER_NONEXT:
		case PLANNER_SUCCEED:
			_plannerState = PLANNER_ZONING;
			goto ZONING;
			break;

		case ERR_PLAN_FAILED:
			DUMP_INFORM("ERR: blender failed!\n");
			_plannerState = PLANNER_ZONING;
			break;
		}
		break;

	case PLANNER_ZONING:
ZONING: _plannerState = _getTarget(nextPos,nextVel,nextAcc);
		_plannerStartTime += step/1000.0;
		if (_plannerStartTime >= _plannerSwitchTime)
		{
			switch(_blenderState)
			{
			case PLANNER_NONEXT:
				_plannerState = PLANNER_DONE;
				break;

			case PLANNER_SUCCEED:
				_plannerState = PLANNER_ZONEOUT;
				break;
			}
		}
		break;

	case PLANNER_ZONEOUT:
		DUMP_ERROR("ERR: something is wrong with in moveJ/moveL/moveC/moveR/moveB!\n");
		_plannerState = ERR_PLAN_FAILED;
		break;

	case PLANNER_WAITING:
		_plannerStartTime += step/1000.0;
		if (_plannerStartTime >= _plannerSwitchTime)
		{
			_plannerState = PLANNER_DONE;
		}
		break;

	case ERR_PLAN_FAILED:
		break;
	}

	return _plannerState;
}


int PlannerOpt::setJointRange(int dof, double jointUpBound[], double jointLowBound[])
{
	_dof = dof;
	memcpy(_jointUpBound,jointUpBound,sizeof(double)*dof);
	memcpy(_jointLowBound,jointLowBound,sizeof(double)*dof);

	return PLANNER_SUCCEED;
}


int PlannerOpt::_scaleOptForRedundant(double& scales, double jointVel[],double nextVel[])
{
	int sumMask = 0;
	for (int i=0; i<6; i++)
	{
		if (_flangeMask[i] != 0)
		{
			sumMask++;
		}
	}

	double qv_min[MAX_DOF],qv_max[MAX_DOF];
	_getJointVelLimit(qv_max,qv_min);

	double jacbo[6][MAX_DOF];
	_kine.Jacob0(jacbo,_lastTracTargetJoint);

	double invJacob[MAX_DOF][6];
	_kine.InvJacob0(invJacob,_lastTracTargetJoint);

	//-----
	for(int i = 0; i < _dof; i++)
	{
		jointVel[i] = 0;
		for(int k = 0; k < 6; k ++)
		{
			jointVel[i] += invJacob[i][k]*nextVel[k]*scales*_flangeMask[k];
		}
	}

	//------
	int indexMap[6];
	double C[6] = {0};
	double At[MAX_DOF][6];
	for (int i=0; i<_dof; i++)
	{			
		C[0] = -1.0;
		At[i][0] = 0;
		for (int j=0; j<6; j++)
		{
			At[i][0] = invJacob[i][j]*nextVel[j]*_flangeMask[j];
		}

		int index = 0;
		for (int j=0; j<6; j++)
		{
			if (_flangeMask[j] == 0)
			{
				At[i][index+1] = invJacob[i][j];
				indexMap[index] = j;
				index++;
			}
		}
	}

	//-----
	double B[2*MAX_DOF];
	double A[2*MAX_DOF*6];
	double Am[2*MAX_DOF][6];
	for (int i=0; i<_dof; i++)
	{
		for (int j=0; j<6; j++)
		{
			if (j <= sumMask)
			{
				Am[i][j] = At[i][j];
				Am[i+_dof][j] = -At[i][j];
			}
			else
			{
				Am[i][j] = 0;
				Am[i+_dof][j] = 0;
			}
		}
		B[i] = qv_max[i];
		B[i+_dof] = -qv_min[i];
	}
	for (int i=0; i<_dof*2; i++)
	{
		for (int j=0; j<= sumMask; j++)
		{
			A[i*(sumMask+1)+j] = Am[i][j];
		}
	}

	//-----
	double vlb[6]={0}, vub[6]={0}; 
	vub[0] = 1.0 + ACCURACY_FACTOR;

	for (int i=1; i<=sumMask; i++)
	{
		for (int j=0; j<_dof; j++)
		{
			int tmp = indexMap[i-1];
			double data = jacbo[tmp][j];
			if (data > 0 )
			{
				vlb[i] += data*(jointVel[j] - _jointAccLimit[j]*_step/1000.0);
				vub[i] += data*(jointVel[j] + _jointAccLimit[j]*_step/1000.0);
			}
			else
			{
				vlb[i] += data*(jointVel[j] + _jointAccLimit[j]*_step/1000.0);
				vub[i] += data*(jointVel[j] - _jointAccLimit[j]*_step/1000.0);
			}
		}
	}

	// linear programming
	double Z=-1,X[7];
	if (linprog(Z,X,C,A,B,vlb,vub,_dof*2,7-sumMask))
	{
		DUMP_ERROR("ERR: linprog can NOT get a valid value!\n");
		_plannerState = ERR_PLAN_FAILED;
		return _plannerState;
	}
	else
	{
		scales = X[0];
		for (int i=0; i<_dof; i++)
		{
			jointVel[i] = 0;
			for (int j=0; j<7-sumMask; j++)
			{
				jointVel[i] += At[i][j]*X[j];
			}
		}
	}

	return PLANNER_SUCCEED;
}


int PlannerOpt::_scaleOptForNonRedundant(double& scales, double jointVel[],double nextVel[])
{
	double qv_min[MAX_DOF],qv_max[MAX_DOF];
	_getJointVelLimit(qv_max,qv_min);

	double invJacob[MAX_DOF][6];
	_kine.InvJacob0(invJacob,_lastTracTargetJoint);

	while(true)
	{
		for(int i = 0; i < _dof; i++)
		{
			jointVel[i] = 0;
			for(int k = 0; k < 6; k ++)
			{
				jointVel[i] += invJacob[i][k]*nextVel[k]*scales*_flangeMask[k];
			}
		}

		int index = _dof;
		double maxLimit = 0;
		for (int i=0; i<_dof; i++)
		{
			double tmpLimit = MAX(MAX(jointVel[i]-qv_max[i]-ALMOST_ZERO,qv_min[i]-jointVel[i]-ALMOST_ZERO),maxLimit);
			if (tmpLimit > maxLimit)
			{
				index = i;
				maxLimit = tmpLimit;
			}
		}

		if (index < _dof)
		{
			double Smin[MAX_DOF] = {0};
			double Smax[MAX_DOF] = {0};
			for (int i=0; i<_dof; i++)
			{
				if (ABS(jointVel[i]) > ALMOST_ZERO )
				{
					Smin[i] = MIN(qv_min[i]/jointVel[i],qv_max[i]/jointVel[i]);
					Smax[i] = MAX(qv_min[i]/jointVel[i],qv_max[i]/jointVel[i]);
				}
				else
				{
					Smax[i] = 1E20;
					Smin[i] = -1E20;
				}
			}
			double smax = 1E21;
			double smin = -1E21;
			for (int i=0; i<_dof; i++)
			{
				if (Smax[i] < smax)
				{
					smax = Smax[i];
				}
				if (Smin[i] > smin)
				{
					smin = Smin[i];
				}
			}
			if (smin > smax || smax < 0 || smin > 1 + ALMOST_ZERO)
			{
				DUMP_ERROR("ERR: found singularity !!!\n");
				return ERR_PLAN_FAILED;
			}
			else
			{
				scales = MIN(smax-ACCURACY_FACTOR,1.0);
			}
		}
		else
		{
			break;
		}
	}

	return PLANNER_SUCCEED;
}


int PlannerOpt::move(int step, double nextPos[], double nextVel[], double nextJoint[], double nextJointVel[])
{
	if (_isNowInJointSpace)
	{
		double nextAcc[6]={0};
		move(step,nextJoint,nextJointVel,nextAcc);
		memcpy(nextPos,_lastTargetFlangePos,sizeof(double)*6);
		memcpy(nextVel,_lastTargetFlangeVel,sizeof(double)*6);
	}
	else
	{
		double nextAcc[6]={0};
		move(step,nextPos,nextVel,nextAcc);
		memcpy(nextJoint,_lastTracTargetJoint,sizeof(double)*_dof);
		memcpy(nextJointVel,_lastTargetJointVel,sizeof(double)*_dof);
	}

	return _plannerState;
}
