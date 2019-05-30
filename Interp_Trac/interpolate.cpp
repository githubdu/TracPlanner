





#pragma once
#include "interpolate.h"




DoubleS::DoubleS()
{
	initiate();
}

DoubleS::~DoubleS()
{
}

void   DoubleS::initiate()
{
	_dir = 0;
	_velLimit = 0;
	_accLimit = 0;
	_jerkLimit = 0;
	_isValid = false;

	memset(_t, 0, sizeof(_t));
	memset(_x, 0, sizeof(_x));
	memset(_v, 0, sizeof(_v));
	memset(_a, 0, sizeof(_a));
	memset(_j, 0, sizeof(_j));
}

bool   DoubleS::isValid() const
{
	return _isValid;
}

double DoubleS::getMaxVel() const
{
	double v = 0;

	if (0 == _dir)
	{
		return 0.0;
	}

	for (int i=0; i<7; i++)
	{
		if (ABS(_v[i]) > ABS(v))
		{
			v = _v[i];
		}
	}

	return v*_dir;
}

double DoubleS::getMaxAcc() const
{
	double a = 0;

	if (0 == _dir)
	{
		return 0.0;
	}

	for (int i=0; i<7; i++)
	{
		if (ABS(_a[i]) > ABS(a))
		{
			a = _a[i];
		}
	}

	return a*_dir;
}

double DoubleS::getMaxJerk() const
{
	double j = 0;

	if (0 == _dir)
	{
		return 0.0;
	}

	for (int i=0; i<7; i++)
	{
		if (ABS(_j[i]) > ABS(j))
		{
			j = _j[i];
		}
	}

	return j*_dir;
}

double DoubleS::getDuration() const
{
	return (_isValid?(_t[7] - _t[0]):0);
}

double DoubleS::pos(double t) const
{
	if (0 == _dir)
	{
		return _x[0];
	}

	//--------------------
	double p0, pf, v0, vf;
	double Tj1, Tj2, Ta, Tv, Td, T;
	double v_max, a_max_a, a_max_d, j_max_a, j_min_d;

	p0 = _x[0];
	pf = _x[7];
	v0 = _v[0];
	vf = _v[7];

	v_max = _v[3];

	a_max_a = _a[1];
	a_max_d = _a[5];

	j_max_a = _j[1];
	j_min_d = -_j[7];

	Tj1 = _t[1] - _t[0];
	Tj2 = _t[7] - _t[6];
	Ta  = _t[3] - _t[0];
	Tv  = _t[4] - _t[3];
	T   = _t[7] - _t[0];
	Td  = T - Tv - Ta;

	//--------------------
	double dt = (t - _t[0]);
	if (dt <= 0)
	{
		dt = 0;
	}

	if (dt >= T)
	{
		dt = T;
	}

	//--------------------
	double p = 0;

	// Ta_acc
	if (dt >=0 && dt < Tj1)
	{
		p = p0 + v0*dt + j_max_a*dt*dt*dt / 6.0;
	}

	// Tv_acc
	if (dt >= Tj1 && dt < Ta-Tj1)
	{
		p = p0 + v0*dt + a_max_a*(3.0*dt*dt - 3.0*Tj1*dt + Tj1*Tj1) / 6.0;
	}

	// Td_acc
	if (dt >= Ta-Tj1 && dt < Ta)
	{
		p = p0 + (v_max + v0)*Ta / 2.0 - v_max*(Ta - dt) + j_max_a*(Ta - dt)*(Ta - dt)*(Ta - dt) / 6.0;
	}

	// Tv
	if (dt >= Ta && dt <= Ta+Tv)
	{
		p = p0 + (v_max + v0)*Ta / 2.0 + v_max*(dt - Ta);
	}

	// Ta_dec
	if (dt > T-Td && dt <= T-Td+Tj2)
	{
		p = pf - (v_max + vf)*Td / 2.0 + v_max*(dt - T + Td) + j_min_d*(dt - T + Td)*(dt - T + Td)*(dt - T + Td) / 6.0;
	}

	// Tv_dec
	if (dt > T-Td+Tj2 && dt <= T-Tj2)
	{
		p = pf - (v_max + vf)*Td / 2.0 + v_max*(dt - T + Td) + a_max_d / 6.0*(3.0*(dt - T + Td)*(dt - T + Td) - 3.0*Tj2*(dt - T + Td) + Tj2*Tj2);
	}

	// Td_dec
	if (dt > T-Tj2)
	{
		p = pf - vf*(T - dt) + j_min_d / 6.0*(T - dt)*(T - dt)*(T - dt);
	}

	p = p*_dir;

	return p;
}

double DoubleS::vel(double t) const
{
	if (0 == _dir)
	{
		return 0.0;
	}

	//--------------------
	double p0, pf, v0, vf;
	double Tj1, Tj2, Ta, Tv, Td, T;
	double v_max, a_max_a, a_max_d, j_max_a, j_min_d;

	p0 = _x[0];
	pf = _x[7];
	v0 = _v[0];
	vf = _v[7];

	v_max = _v[3];

	a_max_a = _a[1];
	a_max_d = _a[5];

	j_max_a = _j[1];
	j_min_d = -_j[7];

	Tj1 = _t[1] - _t[0];
	Tj2 = _t[7] - _t[6];
	Ta = _t[3] - _t[0];
	Tv = _t[4] - _t[3];
	T = _t[7] - _t[0];
	Td = T - Tv - Ta;

	//--------------------
	double dt = (t - _t[0]);
	if (dt <= 0)
	{
		dt = 0;
	}

	if (dt >= T)
	{
		dt = T;
	}

	//--------------------
	double v = 0.0;

	//Ta_acc
	if (dt < Tj1)
	{
		v = v0 + j_max_a*dt*dt / 2.0;
	}

	//Tv_acc
	if (dt >= Tj1 && dt < Ta-Tj1)
	{
		v = v0 + a_max_a*(dt - Tj1 / 2.0);
	}

	//Td_acc
	if (dt >= Ta-Tj1 && dt < Ta)
	{
		v = v_max - j_max_a*(Ta - dt)*(Ta - dt) / 2.0;
	}

	// Tv
	if (dt >= Ta && dt < Ta+Tv)
	{
		v = v_max;
	}

	// Ta_dec
	if (dt >= T-Td && dt < T-Td+Tj2)
	{
		v = v_max + j_min_d*(dt - T + Td)*(dt - T + Td) / 2.0;
	}

	//Tv_dec
	if (dt >= T-Td+Tj2 && dt < T-Tj2)
	{
		v = v_max + a_max_d*(dt - T + Td - Tj2 / 2.0);
	}

	// Td_dec
	if (dt >= T-Tj2)
	{
		v = vf - j_min_d*(T - dt)*(T - dt) / 2.0;
	}

	v = v*_dir;

	return v;
}

double DoubleS::acc(double t) const
{
	if (0 == _dir)
	{
		return 0.0;
	}

	//--------------------
	double p0, pf, v0, vf;
	double Tj1, Tj2, Ta, Tv, Td, T;
	double v_max, a_max_a, a_max_d, j_max_a, j_min_d;

	p0 = _x[0];
	pf = _x[7];
	v0 = _v[0];
	vf = _v[7];

	v_max = _v[3];

	a_max_a = _a[1];
	a_max_d = _a[5];

	j_max_a = _j[1];
	j_min_d = -_j[7];

	Tj1 = _t[1] - _t[0];
	Tj2 = _t[7] - _t[6];
	Ta  = _t[3] - _t[0];
	Tv  = _t[4] - _t[3];
	T   = _t[7] - _t[0];
	Td  = T - Tv - Ta;

	//--------------------
	double dt = (t - _t[0]);
	if (dt <= 0)
	{
		dt = 0;
	}

	if (dt >= T)
	{
		dt = T;
	}

	//--------------------
	double a = 0.0;

	// Ta_acc
	if (dt < Tj1)
	{
		a = j_max_a*dt;
	}

	//Tv_acc
	if (dt >= Tj1 && dt < Ta-Tj1)
	{
		a = a_max_a;
	}

	//Td_acc
	if (dt >= Ta-Tj1 && dt < Ta)
	{
		a = j_max_a*(Ta - dt);
	}

	//Tv
	if (dt >= Ta && dt < Ta+Tv)
	{
		a = 0;
	}

	//Ta_dec
	if (dt >= T-Td && dt < T-Td+Tj2)
	{
		a = j_min_d*(dt - T + Td);
	}

	//Tv_dec
	if (dt >= T-Td+Tj2 && dt < T-Tj2)
	{
		a = a_max_d;
	}

	//Td_dec
	if (dt >= T-Tj2)
	{
		a = j_min_d*(T - dt);
	}

	a = a*_dir;

	return a;
}

double DoubleS::jerk(double t) const
{
	if (0 == _dir)
	{
		return 0.0;
	}

	double j=0.0;

	double dt = (t - _t[0]);

	double Tj1, Tj2, Ta, Tv, Td, T, j_max_a, j_min_d;

	j_max_a = _j[1];
	j_min_d = -j_max_a;

	Tj1 = _t[1] - _t[0];
	Ta  = _t[3] - _t[0];
	Tv  = _t[4] - _t[3];
	T   = _t[7] - _t[0];
	Tj2 = _t[7] - _t[6];
	Td  = T - Tv - Ta;

	if (dt <= 0)
	{
		dt = 0;
	}

	if (dt >= T)
	{
		dt = T;
	}

	//Ta_acc
	if (dt < Tj1)
	{
		j = j_max_a;
	}

	//Tv_acc
	if (dt >= Tj1 && dt < Ta-Tj1)
	{
		j = 0.0;
	}

	//Td_acc
	if (dt >= Ta-Tj1 && dt < Ta)
	{
		j = j_min_d;
	}

	//Tv
	if (dt >= Ta && dt < Ta+Tv)
	{
		j = 0.0;
	}

	//Ta_dec
	if (dt >= T-Td && dt < T-Td+Tj2)
	{
		j = j_min_d;
	}

	//Tv_dec
	if (dt >= T-Td+Tj2 && dt < T-Tj2)
	{
		j = 0.0;
	}

	//Td_dec
	if (dt >= T-Tj2)
	{
		j = j_max_a;
	}

	j = j*_dir;

	return j;
}

prfTyp DoubleS::getProfileType() const
{
	return doubleS;
}

bool   DoubleS::scaleTo(double newDuration)
{
	if (0 == _dir)
	{
		return _isValid;
	}

	if (!_isValid)
	{
		DUMP_ERROR("ERR: the profile is invalid in <DoubleS::scaleTo>!\n");
		return _isValid;
	}

	if (newDuration <= (_t[7] - _t[0]))
	{
		DUMP_ERROR("ERR: new duration must be longer in <DoubleS::scaleTo>!\n");
		return false;
	}

	double t0 = _t[0];
	double p0 = _x[0];
	double pf = _x[7];
	double v0 = _v[0];
	double vf = _v[7];

	double Ta = _t[3] - _t[0];
	double Tv = _t[4] - _t[3];
	double Td = _t[7] - _t[4];
	double T  = _t[7] - _t[0];

	double Ta_acc = _t[1] - _t[0];
	double Tv_acc = _t[2] - _t[1];
	double Td_acc = _t[3] - _t[2];
	double Ta_dec = _t[5] - _t[4];
	double Tv_dec = _t[6] - _t[5];
	double Td_dec = _t[7] - _t[6];

	if (T == 0)
	{
		_t[7] = _t[0] + newDuration;
		return true;
	}

	Ta = Ta * newDuration/T;
	Tv = Tv * newDuration/T;
	Td = Td * newDuration/T;

	// decrease the cursing velocity, vmax
	double j_max = _j[1];
	double amax_a = _a[1];
	double j_min = abs(_j[5]);
	double amin_a = abs(_a[5]);
	double vmax = (pf - p0 - (Ta*v0) / 2.0 - (Td*vf) / 2.0) / (Ta / 2.0 + Td / 2.0 + Tv);

	// decrease jmax_a jmin_a jmax_d jmin_d and re-plan time interval
	double jmax_a,jmax_d,amin_d;
	if ((vmax - v0) * j_max < amax_a * amax_a)
	{
		Ta_acc = 0.5*Ta;
		jmax_a = (vmax - v0) / (Ta_acc*Ta_acc);
	}
	else
	{
		Ta_acc = Ta_acc * newDuration / T;
		amax_a = (vmax - v0) / (Ta - Ta_acc);
		Ta_acc = Ta - (vmax - v0) / amax_a;
		jmax_a = amax_a / Ta_acc;
	}

	if ((vmax - vf) * j_min < amin_a * amin_a)
	{
		Ta_dec = 0.5*Td;
		jmax_d = -(vmax - vf) / (Ta_dec*Ta_dec);
	}
	else
	{
		Ta_dec = Ta_dec * newDuration / T;
		amin_a = (vmax - vf) / (Td - Ta_dec);
		Ta_dec = Td - (vmax - vf) / amin_a;
		jmax_d = -amin_a / Ta_dec;
	}

	amax_a = jmax_a*Ta_acc;
	amin_d = jmax_d*Ta_dec;
	vmax = v0 + (Ta - Ta_acc)*amax_a;

	double tempTv = (pf - p0) / vmax - Ta / 2.0*(1.0 + v0 / vmax) - Td / 2.0*(1.0 + vf / vmax);
	if (ABS(tempTv - Tv) > ACCURACY_FACTOR)
	{
		DUMP_ERROR("ERR: scale to new duration failed in <DoubleS::scaleTo>!\n");
		return false;
	}

	T = Ta + Tv + Td;

	_j[1] = jmax_a;
	_j[3] = -jmax_a;
	_j[5] = jmax_d;
	_j[7] = -jmax_d;

	Td_acc = Ta_acc;
	Tv_acc = Ta - 2.0*Ta_acc;

	if (Tv_acc < 0)
	{
		DUMP_ERROR("ERR: scale to new duration failed in <DoubleS::scaleTo>!\n");
		return false;
	}

	Td_dec = Ta_dec;
	Tv_dec = Td - 2.0*Ta_dec;

	_t[0] = t0;
	_t[1] = _t[0] + Ta_acc;
	_t[2] = _t[1] + Tv_acc;
	_t[3] = _t[2] + Td_acc;
	_t[4] = _t[3] + Tv;
	_t[5] = _t[4] + Ta_dec;
	_t[6] = _t[5] + Tv_dec;
	_t[7] = _t[6] + Td_dec;

	_a[1] = amax_a;
	_a[2] = amax_a;
	_a[5] = amin_d;
	_a[6] = amin_d;

	_v[3] = vmax;
	_v[4] = vmax;

	_v[1] = vel(_t[1]);
	_v[2] = vel(_t[2]);
	_v[5] = vel(_t[5]);
	_v[6] = vel(_t[6]);

	_isValid = true;

	return _isValid;
}

void   DoubleS::setLimit(double v_limit, double a_limit, double j_limit)
{
	_velLimit = ABS(v_limit);
	_accLimit = ABS(a_limit);
	_jerkLimit = ABS(j_limit);
}

bool   DoubleS::planProfile(double t0, double p0, double pf, double v0, double vf)
{
	// check moving direction
	_dir = sign(pf - p0);

	if (ABS(pf-p0) < ALMOST_ZERO)
	{
		DUMP_WARNING("WARNING: target and start points are same in <DoubleS>!\n");

		if (abs(v0 - vf) > ALMOST_ZERO)
		{
			DUMP_ERROR("ERR: same point has two different non-zero velocities in <DoubleS>!\n");
			_isValid = false;
			return _isValid;
		}

		_dir = 0;
		_x[0] = p0;
		_v[0] = v0;
		_t[0] = t0;
		_t[7] = t0;
		_a[0] = 0.0;
		_j[0] = 0.0;
		_isValid = true;
		return _isValid;
	}

	// check limits
	if (ABS(_velLimit) == 0 || ABS(_accLimit) == 0 || ABS(_jerkLimit) == 0)
	{
		DUMP_ERROR("ERR: please set the _velLimit, _accLimit and _jerkLimit with non-zeros first in <DoubleS>!\n");
		_isValid = false;
		return _isValid;
	}

	bool rePlan = false; double originalVL = ABS(_velLimit);
	if (ABS(v0) > ABS(_velLimit) || ABS(vf) > ABS(_velLimit))
	{
		rePlan = true;
		_velLimit = MAX(ABS(v0),ABS(vf))*1.1; // 1.1 is for re-planning if failed
	}

	// re-calculate parameters
	p0 = _dir*p0;
	pf = _dir*pf;
	v0 = _dir*v0;
	vf = _dir*vf;

	double vmax = ABS(_velLimit);
	double amax = ABS(_accLimit);
	double jmax = ABS(_jerkLimit);
	double vmin = - vmax;
	double amin = - amax;
	double jmin = - jmax;

	double v_limit = (_dir+1.0)/2.0*vmax + (_dir-1.0)/2.0*vmin;
	double a_limit = (_dir+1.0)/2.0*amax + (_dir-1.0)/2.0*amin;
	double j_limit = (_dir+1.0)/2.0*jmax + (_dir-1.0)/2.0*jmin;

	// the 'doubleS' has acceleration phase, cursing phase and deceleration phase
	// in acceleration phase and deceleration phase, three phases are included: 
	//			acc acceleration phase, acc cursing phase and acc deceleration phase
	//	   a |	_________
	//		 | /         \   
	//       |/           \ 
	//       0--1- -----2--3-------4--5------6--7-----------> t
	//								\			/
	//								 \		   /
	//								  ---------

	_t[0] = t0;
	_x[0] = p0;	_x[7] = pf;
	_v[0] = v0; _v[7] = vf;
	_a[0] = 0;  _a[4] = 0;
	_a[3] = 0;  _a[7] = 0;
	_j[0] = 0;  _j[2] = 0; 
	_j[4] = 0;  _j[6] = 0;

	double r = 1; // scale factor for re-planning doubleS profile
	double Tjx = MIN(sqrt(ABS(vf - v0) / j_limit), a_limit / j_limit);
	if (Tjx == a_limit / j_limit)
	{
		if (pf - p0 <= 0.5*(v0 + vf)*(Tjx + ABS(vf - v0) / a_limit))
		{
			DUMP_ERROR("ERR: target and start points are t0o close! <DoubleS> is not feasible!\n");
			_isValid = false;
			return false;
		}
	}
	else
	{
		if (pf - p0 <= Tjx * (vf + v0))
		{
			DUMP_ERROR("ERR: target and start points are too close! <DoubleS> is not feasible!\n");
			_isValid = false;
			return false;
		}
	}

	double Ta_acc,Tv_acc,Td_acc,Ta,Tv,Ta_dec,Tv_dec,Td_dec,Td,T;
	double jmax_a, jmin_a, jmax_d, jmin_d,amax_a,amin_d;

	// Case 1: vmax = v_limit
	if ((v_limit - v0) * j_limit < a_limit * a_limit)
	{
		// a_limit is not reached
		Ta_acc = sqrt((v_limit - v0)/j_limit);
		Ta = 2*Ta_acc;
	}
	else
	{
		// a_limit is reached
		Ta_acc = a_limit / j_limit;
		Ta = Ta_acc + (v_limit - v0)/a_limit;
	}

	if ((v_limit - vf) * j_limit < a_limit * a_limit)
	{
		// a_limit is not reached
		Ta_dec = sqrt((v_limit - vf)/j_limit);
		Td = 2*Ta_dec;
	}
	else
	{
		// a_limit is reached
		Ta_dec = a_limit/j_limit;
		Td = Ta_dec + (v_limit - vf)/a_limit;
	}

	// determine the time duration of the cruising phase as
	Tv = (pf - p0) / v_limit - Ta / 2.0*(1.0 + v0 / v_limit) - Td / 2.0*(1.0 + vf / v_limit);

	if ( Tv <= 0)
	{
		double temp = 0;
		while(1)
		{
			temp = (a_limit*a_limit*a_limit*a_limit) / (j_limit*j_limit) + 2.0*(v0*v0 + vf*vf) + a_limit*(4.0*(pf - p0) - 2.0*a_limit / j_limit*(v0 + vf));
			Ta = ((a_limit*a_limit) / j_limit - 2.0*v0 + sqrt(temp)) / 2.0 / a_limit;
			Td = ((a_limit*a_limit) / j_limit - 2.0*vf + sqrt(temp)) / 2.0 / a_limit;
			Ta_acc = a_limit / j_limit;
			Ta_dec = a_limit / j_limit;
			Tv = 0;

			if ( Ta < 0 || Td <0)
			{
				if ( Ta < 0)
				{
					Td = 2.0 * (pf - p0) / (vf + v0);
					Ta_dec = (j_limit*(pf - p0) - sqrt(j_limit*(j_limit*(pf - p0)*(pf - p0) + (vf + v0)*(vf + v0)*(vf - v0)))) / (j_limit*(vf + v0));
					Ta_acc = 0.0;
					Ta = 0.0;
					break;
				}
				if ( Td < 0)
				{
					Ta = 2.0 * (pf - p0) / (vf + v0);
					Ta_acc = (j_limit*(pf - p0) - sqrt(j_limit*(j_limit*(pf - p0)*(pf - p0) - (vf + v0)*(vf + v0)*(vf - v0)))) / (j_limit*(vf + v0));
					Ta_dec = 0;
					Td = 0;
					break;
				}
			}
			else
			{
				if ((Ta<2.0*Ta_acc) || (Td<2.0*Ta_dec))
				{
					r = r - 0.01;
					a_limit = r*a_limit;
				}
				else
				{
					break;
				}
			}
		}
	}

	amax_a = j_limit*Ta_acc;
	amin_d = - j_limit*Ta_dec;
	vmax = v0 + (Ta - Ta_acc)*amax_a;

	jmax_a = j_limit;
	jmin_a = - j_limit;
	jmax_d = - j_limit;
	jmin_d = j_limit;

	Td_acc = Ta_acc;
	Tv_acc = Ta - 2.0*Ta_acc;

	Td_dec = Ta_dec;
	Tv_dec = Td - 2.0*Ta_dec;

	T = Ta + Tv + Td;

	_j[1] = jmax_a;
	_j[3] = jmin_a;
	_j[5] = jmax_d;
	_j[7] = jmin_d;

	_t[0] = t0;
	_t[1] = _t[0] + Ta_acc;
	_t[2] = _t[1] + Tv_acc;
	_t[3] = _t[2] + Td_acc;
	_t[4] = _t[3] + Tv;
	_t[5] = _t[4] + Ta_dec;
	_t[6] = _t[5] + Tv_dec;
	_t[7] = _t[6] + Td_dec;

	_a[1] = amax_a;
	_a[2] = amax_a;
	_a[5] = amin_d;
	_a[6] = amin_d;

	_v[3] = vmax;
	_v[4] = vmax;

	_v[1] = _dir * vel(_t[1]);
	_v[2] = _dir * vel(_t[2]);
	_v[5] = _dir * vel(_t[5]);
	_v[6] = _dir * vel(_t[6]);

	if (rePlan)
	{
		double p0 = _x[0];
		double pf = _x[7];
		double v0 = _v[0];
		double vf = _v[7];

		double T  = _t[7] - _t[0];
		double Ta = _t[3] - _t[0];
		double Tv = _t[4] - _t[3];
		double Td = _t[7] - _t[4];

		double newV = MIN(ABS(originalVL),ABS(vmax));
		double alpha = 2.0*(pf-p0)/(Ta*newV + Td*newV + 2.0*Tv*newV + Ta*v0 + Td*vf);

		double vmax = (pf - p0 - (alpha*Ta*v0)/2.0 - (alpha*Td*vf)/2.0)/(alpha*(Ta/2.0 + Td/2.0 + Tv));

		_isValid = true;
		_isValid = scaleTo(alpha*T);
		_velLimit = originalVL;
		return _isValid;
	}
	else
	{
		_isValid = true;
		return _isValid;
	}
}





Trapezoid::Trapezoid()
{
	initiate();
}

Trapezoid::~Trapezoid()
{
}

void   Trapezoid::initiate()
{
	_dir = 0;
	_velLimit = 0;
	_accLimit = 0;
	_jerkLimit = 0;
	_isValid = false;

	memset(_t, 0, sizeof(_t));
	memset(_x, 0, sizeof(_x));
	memset(_v, 0, sizeof(_v));
	memset(_a, 0, sizeof(_a));
}

bool   Trapezoid::isValid() const
{
	return _isValid;
}

double Trapezoid::getMaxVel() const
{
	double v = 0;

	if (0 == _dir)
	{
		return 0.0;
	}

	for (int i = 0; i < 4; i++)
	{
		if (ABS(_v[i]) > ABS(v))
		{
			v = _v[i];
		}
	}

	return v*_dir;
}

double Trapezoid::getMaxAcc() const
{
	double a = 0;

	if (0 == _dir)
	{
		return 0.0;
	}

	for (int i = 0; i < 4; i++)
	{
		if (ABS(_a[i]) > ABS(a))
		{
			a = _a[i];
		}
	}

	return a*_dir;
}

double Trapezoid::getMaxJerk() const
{
	DUMP_WARNING("WARNING: jerk is invalid in <Trapezoid>!\n");
	return 0; // jerk is not valid in Trapezoid profile
}

double Trapezoid::getDuration() const
{
	return (_isValid ? (_t[3] - _t[0]) : 0);
}

double Trapezoid::pos(double t) const
{
	if (0 == _dir)
	{
		return _x[0];
	}

	double p;
	double dt = (t - _t[0]);
	double tT = _t[3] - _t[0];
	double ta = _t[1] - _t[0];
	double tv = _t[2] - _t[1];
	double td = _t[3] - _t[2];

	// check time t
	if (dt <=0)
	{
		dt = 0;
	}

	if (dt>=tT)
	{
		dt = tT;
	}

	// piecewise, phase #0, not started yet
	if (dt <= 0)
	{
		dt = 0;
		p = _x[0];
	}

	// phase #1, accelerating
	if ( dt < ta && dt >= 0)
	{
		p = _x[0] + _v[0] * dt + (_v[1] - _v[0]) / 2.0 / ta*dt*dt;
	}

	// phase #2, cruising
	if (ta <= dt && dt < ta + tv)
	{
		p = _x[0] + _v[0] * ta / 2.0 + _v[1] * (dt - ta / 2.0);
	}

	// phase #3, decelerating
	if (ta + tv <= dt && dt < tT)
	{
		p = _x[3] - _v[3] * (tT - dt) - (_v[2] - _v[3]) / 2.0 / td*(tT - dt)*(tT - dt);
	}

	// phase $, stopped
	if (dt >= tT)
	{
		dt = tT;
		p = _x[3] - _v[3] * (tT - dt) - (_v[2] - _v[3]) / 2.0 / td*(tT - dt)*(tT - dt);
	}

	p = p*_dir;

	return p;
}

double Trapezoid::vel(double t) const
{
	if (0 == _dir)
	{
		return _v[0];
	}

	double v;
	double dt = (t - _t[0]);
	double tT = _t[3] - _t[0];
	double ta = _t[1] - _t[0];
	double tv = _t[2] - _t[1];
	double td = _t[3] - _t[2];

	// check time t
	if (dt <= 0)
	{
		dt = 0;
	}

	if (dt >= tT)
	{
		dt = tT;
	}

	// piecewise, phase #0, not started yet
	if (dt <= 0)
	{
		dt = 0;
		v = _v[0];
	}

	// phase #1, accelerating
	if ( dt < ta && dt >= 0)
	{
		v = _v[0] + (_v[1] - _v[0]) / ta*dt;
	}

	// phase #2, cruising
	if (ta <= dt && dt < ta + tv)
	{
		v = _v[1];
	}

	// phase #3, decelerating
	if (ta + tv <= dt && dt < tT)
	{
		v = _v[3] + (_v[2] - _v[3]) / td*(tT - dt);
	}

	// phase #4, stopped
	if (dt >= tT)
	{
		dt = tT;
		v = _v[3] + (_v[2] - _v[3]) / td*(tT - dt);
	}

	v = v*_dir;

	return v;
}

double Trapezoid::acc(double t) const
{
	if (0 == _dir)
	{
		return _a[0];
	}

	double a;
	double dt = (t - _t[0]);
	double tT = _t[3] - _t[0];
	double ta = _t[1] - _t[0];
	double tv = _t[2] - _t[1];
	double td = _t[3] - _t[2];

	// check time t
	if (dt <=0)
	{
		dt = 0;
	}

	if (dt>=tT)
	{
		dt = tT;
	}

	// piecewise, phase #0, not started yet
	if (dt <= 0)
	{
		dt = 0;
		a = 0.0;
	}

	// phase #1, accelerating
	if ( dt < ta && dt >= 0)
	{
		a = _a[1];
	}

	// phase #2, cruising
	if (ta <= dt && dt < ta + tv)
	{
		a = _a[2];
	}

	// phase #3, decelerating
	if (ta + tv <= dt && dt < tT)
	{
		a = _a[3];
	}

	// phase #4, stopped
	if (dt >= tT)
	{
		dt = tT;
		a = 0.0;
	}

	a = a*_dir;
	return a;
}

double Trapezoid::jerk(double t) const
{
	DUMP_WARNING("WARNING: jerk is invalid in <Trapezoid>!\n");
	return 0; // jerk is not valid in trapezoid profile
}

prfTyp Trapezoid::getProfileType() const
{
	return trapezoid;
}

bool   Trapezoid::scaleTo(double newDuration)
{
	if (0 == _dir)
	{
		return _isValid;
	}

	if (!_isValid)
	{
		DUMP_ERROR("ERR: the profile is not valid in <Trapezoid::scaleTo>!\n");
		return false;
	}

	if (newDuration <= getDuration())
	{
		DUMP_ERROR("ERR: new duration must be longer in <Trapezoid::scaleTo>!\n");
		return false;
	}

	double p0 = _x[0];
	double pf = _x[3];
	double v0 = _v[0];
	double vf = _v[3];
	double h = pf - p0;

	double T  = _t[3] - _t[0];
	double Ta = _t[1] - _t[0];
	double Tv = _t[2] - _t[1];
	double Td = _t[3] - _t[2];

	// solve these 4 equations to get new _v[1] Ta Td and Tv

	// Ta + Tv + Td = newT
	// Ta = (_v[1] - v0)/_a[1];
	// Td = (_v[3] - _v[1])/_a[3];
	// Tv = ((pf-p0) - (_v[1]*Ta - 0.5*_a[1]*Ta*Ta) - (_v[1]*Td + 0.5*_a[3]*Td*Td))/_v[1];

	while(true)
	{
		double a1 = _a[1], a3 = _a[3], newT = newDuration,tempT[3] = {0};

		if (sign(a1) == sign(a3))
		{
			Tv = (v0 - 1.0*vf + a1*newT) / a1;
			Td = (-1.0*v0*v0 + 2.0*v0*vf - 1.0*vf*vf + 2.0*a1*newT*vf + 2.0*a1*p0 - 2.0*a1*pf) / (2.0*a1*v0 - 2.0*a1*vf + 2.0*a1*a1*newT);
			Ta = -(0.5*(v0*v0 - 2.0*v0*vf + 2.0*a1*newT*v0 + vf*vf + 2.0*a1*p0 - 2.0*a1*pf)) / (a1*(v0 - 1.0*vf + a1*newT));
			_v[1] = -(0.5*(-1.0*v0*v0 + vf*vf + 2.0*a1*p0 - 2.0*a1*pf)) / (v0 - 1.0*vf + a1*newT);
			_v[2] = _v[1];
		}
		else
		{
			double temp = sqrt(4.0*a1*p0 - 4.0*a1*pf + 2.0*v0*vf - 1.0*v0*v0 - 1.0*vf*vf + a1*a1*newT*newT + 2.0*a1*newT*v0 + 2.0*a1*newT*vf);
			Tv = (v0 - 1.0*vf + a1*newT) / a1 - (1.0*(v0 - 1.0*vf + a1*newT + temp)) / a1;
			if (Tv < 0 || Tv > newT)
			{
				temp = -temp;
				Tv = (v0 - 1.0*vf + a1*newT) / a1 - (1.0*(v0 - 1.0*vf + a1*newT + temp)) / a1;
			}

			Td = (0.5*(v0 - 1.0*vf + a1*newT + temp)) / a1;
			Ta = (0.5*vf - 0.5*v0 + 0.5*a1*newT + 0.5*temp) / a1;
			_v[1] = 0.5*v0 + 0.5*vf + 0.5*a1*newT + 0.5*temp;
			_v[2] = _v[1];
		}

		if (Ta < 0)
		{
			_a[1] = -_a[1];
		}
		if (Td < 0)
		{
			_a[3] = -_a[3];
		}

		if (Ta >= 0 && Td >= 0)
		{
			break;
		}
	}

	_t[1] = _t[0] + Ta;
	_t[2] = _t[1] + Tv;
	_t[3] = _t[2] + Td;

	_x[1]  = _x[0] + _v[0]*Ta/2.0 + _v[1]*(Ta - Ta/2.0);
	_x[2]  = _x[0] + _v[0]*Ta/2.0 + _v[1]*(Tv - Ta/2.0);

	_isValid = true;
	return _isValid;
}

void   Trapezoid::setLimit(double v_limit, double a_limit, double j_limit)
{
	_velLimit = ABS(v_limit);
	_accLimit = ABS(a_limit);
	_jerkLimit = ABS(j_limit);
}

bool   Trapezoid::planProfile(double t0, double p0, double pf, double v0, double vf)
{
	// check moving direction
	_dir = sign(pf - p0);

	if (ABS(pf - p0) < ALMOST_ZERO)
	{
		DUMP_WARNING("WARNING: target and start points are same in <Trapezoid>!\n");

		if (abs(v0 - vf) > ALMOST_ZERO)
		{
			DUMP_ERROR("ERR: same point has two different non-zero velocities in <Trapezoid>!\n");
			_isValid = false;
			return _isValid;
		}

		_dir = 0;
		_x[0] = p0;
		_v[0] = v0;
		_t[0] = t0;
		_t[3] = t0;
		_a[0] = 0.0;

		_isValid = true;
		return _isValid;
	}

	// check limits
	if (ABS(_velLimit) == 0 || ABS(_accLimit) == 0)
	{
		DUMP_ERROR("ERR: please set the v_limit and a_limit with non-zeros first in <Trapezoid>!\n");
		_isValid = false;
		return _isValid;
	}

	// re-calculate parameters
	p0 = _dir*p0;
	pf = _dir*pf;
	v0 = _dir*v0;
	vf = _dir*vf;

	double vmax = ABS(_velLimit);
	double amax = ABS(_accLimit);
	double vmin = - vmax;
	double amin = - amax;

	double v_limit = (_dir+1.0)/2.0*vmax + (_dir-1.0)/2.0*vmin;
	double a_limit = (_dir+1.0)/2.0*amax + (_dir-1.0)/2.0*amin;

	// the 'trapezoid' profile trajectory looks like :
	//			
	//	   v |	_________
	//		 | /         \   
	//     v0|/           \ 
	//       |             \ vf
	//	     |              |
	//       0--1-------2---3--> t
	// let Ta Tv Td be the accelerating/cursing and decelerating time intervals
	// so, pf - p0 = v0*Ta +0.5*_a[1]*Ta*Ta + _v[1]*Tv + vf*Td - 0.5*_a[3]*Td*Td
	// Ta = (_v[1] - _v[0])/_a[1];	// here, _a[1],acceleration, bigger than zero
	// Td = (_v[3] - _v[2])/_a[3];	// here, _a[3],deceleration, samller than zero,
	// Tv = ((pf-p0) - (_v[1]*Ta - 0.5*_a[1]*Ta*Ta) - (_v[1]*Td + 0.5*_a[3]*Td*Td))/_v[1];

	_t[0] = t0;
	_x[0] = p0;		_x[3] = pf;
	_v[0] = v0;		_v[3] = vf;
	_a[0] = 0;		_a[2] = 0; 

	amax = a_limit;		// a_limit: acceleration in phase #1;
	_a[1] = a_limit;	// plan the trajectory with the limited acceleration

	amin = - a_limit;	//amin: acceleration in phase #3, be deceleration.
	_a[3] = amin;		// by default, deceleration equals to minus acceleration

	// it is necessary to check whether the trajectory is feasible or not by
	if (a_limit*(pf - p0) < ABS(v0*v0-vf*vf)/2.0)
	{
		DUMP_ERROR("ERR: trajectory is not feasible in <Trapezoid>!\n");
		_isValid = false;
		return _isValid;
	}
	else
	{
		double Ta,Tv,Td;
		_a[1] = sign(_velLimit - v0)*ABS(_a[1]);
		_a[3] = sign(vf - _velLimit)*ABS(_a[3]);

		if ((pf - p0) >= (v_limit*v_limit-v0*v0)/2.0/_a[1] + (-v_limit*v_limit+vf*vf)/2.0/_a[3])
		{
			// Case 1: _v[1] = v_limit
			_v[1] = v_limit; 
		}
		else
		{
			// Case 2: _v[1] < v_limit
			_v[1] = sqrt( (pf-p0 + v0*v0/(2.0*_a[1]) - vf*vf/(2.0*_a[3]))/(0.5/_a[1] - 0.5/_a[3]));
		}

		_v[2] = _v[1];

		Ta = ((_v[1] - _v[0])/_a[1]);
		Td = ((_v[3] - _v[2])/_a[3]);
		Tv = ((pf-p0) - (_v[1]*Ta - 0.5*_a[1]*Ta*Ta) - (_v[1]*Td + 0.5*_a[3]*Td*Td))/_v[1];

		_t[1] = _t[0] + Ta;
		_t[2] = _t[1] + Tv;
		_t[3] = _t[2] + Td;

		_x[1]  = _x[0] + _v[0]*Ta/2.0 + _v[1]*(Ta - Ta/2.0);
		_x[2]  = _x[0] + _v[0]*Ta/2.0 + _v[1]*(Tv - Ta/2.0);
	}

	_isValid = true;
	return _isValid;
}




DS_or_Trap::DS_or_Trap()
{
	initiate();
}

DS_or_Trap::~DS_or_Trap()
{
}

void   DS_or_Trap::initiate()
{
	_curPrf = doubleS;
	_doubleS.initiate();
	_trapezoid.initiate();
}

bool   DS_or_Trap::isValid() const
{
	switch(_curPrf)
	{
	case doubleS:
		return _doubleS.isValid();
		break;

	case trapezoid:
		return _trapezoid.isValid();
		break;
	}

	return _doubleS.isValid();
}

double DS_or_Trap::getMaxVel() const
{
	switch(_curPrf)
	{
	case doubleS:
		return _doubleS.getMaxVel();
		break;

	case trapezoid:
		return _trapezoid.getMaxVel();
		break;
	}

	return _doubleS.getMaxVel();
}

double DS_or_Trap::getMaxAcc() const
{
	switch(_curPrf)
	{
	case doubleS:
		return _doubleS.getMaxAcc();
		break;

	case trapezoid:
		return _trapezoid.getMaxAcc();
		break;
	}

	return _doubleS.getMaxAcc();
}

double DS_or_Trap::getMaxJerk() const
{
	switch(_curPrf)
	{
	case doubleS:
		return _doubleS.getMaxJerk();
		break;

	case trapezoid:
		return _trapezoid.getMaxJerk();
		break;
	}

	return _doubleS.getMaxJerk();
}

double DS_or_Trap::getDuration() const
{
	switch(_curPrf)
	{
	case doubleS:
		return _doubleS.getDuration();
		break;

	case trapezoid:
		return _trapezoid.getDuration();
		break;
	}

	return _doubleS.getDuration();
}

double DS_or_Trap::pos(double t) const
{
	switch(_curPrf)
	{
	case doubleS:
		return _doubleS.pos(t);
		break;

	case trapezoid:
		return _trapezoid.pos(t);
		break;
	}

	return _doubleS.pos(t);
}

double DS_or_Trap::vel(double t) const
{
	switch(_curPrf)
	{
	case doubleS:
		return _doubleS.vel(t);
		break;

	case trapezoid:
		return _trapezoid.vel(t);
		break;
	}

	return _doubleS.vel(t);
}

double DS_or_Trap::acc(double t) const
{
	switch(_curPrf)
	{
	case doubleS:
		return _doubleS.acc(t);
		break;

	case trapezoid:
		return _trapezoid.acc(t);
		break;
	}

	return _doubleS.acc(t);
}

double DS_or_Trap::jerk(double t) const
{
	switch(_curPrf)
	{
	case doubleS:
		return _doubleS.jerk(t);
		break;

	case trapezoid:
		return _trapezoid.jerk(t);
		break;
	}

	return _doubleS.jerk(t);
}

prfTyp DS_or_Trap::getProfileType() const
{
	return _curPrf;
}

bool   DS_or_Trap::scaleTo(double newDuration)
{
	bool ret = true;
	switch(_curPrf)
	{
	case doubleS:
		ret = _doubleS.scaleTo(newDuration);
		break;

	case trapezoid:
		_trapezoid.scaleTo(newDuration);
		break;
	}

	return ret;
}

void   DS_or_Trap::setLimit(double vl, double al, double jl)
{
	_doubleS.setLimit(vl,al,jl);
	_trapezoid.setLimit(vl,al,jl);
}

bool   DS_or_Trap::planProfile(double t0, double p0, double pf, double v0, double vf)
{
	_curPrf = doubleS;

	if (!_doubleS.planProfile(t0,p0,pf,v0,vf))
	{
		_curPrf = trapezoid;
		return _trapezoid.planProfile(t0,p0,pf,v0,vf);
	}
	else
	{
		return _doubleS.isValid();
	}
}









Multi_ST_1D::Multi_ST_1D()
{
	initiate();
};

Multi_ST_1D::~Multi_ST_1D()
{

}

void   Multi_ST_1D::initiate()
{
	_pNum = 0;
	_v0 = 0.0;
	_vf = 0.0;

	_rePlanN = 0;
	_rePlanT = 0;

	_fistMax = true;
	_isValid = false;
	_timeChanged = false;
}

int    Multi_ST_1D::_computeMax()
{
	_vMax = 0;
	_aMax = 0;
	_jMax = 0;
	_fistMax = false;

	for (int i=0; i<_pNum; i++)
	{
		if(abs(_dst[i].getMaxVel()) > _vMax)
		{
			_vMax = abs(_dst[i].getMaxVel());
		}

		if(abs(_dst[i].getMaxAcc()) > _aMax)
		{
			_aMax = abs(_dst[i].getMaxAcc());
		}

		if(abs(_dst[i].getMaxJerk()) > _jMax)
		{
			_jMax = abs(_dst[i].getMaxJerk());
		}
	}

	return 0;
}

bool   Multi_ST_1D::_computeTimeT()
{
	if (_pNum > 2)
	{
		_timeChanged = true;
	}

	double T = _t[_pNum-1] - _t[MAX(_rePlanN-1,0)];

	if ( T < 0)
	{
		return false;
	}

	double d = 0.0;
	for (int i=MAX(_rePlanN-1,0); i<_pNum-1; i++)
	{
		d += ABS(_p[i+1]-_p[i]);
	}
	if (d < 0)
	{
		return false;
	}

	if (T > 0 && d > 0)
	{
		for (int i=MAX(_rePlanN,1); i<_pNum; i++)
		{
			_t[i] = ABS(_p[i]-_p[i-1])/d*T + _t[i-1];
		}
	}
	else
	{
		for (int i=MAX(_rePlanN,1); i<_pNum; i++)
		{
			_t[i] = _t[0];
		}
	}

	return true;
}

bool   Multi_ST_1D::isValid() const
{
	return _isValid;
}

double Multi_ST_1D::getMaxVel() const
{
	return _vMax;
}

double Multi_ST_1D::getMaxAcc() const
{
	return _aMax;
}

double Multi_ST_1D::getMaxJerk() const
{
	return _jMax;
}

bool   Multi_ST_1D::_computeVelocity()
{
	for (int i=1+_rePlanN; i<_pNum-1; i++)
	{
		if ((_p[i] - _p[i-1])*(_p[i+1] - _p[i]) < 0)
		{
			_v[i] = 0.0;
		}
		else if (ABS(_p[i] - _p[i-1]) < ALMOST_ZERO || ABS(_p[i+1] - _p[i]) < ALMOST_ZERO)
		{
			_v[i] = 0.0;
		}
		else
		{
			if (abs((_p[i] - _p[i-1])/(_t[i] - _t[i-1])) > abs(_vLimit))
			{
				_timeChanged = true;
				double deltaT = abs(_p[i] - _p[i-1])/abs(_vLimit) - _t[i] + _t[i-1];
				for (int j=i; j<_pNum; j++)
				{
					_t[j] = _t[j] + deltaT;
				}
			}

			if (abs((_p[i+1] - _p[i])/(_t[i+1] - _t[i])) > abs(_vLimit))
			{
				_timeChanged = true;
				double deltaT = abs((_p[i+1] - _p[i]))/abs(_vLimit) - _t[i+1] + _t[i];
				for (int j=i+1; j<_pNum; j++)
				{
					_t[j] = _t[j] + deltaT;
				}
			}

			_v[i] = (_p[i] - _p[i-1])/(_t[i] - _t[i-1])*0.5 + (_p[i+1] - _p[i])/(_t[i+1] - _t[i])*0.5;
		}
	}

	_v[0] = _v0;
	_v[_pNum-1] = _vf;

	return true;
}

double Multi_ST_1D::getDuration() const
{
	if (!_isValid)
	{
		return 0.0;
	}

	return _t[_pNum-1] - _t[0];
}

double Multi_ST_1D::pos(double t) const
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

	return _dst[index].pos(t);
}

double Multi_ST_1D::vel(double t) const
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
	return _dst[index].vel(t);
};

double Multi_ST_1D::acc(double t) const 
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
	return _dst[index].acc(t);
}

double Multi_ST_1D::jerk(double t) const
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
	return _dst[index].jerk(t);
}

bool   Multi_ST_1D::setStartVel(double v0)
{
	_v0 = v0;
	return true;
}

bool   Multi_ST_1D::_computeAcceleration()
{
	for (int i=1+_rePlanN; i<_pNum-1; i++)
	{
		if ((_v[i] - _v[i-1])*(_v[i+1] - _v[i]) < 0)
		{
		}
		else
		{
			if (abs((_v[i] - _v[i-1])/(_t[i] - _t[i-1])) > abs(_aLimit))
			{
				_timeChanged = true;
				double deltaT = abs(_v[i] - _v[i-1])/abs(_aLimit) - _t[i] + _t[i-1];
				for (int j=i; j<_pNum; j++)
				{
					_t[j] = _t[j] + deltaT;
				}
			}

			if (abs((_v[i+1] - _v[i])/(_t[i+1] - _t[i])) > abs(_aLimit))
			{
				_timeChanged = true;
				double deltaT = abs(_v[i+1] - _v[i]) / abs(_aLimit) - _t[i+1] + _t[i];
				for (int j=i+1; j<_pNum; j++)
				{
					_t[j] += deltaT;
				}
			}
		}
	}
	return true;
}

bool   Multi_ST_1D::setTargetVel(double vf)
{
	_vf = vf;
	return true;
}

bool   Multi_ST_1D::isTimeChanged() const
{
	return _timeChanged;
}

prfTyp Multi_ST_1D::getProfileType() const
{
	return multi_ST_1D;
}

int    Multi_ST_1D::_iSpan(double t) const
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

bool   Multi_ST_1D::_checkAndRecomputeTimeT()
{
	// planning
	_isValid = true;
	for (int i=0; i<_pNum-1; i++)
	{
		_dst[i].setLimit(_vLimit,_aLimit,_jLimit);
		while (true)
		{
			_isValid = _dst[i].planProfile(_t[i],_p[i],_p[i+1],_v[i],_v[i+1]);
			if (_isValid)
			{
				if (_t[i+1] - (_t[i] + _dst[i].getDuration()) > PLAN_CYCLE/1000.0)
				{
					_isValid = _dst[i].scaleTo(_t[i+1] - _t[i]);
					break;
				}
				else
				{
					double deltaT = _dst[i].getDuration() - (_t[i+1] - _t[i]);
					if (deltaT <= ALMOST_ZERO)
					{
						break;
					}

					// re-plan time t
					_timeChanged = true;
					for (int j = i+1; j < _pNum; j++)
					{
						_t[j] = _t[j] + deltaT;
					}
					// no necessary to re-plan velocities and accelerations
				}
			}
			else
			{
				return _isValid;
			}
		}

		if (!_isValid)
		{
			return _isValid;
		}
	}

	if (_isValid)
	{
		_computeMax();
	}

	return _isValid;
}

bool   Multi_ST_1D::scaleTo(double newDuration)
{
	if (!_isValid)
	{
		DUMP_ERROR("ERR: the spline profile is invalid in <Multi_ST::scaleToDuration>!\n");
		return _isValid;
	}

	if (newDuration <= getDuration())
	{
		DUMP_ERROR("ERR: new duration must be longer in <Multi_ST::scaleToDuration>!\n");
	}

	// re-plan time t
	_t[_pNum-1] = _t[0] + newDuration;
	if (!_computeTimeT())
	{
		_isValid = false;
		return _isValid;
	}

	// re-plan velocity
	if (!_computeVelocity())
	{
		_isValid = false;
		return _isValid;
	}

	// re-plan acceleration
	if (!_computeAcceleration())
	{
		_isValid = false;
		return _isValid;
	}

	// re-planning
	for (int i=MAX(_rePlanN-1,0); i<_pNum-1; i++)
	{
		if (i == _rePlanN-1)
		{
			_dst[i].scaleTo(_t[i+1] - _t[i] - (_rePlanT-_t[i]));
		}
		else
		{
			_dst[i].scaleTo(_t[i+1]-_t[i]);
		}
	}

	return _isValid;
}

bool   Multi_ST_1D::_checkTimeT(double* t) const
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

void   Multi_ST_1D::setLimit(double vl, double al, double jl)
{
	_vLimit = ABS(vl);
	_aLimit = ABS(al);
	_jLimit = ABS(jl);
}

bool   Multi_ST_1D::getTimeSequence(double* t, int num) const
{
	if (num > _pNum)
	{
		DUMP_WARNING("WARNING: only %d points exist in <SPLINE::getTimeSequence>!\n");
	}
	num = MIN(num, _pNum);
	memcpy(t, &_t[0], sizeof(double)*num);
	return true;
}

bool   Multi_ST_1D::planProfile(double* t, int num, double* p, double* v)
{
	// clear flags
	_fistMax = true;
	_isValid = false;
	_timeChanged = false;

	// save
	_pNum = num;
	memset(_p,0,sizeof(_p));
	memcpy(_p,p,sizeof(double)*num);

	// check t
	if (!_checkTimeT(t))
	{
		_isValid = false;
		return _isValid;
	}

	// save time sequence
	memset(_t,0,sizeof(_t));
	memcpy(_t,t,sizeof(double)*num);

	// check velocity
	if (nullptr != v)
	{
		memset(_v, 0, sizeof(_v));
		memcpy(_v, v, sizeof(double)*_pNum);
	}
	else
	{
		if (!_computeVelocity())
		{
			_isValid = false;
			return _isValid;
		}
	}

	// compute acceleration
	if (!_computeAcceleration())
	{
		_isValid = false;
		return _isValid;
	}

	// planning
	_rePlanN = 0;
	return _checkAndRecomputeTimeT();
}

bool   Multi_ST_1D::rePlanProfile(double t, double vl, double al, double jl)
{

	if (ABS(vl) <=0 || ABS(al) <= 0 || ABS(jl) <= 0)
	{
		DUMP_ERROR("ERR: new limits are invalid for being zeros in <Multi_ST::rePlanProfile>!\n");
		return false;
	}

	if (  ABS(ABS(vl) - _vLimit) < ALMOST_ZERO 
		&&ABS(ABS(al) - _aLimit) < ALMOST_ZERO
		&&ABS(ABS(jl) - _jLimit) < ALMOST_ZERO)
	{
		return true;
	}

	// backup time sequence
	double ot[POINTS_MAX_NUM] = {0};
	this->getTimeSequence(ot,_pNum);

	// re-plan profile on line
	int index = _iSpan(t);
	double p0 = _dst[index].pos(t);
	double v0 = _dst[index].vel(t);
	_dst[index].setLimit(vl,al,jl);
	if (_dst[index].planProfile(t,p0,_p[index+1],v0,_v[index+1]))
	{
		bool ret = true; // re-plan the reset profile
		_t[index+1] = t	 + _dst[index].getDuration();
		for (int i=index+1; i<_pNum-1; i++)
		{
			_dst[i].setLimit(vl,al,jl);
			if (!_dst[i].planProfile(_t[i],_p[i],_p[i+1],_v[i],_v[i+1]))
			{
				_rePlanN = 0;
				ret = false;
				break;
			}
			_t[i+1] = _t[i] + _dst[i].getDuration();
		}

		// failed, reset everything
		if (!ret)
		{
			memcpy(_t,ot,sizeof(double)*_pNum);
			_checkAndRecomputeTimeT();
			_rePlanN = 0;
			return false;
		}
		else // succeed, update
		{
			setLimit(vl,al,jl);
			_rePlanN = index+1;
			_rePlanT = t;
			return true;
		}
	}
	else // failed, reset everything
	{
		memcpy(_t,ot,sizeof(double)*_pNum);
		_checkAndRecomputeTimeT();
		_rePlanN = 0;
		return false;
	}
}

bool   Multi_ST_1D::move(double t, double* pos, double* vel, double* acc) const
{
	*pos = this->pos(t);
	*vel = this->vel(t);
	*acc = this->acc(t);

	return true;
}

bool   Multi_ST_1D::planProfile(double t0, double tf, int num, double* p, double* v)
{
	// clear flags
	_isValid = false;
	_fistMax = true;
	_timeChanged = false;

	// save
	_pNum = num;
	memset(_p,0,sizeof(_p));
	memcpy(_p,p,sizeof(double)*num);

	// compute time t
	_t[0] = t0; _t[num-1] = tf;
	if (!_computeTimeT())
	{
		_isValid = false;
		return _isValid;
	}

	// check velocity
	if (nullptr != v)
	{
		memset(_v, 0, sizeof(_v));
		memcpy(_v, v, sizeof(double)*_pNum);
	}
	else
	{
		if (!_computeVelocity())
		{
			_isValid = false;
			return _isValid;
		}
	}

	// compute acceleration
	if (!_computeAcceleration())
	{
		_isValid = false;
		return _isValid;
	}

	// planning 
	_rePlanN = 0;
	return _checkAndRecomputeTimeT();
}
