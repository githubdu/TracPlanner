


#pragma once

#ifndef _TYPESANDERRORS_H_
#define _TYPESANDERRORS_H_

#include <stdlib.h>
#include <iostream>



#define NO_DUMP_WARNING

#ifndef DUMP_ERROR
#ifndef NO_DUMP_ERROR
#define DUMP_ERROR		printf
#else
#define DUMP_ERROR
#endif
#endif	// errors

#ifndef NO_DUMP_INFORM
#define DUMP_INFORM		printf
#else
#define DUMP_INFORM
#endif	// informs

#ifndef NO_DUMP_WARNING
#define DUMP_WARNING	printf
#else
#define DUMP_WARNING	
#endif  // warning



#define PI (3.1415926535897932380)


#define ALMOST_ZERO		(1E-15)
#define ACCURACY_FACTOR	(1E-05)

#define	ROT_INDEX		(5)

#define	PLAN_CYCLE		(2)	

#define ZONE_RATIO		(0)
#define ZONE_LENGTH		(1)
#define MIN_ZONE_SIZE	(1E-2)

#define MAX_DOF			(10)
#define POINTS_MAX_NUM	(128)

#define ERR_REP_NOTYET	(100)
#define ERR_REP_FAILED	(101)
#define ERR_PLAN_FAILED	(200)


enum trcTyp
{
	tracUnset		= 0,
	tracSleep		= 1,
	tracPtp			= 2,
	tracLine		= 3,
	tracCircle		= 4,
	tracRotate		= 5,
	tracBspline		= 6,
	tracBlender		= 7,
	tracTurning		= 8
};



#endif