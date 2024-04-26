//                         - SPINTRAJ.CPP -
//
//   Implementation of class "CSpinTraj", which defins the trajectory of
//   spin turn.
//
//   Author: Zhang Lei
//   Date:   2001. 5. 21
//

#include <stdafx.h>
#include <math.h>
#include <float.h>
#include "SpinTraj.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CSpinTraj".

//
//   CSpinTraj: The constructor.
//
void CSpinTraj::CreateTraj(CPnt& pt, CAngle& angStart, CAngle& angEnd,
									CTurnDir TurnDir, float fFrom, float fTo)
{
	m_nType = SPIN_TRAJ;
	m_TurnDir = TurnDir;
	m_angStartHeading = m_angHeading = angStart;
	m_angEndHeading = angEnd;
	m_angSteer = CAngle(0.0f);

	m_uBumperPattern = BUMPER_ALL_CHK;
	
	CAngle angTurn = angEnd - angStart;
	if (TurnDir == CLOCKWISE)
		angTurn = -angTurn;

	m_fTurnAngle = angTurn.m_fRad;

	if (fFrom > m_fTurnAngle)
		fFrom = m_fTurnAngle;

	if (fTo < 0 || fTo > m_fTurnAngle)
		fTo = m_fTurnAngle;

	m_fFrom = fFrom;
	m_fTo = fTo;
	m_fRange = (float)fabs(fTo - fFrom);
	
	// Assign trajectory point
	m_pt = pt;

	if (TurnDir == COUNTER_CLOCKWISE)
		m_fCurvature = FLT_MAX;
	else
		m_fCurvature = -FLT_MAX;
}

//
//   SetProgress: Set the progress variable to specify the current point.
//
void CSpinTraj::SetProgress(float fRate, float fPhi)
{
	m_fProgRate = fRate;
	CAngle ang = (m_TurnDir == COUNTER_CLOCKWISE) ?
					  CAngle(fPhi) : CAngle(-fPhi);

	m_angHeading = m_angStartHeading + ang;
}

//
//   VelocityFun: The velocity generation function.
//
CVelocity CSpinTraj::VelocityFun()
{
	m_vel.fLinear = 0.0f;
	m_vel.fAngular = (m_TurnDir == COUNTER_CLOCKWISE) ?
							m_fProgRate : -m_fProgRate;

	return m_vel;
}
