//                          - SIDETRAJ.CPP -
//
//   Implementation of class "CSideTraj" - which defines the geometric
//   concept "Side-moving Straight Line".
//
//   Author: Zhang Lei                              
//   Date:   2001. 5. 21
//

#include <stdafx.h>
#include <math.h>
#include "SideTraj.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
//   Implementation of the class.

//
//   CSideTraj: The constructor.
//
void CSideTraj::CreateTraj(CPnt& ptStart, CPnt& ptEnd, CAngle& angHeading)
{
	m_nType = SIDE_TRAJ;

	m_ln = CLine(ptStart, ptEnd);

	m_angHeading = angHeading;
	m_angSteer = m_ln.SlantAngle() - m_angHeading;
	m_angShift = m_angSteer;

	if (m_angSteer <= CAngle(90.0f, IN_DEGREE))
	{
		m_WheelMoveDir = FORWARD;
		m_uBumperPattern = BUMPER_LE_FWD_CHK;
	}
	else if (m_angSteer <= CAngle(180.0f, IN_DEGREE))
	{
		m_WheelMoveDir = BACKWARD;
		m_angSteer = !m_angSteer;
		m_uBumperPattern = BUMPER_LE_BWD_CHK;
	}
	else if (m_angSteer <= CAngle(270.0f, IN_DEGREE))
	{
		m_WheelMoveDir = BACKWARD;
		m_angSteer = !m_angSteer;
		m_uBumperPattern = BUMPER_RI_BWD_CHK;
	}
	else
	{
		m_WheelMoveDir = FORWARD;
		m_uBumperPattern = BUMPER_RI_FWD_CHK;
	}
}

//
//   SetProgress: Set the progress variable to specify the current point.
//
void CSideTraj::SetProgress(float fRate, float fCurLen)
{
	m_fProgRate = fRate;
	m_pt = m_ln.TrajFun(fCurLen);
}

//
//   VelocityFun: The vehicle's velocity generation function.
//
CVelocity CSideTraj::VelocityFun()
{
	m_vel.fLinear = (m_WheelMoveDir == FORWARD) ? m_fProgRate : -m_fProgRate;
	m_vel.fAngular = 0.0f;

	return m_vel;
}
