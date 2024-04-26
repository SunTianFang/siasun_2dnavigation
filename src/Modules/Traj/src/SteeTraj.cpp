//                          - STEETRAJ.CPP -
//
//   Implementation of class "CSteerTraj".
//
//   Author: Zhang Lei
//   Date:   2003. 2. 27
//

#include <stdafx.h>
#include <math.h>
#include "SteerTraj.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
//   Implementation of the class.

//
//   Trajectory creation function.
//
void CSteerTraj::CreateTraj(CPnt& pt, CAngle& angHeading, float fFromSteerAngle,
									 float fToSteerAngle)
{
	m_nType = STEER_TRAJ;
	m_pt = pt;
	m_angHeading = angHeading;
	m_WheelMoveDir = FORWARD;      // Meaningless !
	m_uBumperPattern = 0;
	m_uLightType = 0;
	m_fFrom = fFromSteerAngle;
	m_fTo = fToSteerAngle;
	m_fRange = (float)fabs(fToSteerAngle-fFromSteerAngle);
	m_angStartSteer = CAngle(fFromSteerAngle);
	m_angEndSteer = CAngle(fToSteerAngle);
	m_angSteer = CAngle(m_fFrom);
	m_vel.fLinear = 0;
	m_vel.fAngular = 0;
}
