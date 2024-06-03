//                          - SCPTRAJ.CPP -
//
//   Implementation of class "CScpTraj", which defines Single Cartesian-
//   Polynomial curves. This kinds of curves serve as lane-change path
//   in this AGVS.
//
//   Author: Zhang Lei
//   Date:   2001. 5. 22
//

#include <stdafx.h>
#include <math.h>
#include "ScpTraj.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CScpTraj".

//
//   CScpTraj: The constructor.
//
void CScpTraj::CreateTraj(CPnt& ptStart, CPnt& ptEnd, CAngle& angHeading, 
								  float fFrom,	float fTo)
{
	m_nType = SCP_TRAJ;

	m_angHeading = angHeading;
	CLine ln(ptStart, ptEnd);
    CAngle angSlant = ln.SlantAngle();

	CAngle angLane = angHeading;
	CAngle ang90(90.0f, IN_DEGREE);

	m_angShift = angSlant - angHeading;
	m_fK = (float)fabs(cos(m_angShift));
   m_fCurX = fFrom;
	if (m_angShift > CAngle(90.0f, IN_DEGREE) &&
		 m_angShift < CAngle(270.0f, IN_DEGREE))
		angLane = !angLane;

	m_Scp = CScp(ptStart, ptEnd, angLane);

	float fMaxX = m_Scp.GetX();

	if (fFrom > fMaxX)
		fFrom = fMaxX;

	if (fTo < 0 || fTo > fMaxX)
		fTo = fMaxX;

	m_fFrom = fFrom;
	m_fTo = fTo;
	m_fRange = (float)fabs(fTo - fFrom);
	
	// Start/end steer angles are 0
	m_angSteer = CAngle(0.0f);

	switch (m_angShift.Quadrant())
	{
		case 1:
			m_WheelMoveDir = FORWARD;
			m_uBumperPattern = BUMPER_LE_FWD_CHK;
			m_uLightType = LIGHT_LEFT_ON;
			break;
					
		case 4:
			m_WheelMoveDir = FORWARD;
			m_uBumperPattern = BUMPER_RI_FWD_CHK;
			m_uLightType = LIGHT_RIGHT_ON;
			break;

		case 2:
			m_WheelMoveDir = BACKWARD;
			m_uBumperPattern = BUMPER_LE_BWD_CHK;
			m_uLightType = LIGHT_LEFT_ON;
			break;
			
		case 3:
			m_WheelMoveDir = BACKWARD;
			m_uBumperPattern = BUMPER_RI_BWD_CHK;
			m_uLightType = LIGHT_RIGHT_ON;
			break;
	}
}

//
//   SetAbsProgress: Specify the current point.
//
void CScpTraj::SetAbsProgress(float fRate, float fX)
{
	m_fProgRate = fRate;
	m_Scp.SetCurX(fX);
	m_fLinearVel = (float)fabs(m_fProgRate / cos(m_Scp.TangentFun(false)));
	m_fCurvature = m_Scp.CurvatureFun();
}

//
//   SetProgress: Specify the current point.
//
void CScpTraj::SetProgress(float fRate, float fX)
{
	SetAbsProgress(fRate, m_fFrom + fX);
}

//
//   SteeringFun: The steering angle generation function.
//
CAngle CScpTraj::SteeringFun()
{
	return m_Scp.TangentFun(false);
}

//
//   VelocityFun: The velocity generation function.
//
CVelocity CScpTraj::VelocityFun()
{
	m_vel.fLinear = (m_WheelMoveDir == FORWARD) ? m_fLinearVel : -m_fLinearVel;
	m_vel.fAngular = 0.0f;

	return m_vel;
}

//
//   Get the deviation between the trajectory and the specified posture.
//
bool CScpTraj::ProjectPosture(CPosture& pst, CProjectData* pData)
{
	float fRefX, fErrX, fErrY;

	if (m_Scp.FindRefX(pst.GetPntObject(), fRefX, fErrX))
	{
		pData->uType = 0;
		m_Scp.FindErrY(pst.GetPntObject(), fErrY);
		pData->fX = fErrX;
		pData->fY = fErrY;
		CAngle angErr = pst.GetAngle() - m_angHeading;
		pData->fThita = angErr.m_fRad;
		pData->fTangent = m_angHeading.m_fRad;
		pData->fProgress = fRefX;
		pData->fCurvature = 0;
		return true;
	}
	else
	{
		pData->uType = 1;
		pData->fX = 0;
		pData->fY = 0;
		pData->fThita = 0;
		pData->fTangent = 0;
		pData->fProgress = 0;
		pData->fCurvature = 0;
		return false;
	}
}

//
//   Get the profile value for the given velocity vector.
//
float CScpTraj::GetProfileValue(float fLinearVel)
{
	return fLinearVel * m_fK;
}

//
//   Get the profile slope for the given acceleration.
//
float CScpTraj::GetProfileSlope(float fAcc)
{
	return fAcc * m_fK;
}

//
//   Get the linear according to the profile value.
//
float CScpTraj::GetLinearVel(float fValue)
{
	return fValue / m_fK;
}
