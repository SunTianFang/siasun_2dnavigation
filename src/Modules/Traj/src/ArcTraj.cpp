//                           - ARCTRAJ.CPP -
//
//   Implementation of class "CArcTraj", which depicts the trajectory of
//   Single Polar-Polynomial(ARC) curve.
//
//   Author: Zhang Lei
//   Date:   2001. 7. 13
//

#include <stdafx.h>
#include <math.h>                       
#include <assert.h>
#include "ArcTraj.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CArcTraj".

CArcTraj::~CArcTraj()
{
	if (m_pArc != NULL)
	{
		delete m_pArc;
		m_pArc = NULL;
	}
}

//
//   Do basic initializations.
//
void CArcTraj::Init(CMoveDir WheelMoveDir, CTurnDir TurnDir, float fFrom, float fTo)
{
	m_WheelMoveDir = WheelMoveDir;
	m_TurnDir = TurnDir;
	m_angSteer = CAngle(0.0f);
	
	if (WheelMoveDir == FORWARD)
	{
		m_uBumperPattern = (USHORT)((TurnDir == COUNTER_CLOCKWISE) ? BUMPER_LE_FWD_CHK : BUMPER_RI_FWD_CHK);
		m_uLightType = (TurnDir == COUNTER_CLOCKWISE) ? LIGHT_LEFT_ON : LIGHT_RIGHT_ON;
	}
	else
	{
		m_uBumperPattern = (USHORT)((TurnDir == COUNTER_CLOCKWISE) ? BUMPER_RI_BWD_CHK : BUMPER_LE_BWD_CHK);
		m_uLightType = (TurnDir == COUNTER_CLOCKWISE) ? LIGHT_RIGHT_ON : LIGHT_LEFT_ON;
	}

	float fTurnAng = m_pArc->TurnAngle();

	if (fFrom > fTurnAng)
		fFrom = fTurnAng;

	if (fTo < 0 || fTo > fTurnAng)
		fTo = fTurnAng;

	m_fFrom = fFrom;
	m_fTo = fTo;
	m_fRange = (float)fabs(fTo - fFrom);

	SetAbsProgress(0.0f, m_fFrom);
	m_angStartHeading = m_angHeading;

	// Caculate the end heading
	SetAbsProgress(0.0f, m_fTo);
	m_angEndHeading = m_angHeading;

	m_angHeading = m_angStartHeading;
}

//
//   Do initializations that are specific to the ARC trajectory.
//
void CArcTraj::InitEx(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CTurnDir TurnDir)
{
	m_nType = ARC_TRAJ;
	m_pArc = new CArc(ptCenter, ptStart, ptEnd, TurnDir);
	assert(m_pArc != NULL);
}

//
//   Create the ARC trajectory.
//
void CArcTraj::CreateTraj(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd,
								  CMoveDir WheelMoveDir, CTurnDir TurnDir, float fFrom, 
								  float fTo)
{
	InitEx(ptCenter, ptStart, ptEnd, TurnDir);
	Init(WheelMoveDir, TurnDir, fFrom, fTo);
}

//
//   SetAbsProgress: Specify the current point on the curve.
//
void CArcTraj::SetAbsProgress(float fRate, float fPhi)
{
	m_fProgRate = fRate;
	m_pArc->SetCurAngle(fPhi);

	m_pt = m_pArc->TrajFun();
	m_angHeading = m_pArc->TangentFun();
	m_fCurvature = m_pArc->CurvatureFun();
	if (m_WheelMoveDir == BACKWARD)
		m_angHeading = !m_angHeading;
}

//
//   SetProgress: Specify the current point on the curve.
//
void CArcTraj::SetProgress(float fRate, float fPhi)
{
	if (fPhi > m_fRange)
		fPhi = m_fRange;

	SetAbsProgress(fRate, m_fFrom + fPhi);
}

//
//   VelocityFun: The vehicle velocity generation function. It will
//   override the virtual function in the base class "CTraj".
//
CVelocity CArcTraj::VelocityFun()
{
	float fLinear = m_fProgRate * m_pArc->CurRadius();
	m_vel.fAngular = fLinear * m_pArc->CurvatureFun();
	m_vel.fLinear = (m_WheelMoveDir == FORWARD) ? fLinear: -fLinear;

	// Return the vehicle's velocity vector
	return m_vel;
}

//
//   Get the deviation between the trajectory and the specified posture.
//
bool CArcTraj::ProjectPosture(CPosture& pst, CProjectData* pData)
{
	CAngle ang0, angTemp1, angTemp2;
    bool bResult = true;

	CPnt& pt = pst.GetPntObject();
	CLine ln0(m_pArc->m_ptCenter, pt);
	CAngle angStart = StartAngle();
	CAngle angEnd = EndAngle();

	if (m_TurnDir == CLOCKWISE)
	{ 		
		angTemp1 = ln0.SlantAngle() - angEnd;
		angTemp2 = angStart - angEnd;		   				
		if (angTemp1.m_fRad <= angTemp2.m_fRad)
			ang0 = angStart - ln0.SlantAngle();
		else
		{
			if ((angTemp1-angStart) > (angEnd-angTemp1))
				ang0 = angTemp2;
			else
				ang0 = 0.0f;
            bResult = false;
		}
	}
	else                                  // Counter-clockwise
	{   		
		angTemp1 = ln0.SlantAngle() - angStart;
		angTemp2 = angEnd - angStart;
		if (angTemp1.m_fRad <= angTemp2.m_fRad)
			ang0 = angTemp1;
		else
		{
			if ((angTemp1-angEnd) > (angStart-angTemp1))
				ang0 = 0.0f;
			else
				ang0 = angTemp2;
            bResult = false;
		}
	}   		

	SetProgress(0.0f, ang0.m_fRad);

	CPnt ptRef = TrajFun();
	CAngle angRef = HeadingFun();
	CAngle angDiff = pst.GetAngle() - angRef;	 			
	pData->fThita = angDiff.m_fRad;			

	pData->uType = 0;
	pData->fY = 0;
	pData->fProgress = ang0.m_fRad;
	pData->fTangent = angRef.m_fRad;
	pData->fCurvature = CurvatureFun();

	// Construct a line - from the reference point to the posture point
	if (ptRef != pt)
	{
		CLine ln1(ptRef, pt);
		pData->fX = ln1.Length();
		CAngle angTemp = ln1.SlantAngle() - angRef;
		if (angTemp.Degree() < 180.0f)
			pData->fX = -pData->fX;
	}
	else
		pData->fX = 0;

	return bResult;
}

//
//   应返回实际轨迹的起点角（当轨迹为顺时针时应返回终点角）.
//
CAngle CArcTraj::StartAngle()
{
	if (m_TurnDir == CLOCKWISE)
		return CAngle(m_pArc->m_angStart.m_fRad + m_pArc->m_fTurnAngle - m_fFrom);
	else
		return CAngle(m_pArc->m_angStart.m_fRad + m_fFrom);
}

//
//   应返回实际轨迹的终点角（当轨迹为顺时针时应返回起点角）.
//
CAngle CArcTraj::EndAngle()
{
	if (m_TurnDir == CLOCKWISE)
		return CAngle(m_pArc->m_angStart.m_fRad + m_pArc->m_fTurnAngle - m_fTo);
	else
		return CAngle (m_pArc->m_angStart.m_fRad + m_fTo);
}	
