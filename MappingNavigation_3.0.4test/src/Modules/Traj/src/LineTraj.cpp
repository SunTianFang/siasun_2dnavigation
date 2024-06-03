//                          - LINETRAJ.CPP -
//
//   Implementation of class "CLineTraj" - which defines the geometric
//   concept "Directional Straight Line".
//
//   Author: Zhang Lei
//   Date:   2001. 7. 13
//

#include <stdafx.h>
#include <math.h>
#include "LineTraj.h"

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
void CLineTraj::CreateTraj(CPnt& ptStart, CPnt& ptEnd, CMoveDir WheelMoveDir,
									float fFrom, float fTo)
{
	m_nType = LINE_TRAJ;
	m_WheelMoveDir = WheelMoveDir;
	m_uBumperPattern = (USHORT)((WheelMoveDir == FORWARD) ? BUMPER_FRONT_TYPE : BUMPER_REAR_TYPE);

	// Construct a line object
	m_ln = CLine(ptStart, ptEnd);
	float fLen = m_ln.Length();

	if (fFrom > fLen)
		fFrom = fLen;

	if (fTo < 0 || fTo > fLen)
		fTo = fLen;

	m_fFrom = fFrom;
	m_fTo = fTo;
	m_fRange = (float)fabs(fTo-fFrom);

	m_angSteer = CAngle(0.0f);
	m_angHeading = m_ln.SlantAngle();

	if (m_WheelMoveDir == BACKWARD)
		m_angHeading = !m_angHeading;
}

//
//   SetProgress: Set the progress variable to specify the current point.
//
void CLineTraj::SetProgress(float fRate, float fCurLen)
{
	if (fCurLen > m_fRange)
		fCurLen = m_fRange;

	m_fProgRate = fRate;
	m_pt = m_ln.TrajFun(m_fFrom + fCurLen);
}

//
//   VelocityFun: The vehicle's velocity generation function.
//
CVelocity CLineTraj::VelocityFun()
{
	m_vel.fLinear = (m_WheelMoveDir == FORWARD) ? m_fProgRate : -m_fProgRate;
	m_vel.fAngular = 0.0f;

	return m_vel;
}

//
//   Get the deviation between the trajectory and the specified posture.
//
bool CLineTraj::ProjectPosture(CPosture& pst, CProjectData* pData)
{
	CAngle ang1, ang2;
	float fLen1;
    bool bReturn;

	CAngle angDiff = pst.GetAngle() - m_angHeading;

	pData->fY = 0;
	pData->fCurvature = 0;
	pData->fThita = angDiff.m_fRad;
	pData->fTangent = m_angHeading.m_fRad;

	CPnt& pt0 = pst.GetPntObject();

	if (m_ln.m_ptStart != pt0)
	{
		CLine ln1(m_ln.m_ptStart, pt0);
		fLen1 = ln1.Length();
		ang1 = ln1.SlantAngle() - m_ln.SlantAngle();

		if (fLen1 < 0.03f)     // The distance is smaller than 3cm
		{
			pData->uType = 0;
			pData->fX = 0;
			pData->fProgress = 0;
            return true;
		}
	}
	else
	{
/*
		fLen1 = 0;
		ang1 = CAngle(0.0f);
*/
		pData->uType = 0;
		pData->fX = 0;
		pData->fProgress = 0;
        return true;
	}

	if (m_ln.m_ptEnd != pt0)
	{
		CLine ln2(m_ln.m_ptEnd, pt0);
		ang2 = !(m_ln.SlantAngle()) - ln2.SlantAngle();

		if (ln2.Length() < 0.03f)     // The distance is smaller than 3cm
		{
			pData->uType = 0;
			pData->fX = 0;
			pData->fProgress = fLen1 - m_fFrom;
            return true;
		}
	}
	else
	{
//		ang2 = CAngle(PI);

		pData->uType = 0;
		pData->fX = 0;
		pData->fProgress = fLen1 - m_fFrom;
        return true;
	}

	// Decide the projection type
	if (ang1.Degree() > 90.0f && ang1.Degree() < 270.0f)
	{
		pData->uType = 1;
		pData->fX = -fLen1 * sin(ang1);
		pData->fProgress = 0;
        bReturn = false;
	}
	else if (ang2.Degree() > 90.0f && ang2.Degree() < 270.0f)
	{
		pData->uType = 2;
		pData->fX = -fLen1 * sin(ang1);
		pData->fProgress = m_fRange;
        bReturn = false;
	}
	else
	{
		pData->uType = 0;
		pData->fX = -fLen1 * sin(ang1);
		pData->fProgress = fLen1 * cos(ang1) - m_fFrom;
        bReturn = true;
	}

	if (WheelMoveDir() == BACKWARD)
		pData->fX = -pData->fX;

	return bReturn;
}
