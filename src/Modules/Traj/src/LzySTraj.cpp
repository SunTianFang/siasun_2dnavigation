//                           - LAZYTRAJ.CPP -
//
//   Implementation of class "CLazySTraj", which defines a "lazy-S" like
//   curves. This kinds of curves serve as lane-change path in this AGVS.
//
//   Author: Zhang Lei
//   Date:   2001. 5. 21
//

#include <stdafx.h>
#include <math.h>
#include "LazySTraj.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CLazySTraj".

//
//   CLazySTraj: The constructor.
//
void CLazySTraj::CreateTraj(CPnt& ptStart, CPnt& ptEnd, CAngle& angHeading,
									 float fFrom, float fTo)
{
	CScpTraj::CreateTraj(ptStart, ptEnd, angHeading, fFrom, fTo);
	m_nType = LAZY_S_TRAJ;
}

//
//   Set the progress variable to specify the current point.
//
void CLazySTraj::SetProgress(float fRate, float fProgress)
{
	CScpTraj::SetProgress(fRate, fProgress);
   m_fCurX = fProgress;
	m_pt = m_Scp.TrajFun();
	m_angHeading = m_Scp.TangentFun(true);
	m_fCurvature = m_Scp.CurvatureFun();
}

//
//   VelocityFun: The velocity generation function.
//
CVelocity CLazySTraj::VelocityFun()
{
	m_vel.fLinear = (m_WheelMoveDir == FORWARD) ? m_fLinearVel : -m_fLinearVel;
	m_vel.fAngular = m_fLinearVel * m_Scp.CurvatureFun();

	return m_vel;
}

//
//   Get the deviation between the trajectory and the specified posture.
//
bool CLazySTraj::ProjectPosture(CPosture& pst, CProjectData* pData)
{
	bool bResult = true;

   CPnt& pt = pst.GetPntObject();
   CPnt ptRef = TrajFun();
	CAngle angRef = HeadingFun();
	CAngle angDiff = pst.GetAngle() - angRef;	 			
	pData->fThita = angDiff.m_fRad;

	pData->uType = 0;
	pData->fY = 0;
	pData->fProgress = m_fCurX;
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

/*	if (WheelMoveDir() == BACKWARD)
      pData->fCurvature = -pData->fCurvature;
*/
	return bResult;
}
