//                          - SPLNTRAJ.CPP -
//
//   Implementation of class "CSplineTraj", which depicts piecewise
//   polynomial curve or spline curve.
//
//   Author: Zhang Lei
//   Date:   2001. 5. 22
//

#include <stdafx.h>
#include <assert.h>
#include "SplineTraj.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CSplineTraj".

//
//   Do initializations that are specific to the SPLINE trajectory.
//
void CSplineTraj::InitEx(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CTurnDir TurnDir)
{
	m_nType = SPLINE_TRAJ;
	m_pArc = new CSpline(ptCenter, ptStart, ptEnd, TurnDir);
	assert(m_pArc != NULL);
}
