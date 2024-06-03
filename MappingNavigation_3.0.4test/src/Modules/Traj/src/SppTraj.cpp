//                           - SPPTRAJ.CPP -
//
//   Implementation of class "CSppTraj", which depicts the trajectory of
//   Single Polar-Polynomial(SPP) curve.
//
//   Author: Zhang Lei
//   Date:   2001. 5. 22
//

#include <stdafx.h>
#include <assert.h>
#include "SppTraj.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CSppTraj", which depicts the trajectory of

//
//   Do initializations that are specific to the SPP trajectory.
//
void CSppTraj::InitEx(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CTurnDir TurnDir)
{
	m_nType = SPP_TRAJ;
	m_pArc = new CSpp(ptCenter, ptStart, ptEnd, TurnDir);
	assert(m_pArc != NULL);
}
