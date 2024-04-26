#pragma once

#include "ArcPath.h"
#include "SplineTraj.h"

//////////////////////////////////////////////////////////////////////////////
//   Class "CSplinePath": A class derived from base class "CPath", defining
//   SPLINE-type paths.
//
class DllExport CSplinePath : public CArcPath
{
public:
	// Make a trajectory from the path
	virtual CTraj* MakeTraj()
	{
		CSplineTraj* pSplineTraj = new CSplineTraj;
		CPnt& ptStart = GetStartPnt();
		CPnt& ptEnd = GetEndPnt();
		pSplineTraj->CreateTraj(m_ptCenter, ptStart, ptEnd, FORWARD, m_TurnDir);
		return pSplineTraj;
	}
};

