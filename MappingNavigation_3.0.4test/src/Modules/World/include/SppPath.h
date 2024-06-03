#pragma once

#include "ArcPath.h"
#include "SppTraj.h"

//////////////////////////////////////////////////////////////////////////////
//   Class "CSppPath": A class derived from base class "CPath", defining
//   SPP-type paths.
//
class DllExport CSppPath : public CArcPath
{
public:
	// Make a trajectory from the path
	virtual CTraj* MakeTraj()
	{
		CSppTraj* pSppTraj = new CSppTraj;
		CPnt& ptStart = GetStartPnt();
		CPnt& ptEnd = GetEndPnt();
		pSppTraj->CreateTraj(m_ptCenter, ptStart, ptEnd, FORWARD, m_TurnDir);
		return pSppTraj;
	}
};
