#pragma once

#include "ScpPath.h"
#include "LazySTraj.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CLazySPath".
class DllExport CLazySPath : public CScpPath
{
public:
	// Make a trajectory from the path
	virtual CTraj* MakeTraj()
	{
		CLazySTraj* pLazySTraj = new CLazySTraj;
		CPnt& ptStart = GetStartPnt();
		CPnt& ptEnd = GetEndPnt();
		pLazySTraj->CreateTraj(ptStart, ptEnd, m_angHeading);
		return pLazySTraj;
	}
};
