#pragma once

#include "ArcTraj.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CSplineTraj".
class DllExport CSplineTraj : public CArcTraj
{
protected:
	// Do initializations that are specific to the SPLINE trajectory
	void InitEx(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CTurnDir TurnDir);

public:
	// The curvature at the start point
	virtual float StartCurvature() { return 0.0f; }

	// The curvature at the end point
	virtual float EndCurvature() { return 0.0f; }
};
