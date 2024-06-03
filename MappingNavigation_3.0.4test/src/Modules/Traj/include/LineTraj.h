#pragma once

#include "Traj.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CLineTraj".
class DllExport CLineTraj : public CTraj
{
private:
	CLine m_ln;             // The line object

public:
	CLineTraj() {}

	// Trajectory creation function
	void CreateTraj(CPnt& ptStart, CPnt& ptEnd, CMoveDir WheelMoveDir,
		float fFrom = 0, float fTo = -1.0);

	// Get the start point of the trajectory
	virtual CPnt StartPnt() { return m_ln.m_ptStart; }

	// Get the end point of the trajectory
	virtual CPnt EndPnt() { return m_ln.m_ptEnd; }

	// Set the progress variable to specify the current point
	virtual void SetProgress(float fRate, float fCurLen);

	// The velocity generation function
	virtual CVelocity VelocityFun();

	// Get the deviation between the trajectory and the specified posture
	virtual bool ProjectPosture(CPosture& pst, CProjectData* pData);
};
