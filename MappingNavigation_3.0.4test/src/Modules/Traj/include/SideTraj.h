#pragma once

#include "Traj.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "TSideTraj".
class DllExport CSideTraj : public CTraj
{
private:
	CLine  m_ln;             // The line object   	

public:
	// The constructor
	CSideTraj() {}

	// Create the SIDE trajectory
	void CreateTraj(CPnt& ptStart, CPnt& ptEnd, CAngle& angHeading);

	// Vehicle's drive mode on the trajectory
	virtual SHORT GetDrivePattern() { return DRIVE_PATTERN_SHIFT; }

	// Get the start point of the trajectory
	virtual CPnt StartPnt() { return m_pt; }

	// Get the end point of the trajectory
	virtual CPnt EndPnt() { return m_pt; }

	// Get the total range of the progress variable
	virtual float GetRange() { return m_ln.Length(); }

	// Set the progress variable to specify the current point
	virtual void SetProgress(float fRate, float fCurLen);

	// The velocity generation function
	virtual CVelocity VelocityFun();

	// Get the deviation between the trajectory and the specified posture
	virtual bool ProjectPosture(CPosture& pst, CProjectData* pData) { return false; }
};
