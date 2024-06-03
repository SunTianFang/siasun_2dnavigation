#pragma once

#include "Traj.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CScpTraj".
class DllExport CScpTraj : public CTraj
{
protected:
	CScp   m_Scp;           // The SCP curve object
	float  m_fLinearVel;    // The absolute value of linear velocity
	float  m_fK;            // The angle shifting ratio
	float  m_fCurX;         // The current X

protected:
	// Set the progress to specify the current absolute point
	void SetAbsProgress(float fRate, float fCurLen);

public:
	// The constructor
	CScpTraj() {}

	// Create the SCP trajectory.
	void CreateTraj(CPnt& ptStart, CPnt& ptEnd, CAngle& angHeading, float fFrom = 0,
		float fTo = -1.0f);

	// Vehicle's drive mode on the trajectory
	virtual SHORT GetDrivePattern() { return DRIVE_PATTERN_SHIFT; }

	// Get the start point of the trajectory
	virtual CPnt StartPnt() { return m_Scp.m_ptStart; }

	// Get the end point of the trajectory
	virtual CPnt EndPnt() { return m_Scp.m_ptEnd; }

	// Set the progress variable "Xe" to specify the current point
	virtual void SetProgress(float fRate, float fX);

	// The velocity generation function
	virtual CVelocity VelocityFun();

	// Get the deviation between the trajectory and the specified posture
	virtual bool ProjectPosture(CPosture& pst, CProjectData* pData);

	// Get the profile value for the given velocity vector
	virtual float GetProfileValue(float fLinearVel);

	// Get the profile slope for the given acceleration
	virtual float GetProfileSlope(float fAcc);

	// Get the linear according to the profile value
	virtual float GetLinearVel(float fValue);

	// The steer velocity generation function
	virtual CAngle SteeringFun();
};
