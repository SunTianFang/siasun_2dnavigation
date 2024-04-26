#pragma once

#include "Traj.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "TSpinTraj".
class DllExport CSpinTraj : public CTraj
{
private:
	CTurnDir m_TurnDir;          // Turning direction
	CAngle   m_angStartHeading;  // Vehicle's start heading
	CAngle   m_angEndHeading;    // Vehicle's end heading
	float    m_fTurnAngle;       // The turn angle of the trajectory
	float    m_fCurvature;       // The curvature

public:
	// The constructor
	CSpinTraj() {}

	// Trajectory creation function
	void CreateTraj(CPnt& pt, CAngle& angStart, CAngle& angEnd, CTurnDir TurnDir,
		float fFrom = 0, float fTo = -1.0f);

	// Get the vehicle's start heading angle
	virtual CAngle StartHeading() { return m_angStartHeading; }

	// Get the vehicle's end heading angle
	virtual CAngle EndHeading() { return m_angEndHeading; }

	// Get the start point of the trajectory
	virtual CPnt StartPnt() { return m_pt; }

	// Get the end point of the trajectory
	virtual CPnt EndPnt() { return m_pt; }

	// Get the total range of the progress variable
	virtual float GetRange() { return m_fTurnAngle; }

	// Set the progress variable to specify the current point
	virtual void SetProgress(float fRate, float fPhi);

	// The velocity generation function
	virtual CVelocity VelocityFun();

	// The curvature at the start point
	virtual float StartCurvature() { return m_fCurvature; }

	// The curvature at the end point
	virtual float EndCurvature() { return m_fCurvature; }

public:
	// Get the deviation between the trajectory and the specified posture
	virtual bool ProjectPosture(CPosture& pst, CProjectData* pData) { return false; }

	// Get the profile value for the given velocity vector
	virtual float GetProfileValue(float fAngularVel) { return fAngularVel; }

	// Get the profile slope for the given acceleration
	virtual float GetProfileSlope(float fAcc) { return fAcc; }

	CTurnDir TurnDir() { return m_TurnDir; }
};
