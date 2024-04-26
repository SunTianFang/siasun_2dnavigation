#pragma once

#include "Traj.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CSteerTraj".
class DllExport CSteerTraj : public CTraj
{
protected:
	CAngle m_angStartSteer;
	CAngle m_angEndSteer;

public:
	// Trajectory creation function
	void CreateTraj(CPnt& pt, CAngle& angHeading, float fFromSteerAngle,
		float fToSteerAngle);

	// Vehicle's drive mode on the trajectory
	virtual SHORT GetDrivePattern() { return DRIVE_PATTERN_SHIFT; }

	// Get the vehicle's start steer angle
	virtual CAngle StartSteerAngle() { return m_angStartSteer; }

	// Get the vehicle's end steer angle
	virtual CAngle EndSteerAngle() { return m_angEndSteer; }

	// Set the progress variable to specify the current point
	virtual void SetProgress(float fRate, float fProgress) {}

	// Get the deviation between the trajectory and the specified posture
	virtual bool ProjectPosture(CPosture& pst, CProjectData* pData) { return false; }

	// Get the profile value for the given velocity vector
	virtual float GetProfileValue(float fLinearVel) { return 0.0f; }

	// Get the profile slope for the given acceleration
	virtual float GetProfileSlope(float fAcc) { return 0.0f; }

	// Get the linear according to the profile value
	virtual float GetLinearVel(float fValue) { return 0; }
};
