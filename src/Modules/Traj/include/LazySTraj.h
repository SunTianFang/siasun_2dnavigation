#pragma once

#include "ScpTraj.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CLazySTraj".
class DllExport CLazySTraj : public CScpTraj
{
public:
	// The constructor
	CLazySTraj() {}

	// Trajectory creation function
	void CreateTraj(CPnt& ptStart, CPnt& ptEnd, CAngle& angHeading, float fFrom = 0,
		float fTo = -1.0);

	// Vehicle's drive mode on the trajectory
	virtual SHORT GetDrivePattern() { return DRIVE_PATTERN_TURN; }

	// Set the progress variable to specify the current point
	virtual void SetProgress(float fRate, float fProgress);

	// The heading generation function
	virtual CAngle HeadingFun() { return m_Scp.TangentFun(); }

	// The velocity generation function
	virtual CVelocity VelocityFun();

	// Get the deviation between the trajectory and the specified posture
	virtual bool ProjectPosture(CPosture& pst, CProjectData* pData);

	// The steer velocity generation function
	virtual CAngle SteeringFun() { return CTraj::SteeringFun(); }
};
