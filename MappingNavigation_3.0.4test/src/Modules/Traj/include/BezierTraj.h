#pragma once

#include "Traj.h"
#include "Bezier.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CBezierTraj".
class DllExport CBezierTraj : public CTraj
{
protected:
	CTurnDir m_TurnDir;         // Vehicle turn direction(COUNTER/CLOCKWISE)

	CAngle   m_angStartHeading; // Tangent angle at the start point
	CAngle   m_angEndHeading;   // Tangent angle at the end point
	CBezier*    m_pBezier;            // Pointer to the Bezier curve object

protected:
	// Set the progress to specify the current absolute point
	void SetAbsProgress(float fRate, float fCurLen);

	// Do basic initializations
	void Init(CMoveDir WheelMoveDir, CTurnDir TurnDir, float fFrom, float fTo);

	// Do initializations that are specific to the ARC trajectory
	virtual void InitEx(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CTurnDir TurnDir);

public:
	// Default constructor
	CBezierTraj() {}

	// Destructor
	virtual ~CBezierTraj();

	// Create the Bezier trajectory.
	void CreateTraj(CPnt& ptCenter, CPnt& ptStart, CPnt& ptEnd, CMoveDir WheelMoveDir,
		CTurnDir TurnDir, float fFrom = 0, float fTo = -1.0f);

	// Get the vehicle's start heading angle
	virtual CAngle& StartHeading() {return m_angStartHeading;}

	// Get the vehicle's end heading angle
	virtual CAngle& EndHeading() {return m_angEndHeading;}

	// Get the start point of the trajectory
	virtual CPnt StartPnt() { return m_pBezier->m_ptStart; }

	// Get the end point of the trajectory
	virtual CPnt EndPnt() { return m_pBezier->m_ptEnd; }

	// Set the progress variable to specify the current point
	virtual void SetProgress(float fRate, float fPhi);

	// The velocity generation function
	virtual CVelocity& VelocityFun();

	// The curvature at the start point
	virtual float StartCurvature() { return 1.0f / Radius(); }

	// The curvature at the end point
	virtual float EndCurvature() { return 1.0f / Radius(); }

	// Get the deviation between the trajectory and the specified posture
	virtual bool ProjectPosture(CPosture& pst, CProjectData* pData);

	// Get the profile value for the given velocity vector
	virtual float GetProfileValue(float fLinearVel) { return fLinearVel / Radius(); }

	// Get the profile slope for the given acceleration
	virtual float GetProfileSlope(float fAcc) { return fAcc / Radius(); }

	// Get the linear according to the profile value
	virtual float GetLinearVel(float fValue) { return fValue * Radius(); }

	// Get the radius of the arc to be replaced
	float Radius() { return m_pArc->Radius(); }

	// Get the center of the curve
	CPnt& Center() { return m_pArc->m_ptCenter; }

	// Get the turn direction
	CTurnDir TurnDir() { return m_TurnDir; }

	// Get the start angle
	CAngle StartAngle();

	// Get the end angle
	CAngle EndAngle();
};
