//                          - TRAJ.H -
//
//   The interface of class "CTraj", which serves as the base class for
//   other geometric trajectories. It is an abstract class.
//
//   Author: Zhang Lei
//   Date:   2001. 9. 7
//

#pragma once

#include "Geometry.h"
#include "Tools.h"
#include <math.h>

#define MAX_BUMPER_NUM           16

// Bumper type bits definition
#define BUMPER_FRONT_TYPE        BIT(0)
#define BUMPER_REAR_TYPE			BIT(1)
#define BUMPER_LEFT_TYPE			BIT(2)
#define BUMPER_RIGHT_TYPE			BIT(3)
#define BUMPER_ALL_TYPE          (BIT(0)|BIT(1)|BIT(2)|BIT(3))

#define BUMPER_FWD_CHK           BUMPER_FRONT_TYPE
#define BUMPER_BWD_CHK           BUMPER_REAR_TYPE
#define BUMPER_LE_CHK            BUMPER_LEFT_TYPE
#define BUMPER_RI_CHK            BUMPER_RIGHT_TYPE
#define BUMPER_LE_FWD_CHK        (BUMPER_LEFT_TYPE | BUMPER_FRONT_TYPE)
#define BUMPER_LE_BWD_CHK        (BUMPER_LEFT_TYPE | BUMPER_REAR_TYPE)
#define BUMPER_RI_FWD_CHK        (BUMPER_RIGHT_TYPE | BUMPER_FRONT_TYPE)
#define BUMPER_RI_BWD_CHK        (BUMPER_RIGHT_TYPE | BUMPER_REAR_TYPE)
#define BUMPER_SIDE_FWD_CHK      (BUMPER_LEFT_TYPE | BUMPER_RIGHT_TYPE | BUMPER_FRONT_TYPE)
#define BUMPER_SIDE_BWD_CHK      (BUMPER_LEFT_TYPE | BUMPER_RIGHT_TYPE | BUMPER_REAR_TYPE)
#define BUMPER_ALL_CHK           (BUMPER_FRONT_TYPE | BUMPER_REAR_TYPE |BUMPER_LEFT_TYPE \
											| BUMPER_RIGHT_TYPE)

#define MAX_LAMPS_NUM         16
#define MAX_PATTERNS_NUM      16

#define LIGHT_ALL_OFF         0
#define LIGHT_LEFT_ON         1
#define LIGHT_RIGHT_ON        2
#define LIGHT_ALARM           3

enum TrajType 
{
	UNDEFINED_TRAJ = -1, 
	LINE_TRAJ,             // 0
	SPP_TRAJ,              // 1
	SCP_TRAJ,              // 2
	SIDE_TRAJ,             // 3
	SPIN_TRAJ,             // 4
	STATIC_TRAJ,           // 5
	LAZY_S_TRAJ,           // 6
	SPLINE_TRAJ,           // 7
	ARC_TRAJ,              // 8
	STEER_TRAJ             // 9
};

enum tagTrajDrivePattern
{
	DRIVE_PATTERN_TURN,
	DRIVE_PATTERN_SHIFT
};

#define PROJECT_DX_FACTOR              1.0f                // 1m equivalent to 1
#define PROJECT_THITA_FACTOR           (57.2958f*10.0f)    // 1 rad equivalent to 10 
#define PROJECT_TYPE_FACTOR            30.0f               // Outside type equivalent to 30mm

//
//   A structure defines the data of a projection from a posture to a trajectory.
//
class DllExport CProjectData
{
public:
	USHORT uType;           // The projection type : 0-inside, 1-outside(start), 2-outside(end)
	float  fX;              // The X distance from the posture to the reference point
	float  fY;              // The Y distance from the posture to the reference point
	float  fProgress;       // The progress variable
	float  fTangent;        // The tangent angle of the projection point
	float  fThita;          // The angular deviation (can not be negative)
	float  fOutsideDist;    // The outside distance
	float  fCurvature;      // The curature of the curve at the projection point

public:
	CProjectData() {}

	// The copy constructor
	void operator = (CProjectData& data)
	{
		uType = data.uType;
		fX = data.fX;
		fY = data.fY;
		fProgress = data.fProgress;
		fTangent = data.fTangent;
		fThita = data.fThita;
		fOutsideDist = data.fOutsideDist;
		fCurvature = data.fCurvature;
	}
	
	float DevFactor()
	{
		CAngle ang = abs(CAngle(fThita));
		float fDxFactor = PROJECT_DX_FACTOR * (float)fabs(fX);
		float fOutsideFactor = (uType == 0) ? 0 : PROJECT_TYPE_FACTOR * fOutsideDist;
		float fThitaFactor = PROJECT_THITA_FACTOR * ang.m_fRad;
		return (fDxFactor + fThitaFactor + fOutsideFactor);
	}

	// overloaded operator "<"
    bool operator < (CProjectData& data)
	{
		return DevFactor() < data.DevFactor();
	}
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CTraj".
class DllExport CTraj
{
public:
	SHORT     m_nType;           // Trajectory type
	float     m_fProgRate;       // The rate of progress
	float     m_fFrom;           // The position to start from
	float     m_fTo;             // The position to end at
	float     m_fRange;          // The range of the trajectory specification
	USHORT    m_uBumperPattern;  // The bumper protection pattern
	USHORT    m_uLightType;      // Light type
	CMoveDir  m_WheelMoveDir;    // Wheels' move direction(FWD/BWD)
	CAngle    m_angHeading;      // Tangent angle at the current point
	float     m_fCurvature;      // Curvature at the current point
	CAngle    m_angSteer;        // Steer angle
	CAngle    m_angShift;        // Shift angle
	CPnt      m_pt;              // Vehicle current point
	CPosture  m_pst;             // Vehicle posture at the current point
	CVelocity m_vel;             // Vehicle velocity at the current point

public:
	// Default constructor
	CTraj() 
	{
		m_nType = UNDEFINED_TRAJ;
		m_angSteer = CAngle(0);
		m_angShift = CAngle(0);
		m_uBumperPattern = 0;
		m_uLightType = 0;
	}

	virtual ~CTraj() {};

	// Get the type of the trajectory
	SHORT GetType() {return m_nType;}

	// Vehicle's drive mode on the trajectory
	virtual SHORT GetDrivePattern() {return DRIVE_PATTERN_TURN;}

	// Get the wheel's move direction
	virtual CMoveDir WheelMoveDir() {return m_WheelMoveDir;}

	// Get the vehicle's start steer angle
	virtual CAngle StartSteerAngle() {return m_angSteer;}

	// Get the vehicle's end steer angle
	virtual CAngle EndSteerAngle() {return m_angSteer;}

	// Get the vehicle's start heading angle
	virtual CAngle StartHeading() {return m_angHeading;}

	// Get the vehicle's end heading angle
	virtual CAngle EndHeading() {return m_angHeading;}

	// Get the start point of the trajectory
	virtual CPnt StartPnt() {return m_pt;}
	
	// Get the end point of the trajectory
	virtual CPnt EndPnt() {return m_pt;}
	
	// Get the progress "from" value
	virtual float GetFrom() {return m_fFrom;}
	
	// Get the progress "to" value
	virtual float GetTo() {return m_fTo;}
	
	// Get the total range of the progress variable
	virtual float GetRange() {return m_fRange;}

	// Set the progress variable to specify the current point
	virtual void SetProgress(float fRate, float fProgress) = 0;

	// The trajectory generation function
	virtual CPnt TrajFun() {return m_pt;}

	// The heading generation function
	virtual CAngle HeadingFun() {return m_angHeading;}

	// The steer velocity generation function
	virtual CAngle SteeringFun() {return m_angSteer;}
	
	// The posture generation function
	virtual CPosture PostureFun()
	{
		m_pst.SetPosture(m_pt, m_angHeading);
		return m_pst;
	}

	// Get the shifting angle
	virtual CAngle ShiftAngle() {return m_angShift;}

	// The velocity generation function
	virtual CVelocity VelocityFun() {return m_vel;}

	// The curvature at the start point
	virtual float CurvatureFun() {return m_fCurvature;}

	// The curvature at the start point
	virtual float StartCurvature() {return 0.0f;}

	// The curvature at the end point
	virtual float EndCurvature() {return 0.0f;}

	// Get the deviation between the trajectory and the specified posture
    virtual bool ProjectPosture(CPosture& pst, CProjectData* pData) = 0;

	// Get the profile value for the given velocity vector
	virtual float GetProfileValue(float fLinearVel) {return fLinearVel;}

	// Get the profile slope for the given acceleration
	virtual float GetProfileSlope(float fAcc) {return fAcc;}
	
	// Get the linear according to the profile value
	virtual float GetLinearVel(float fValue) {return fValue;}
};
