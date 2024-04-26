//                          - PATH.H -
//
//   The interface of class "CPath", which defines the common features
//   of a generic path.
//
//   Author: Zhang Lei
//   Date:   2001. 7. 31
//

#pragma once

#include <stdlib.h>
#include <stdio.h>
#include "NodeBase.h"
#include "Traj.h"

// Defines NULL ID of nodes and paths
#define NULLID	             ((USHORT)0xFFFF)

// Enumeration of standard path types
enum TPathType {LINE_TYPE, SPP_TYPE, SPLINE_TYPE, SCP_TYPE, SIDE_TYPE,
					 LAZY_S_TYPE, ARC_TYPE = 6, SPP_SHIFT_TYPE = 9, GENERIC_TYPE = 10, 
					 UNKNOWN_PATH_TYPE = 100};

// Enumeration of vehicle's heading rules -
//   POSITIVE_HEADING - vehicle heading should be: [Start] -> [End]
//   NEGATIVE_HEADING - vehicle heading should be: [End] -> [Start]
enum THeadingRule {POSITIVE_HEADING, NEGATIVE_HEADING};

// Enumeration of path guidance type
//enum TGuideType {NO_GUIDANCE = 0, TAPE_GUIDANCE = 2, LASER_GUIDANCE = 1, TAPE_LASER_GUIDANCE = 3};
#define NO_GUIDANCE            ((USHORT)0)
#define TAPE_GUIDANCE          ((USHORT)1)
#define LASER_GUIDANCE         ((USHORT)2)
#define TAPE_LASER_GUIDANCE    ((USHORT)3)
#define OD_LEFT_GUIDANCE       ((USHORT)4)
#define OD_RIGHT_GUIDANCE      ((USHORT)5)

#define COMMON_CARRIER_COST    (100.0f)

// 分支类型定义
#define NO_BRANCH               0                       // 无分支
#define LEFT_BRANCH             1                       // 左分支
#define RIGHT_BRANCH            2                       // 右分支
#define ERROR_BRANCH            3                       // 分支超过2个，错误

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CPath".
class DllExport CPath
{
public:
	USHORT m_uId;                  // Path ID number
	USHORT m_uType;                // Path topological type
	USHORT m_uExtType;             // Extended path type
	USHORT m_uStartNode;           // The ID of the start node
	USHORT m_uEndNode;             // The ID of the end node
	float  m_fSize;                // Size of the path
	float  m_fVeloLimit[2];        // The velocity limit on the path
	USHORT m_uGuideType;           // The guide method on this path
											 // 0 - Dead reckoning; D0 - magnetic; D1 - laser
	float  m_fNavParam;            // Navigation parameter
	USHORT m_uObstacle;            // Obstacle basic checking data
	USHORT m_uFwdRotoScannerObstacle;
	USHORT m_uFwdObdetectorObstacle;
	USHORT m_uBwdRotoScannerObstacle;
	USHORT m_uBwdObdetectorObstacle;

	bool	 m_bTimeOrVel;			// 0 - Time , 1 - Velocity
	float	 m_fTimeValue[2];			// Time value
	CAngle   m_angShiftHeading; // 车体平移方向角

#ifdef _MFC_VER
	COLORREF m_clr;
#endif

public:
	static CNodeBase* m_pNodeBase;   // Pointer to the nodes data base

public:
	// The constructor
	CPath(USHORT uId, USHORT uStartNode, USHORT uEndNode, float fVeloLimit, 
			USHORT uType = LINE_TYPE, USHORT uGuideType = TAPE_LASER_GUIDANCE, 
			float fNavParam = 0, USHORT uExtType = 0);

	// The default constructor
	CPath() 
   {
      m_uObstacle = 0;
   }

	void Create(USHORT uId, USHORT uStartNode, USHORT uEndNode, float fVeloLimit,
		USHORT uType = LINE_TYPE, USHORT uGuideType = TAPE_LASER_GUIDANCE,
		float fNavParam = 0, USHORT uExtType = 0);

	// Get the guide method on this path
	USHORT GuideType() {return m_uGuideType;}

	// Get the pointer to the start node
	CNode& GetStartNode();

	// Get the pointer to the end node
	CNode& GetEndNode();

	// Get the world point of the start node
	CPnt& GetStartPnt();

	// Get the world point of the end node
	CPnt& GetEndPnt();

	// Test whether 2 path objects are equal
	bool operator == (CPath& path) {return (m_uId == path.m_uId);}

	// Test whether 2 path objects are not equal
	bool operator != (CPath& path) {return (m_uId != path.m_uId);}

	// Determine the vehicle's heading angle at the specified node
	virtual CAngle& GetHeading(CNode& nd) = 0;

	virtual CAngle& GetShiftHeading() {return m_angShiftHeading;}

	// Get the size of the path
	virtual float Size() {return m_fSize;}
   
   // Make a trajectory from the path
	virtual CTraj* MakeTraj() = 0;
	
	// Fuzzy node checking limit (0.03m)
	virtual float FuzzyNodeCheckLimit() {return 0.03f;}
	
	

	virtual bool Create(FILE *StreamIn);
	virtual bool Save(FILE *StreamOut);

	virtual bool Create(CArchive& ar);
	virtual bool Save(CArchive& ar);

#ifdef _MFC_VER
	void SetColor(COLORREF clr) {m_clr = clr;}
	virtual int PointHitTest(CPoint& pnt, CScreenReference& ScrnRef) = 0;

	virtual void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr, int nWidth = 2) = 0;
#elif defined QT_VERSION

    virtual int PointHitTest(QPoint& pnt, CScreenReference& ScrnRef) = 0;

    virtual void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr, int nWidth = 2) = 0;
#endif
};

