#pragma once

#include "Path.h"
#include "ArcTraj.h"

//////////////////////////////////////////////////////////////////////////////
//   Class "CArcPath": A class derived from base class "CPath", defining
//   arc-type paths.
//
class DllExport CArcPath : public CPath
{
protected:
	CAngle   m_angStartHeading;     // Vehicle heading at the start node
	CAngle   m_angEndHeading;       // Vehicle heading at the end node
	CPnt m_ptCenter;            // Center point of the curve
	CTurnDir m_TurnDir;             // Turning direction of the curve
	float    m_fRadius;             // Turning radius of the curve

protected:
	void Init();

public:
	// The constructor
	CArcPath(USHORT uId, USHORT uStartNode, USHORT uEndNode, CPnt& ptCenter, CTurnDir& TurnDir,
		float fVeloLimit = 0.6f, SHORT nGuideType = TAPE_GUIDANCE,
		USHORT uObstacle = 0, USHORT uDir = POSITIVE_HEADING, float fNavParam = 0,
		USHORT uExtType = 0);

	void Create(USHORT uId, USHORT uStartNode, USHORT uEndNode, CPnt& ptCenter, CTurnDir& TurnDir,
		float fVeloLimit = 0.6f, SHORT nGuideType = TAPE_GUIDANCE,
		USHORT uObstacle = 0, USHORT uDir = POSITIVE_HEADING, float fNavParam = 0,
		USHORT uExtType = 0);

	// The default constructor
	CArcPath() {}

	// Get the radius of the path
	virtual float GetRadius() { return m_fRadius; }

	// Get the center of the path
	virtual CPnt GetCenter() { return m_ptCenter; }

	// Get the turn direction of the path
	virtual CTurnDir GetTurnDir() { return m_TurnDir; }

	// Get the vehicle's required heading at the specified node
	virtual CAngle& GetHeading(CNode& nd);

	virtual CAngle GetStartHeading() { return m_angStartHeading; }

	virtual CAngle GetEndHeading() { return m_angEndHeading; }

	CAngle GetTurnAngle();

	// Make a trajectory from the path
	virtual CTraj* MakeTraj();

	// Fuzzy node checking limit (5 degree)
	virtual float FuzzyNodeCheckLimit() { return 0.087f; }

	virtual bool Create(FILE *StreamIn);
	virtual bool Save(FILE *StreamOut);

	virtual bool Create(CArchive& ar);
	virtual bool Save(CArchive& ar);

#ifdef _MFC_VER
	virtual void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr, int nWidth = 1);
	virtual int PointHitTest(CPoint& pnt, CScreenReference& ScrnRef);
#elif defined QT_VERSION
    virtual void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr, int nWidth = 1);
    virtual int PointHitTest(QPoint& pnt, CScreenReference& ScrnRef);
#endif
};
