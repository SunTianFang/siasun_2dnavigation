#pragma once

#include "Path.h"
#include "LineTraj.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CLinePath" - a class defining line-type paths.
class DllExport CLinePath : public CPath
{
protected:
	CAngle m_angHeading;     // The vehicle's heading at the start/end nodes

protected:
	void Setup();

public:
	// The constructor
	CLinePath(USHORT uId, USHORT uStartNode, USHORT uEndNode,
		float fVeloLimit = 0.6f, SHORT nGuideType = TAPE_GUIDANCE,
		USHORT uObstacle = 0, USHORT uDir = POSITIVE_HEADING,
		USHORT uExtType = 0);

	// The default constructor
	CLinePath() {}

	// Get the vehicle's required heading at the specified node
	virtual CAngle& GetHeading(CNode& nd);

	// Make a trajectory from the path
	virtual CTraj* MakeTraj();

	virtual bool Create(FILE *StreamIn);
	virtual bool Save(FILE *StreamOut);

	virtual bool Create(CArchive& ar);
	virtual bool Save(CArchive& ar);

#ifdef _MFC_VER
    // 判断给定的屏幕点是否落在路径上
    virtual int PointHitTest(CPoint& pnt, CScreenReference& ScrnRef);

    virtual void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr, int nWidth = 1);

#elif defined QT_VERSION
    // 判断给定的屏幕点是否落在路径上
    virtual int PointHitTest(QPoint& pnt, CScreenReference& ScrnRef);

    virtual void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr, int nWidth = 1);
#endif
};
