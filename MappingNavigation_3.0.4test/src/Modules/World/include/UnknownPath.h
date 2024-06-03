#pragma once

#include "Path.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CUnknownPath" - a class defining line-type paths.
class DllExport CUnknownPath : public CPath
{
private:
	CAngle m_angDummy;

protected:
	void Setup();

public:
	// The constructor
	CUnknownPath(USHORT uId, USHORT uStartNode, USHORT uEndNode, float fSize);

	// The default constructor
	CUnknownPath() {}

	// Get the vehicle's required heading at the specified node
	virtual CAngle& GetHeading(CNode& nd) { return m_angDummy; }

	// Make a trajectory from the path
	virtual CTraj* MakeTraj() { return NULL; }

	virtual bool Create(FILE *StreamIn);
	virtual bool Save(FILE *StreamOut);

	virtual bool Create(CArchive& ar);
	virtual bool Save(CArchive& ar);

#ifdef _MFC_VER
    virtual int PointHitTest(CPoint& pnt, CScreenReference& ScrnRef);
    virtual void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr, int nWidth = 1);
#elif defined QT_VERSION
    virtual int PointHitTest(QPoint& pnt, CScreenReference& ScrnRef);
    virtual void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr, int nWidth = 1);
#endif
};

