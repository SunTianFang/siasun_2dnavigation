//                           - GRAPHICOBJ.CPP -
//
//   Implementation of class "CGraphicObj".

#include <stdafx.h>

class CGraphicObj
{
private:
   int m_nType;

public:
   CGraphicObj() {}
   Draw();

  	// Test whether the point is within the node's selection area
	virtual bool PointHitTest(CPoint& pnt, CScreenReference& ScrnRef);

	virtual void Draw(CScreenReference& ScrnRef, CDC* pDC);
	virtual void DrawID(CScreenReference& ScrnRef, CDC* pDC);
};
