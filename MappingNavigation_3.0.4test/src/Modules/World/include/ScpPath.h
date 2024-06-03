#pragma once

#include "SidePath.h"
#include "ScpTraj.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CScpPath".
class DllExport CScpPath : public CSidePath
{
private:
	CScpTraj* m_pTraj;

public:
	// The default constructor
	CScpPath();
	~CScpPath();

	// Size caculation function
	virtual float SizeFun();

	// Make a trajectory from the path
	virtual CTraj* MakeTraj();

	virtual bool Create(FILE *StreamIn);
	virtual bool Create(CArchive& ar);

#ifdef _MFC_VER

	virtual void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr, int nWidth = 1);
#elif defined QT_VERSION
    virtual void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, int nWidth = 1);
#endif
};
