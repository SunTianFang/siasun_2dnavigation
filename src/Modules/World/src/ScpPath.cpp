//                          - SCPPATH.CPP -
//
//   Implementation of class "CScpPath" - a class defining a SCP path in
//   AGVS map.
//
//   Author: Zhang Lei
//   Date:   2000. 10. 28
//

#include "stdafx.h"
#include <stdlib.h>
#include "Tools.h"
#include "ScpPath.h"
   
#ifdef QT_VERSION
#include <QPoint>
#include <QPainter>
#endif

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define SCP_DRAWING_POINTS     50

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CScpPath".

CScpPath::CScpPath()
{
	m_pTraj = NULL;
}

CScpPath::~CScpPath()
{
	if (m_pTraj != NULL)
		delete m_pTraj;
}

//
//   Size caculation function.
//
float CScpPath::SizeFun()
{
	// Construct a line to find out the leng of the path
	CPnt& ptStart = GetStartPnt();
	CPnt& ptEnd = GetEndPnt();
	CLine ln(ptStart, ptEnd);
	return (float)fabs(ln.Length() * cos(ln.SlantAngle() - m_angHeading));
}

//
//   Make a trajectory from the path.
//
CTraj* CScpPath::MakeTraj()
{
	CScpTraj* pScpTraj = new CScpTraj;
	CPnt& ptStart = GetStartPnt();
	CPnt& ptEnd = GetEndPnt();
	pScpTraj->CreateTraj(ptStart, ptEnd, m_angHeading);
	return pScpTraj;
}

bool CScpPath::Create(FILE *StreamIn)
{
	if (!CSidePath::Create(StreamIn))
		return false;

	m_pTraj = (CScpTraj*)MakeTraj();
	return (m_pTraj != NULL);
}

bool CScpPath::Create(CArchive& ar)
{
	if (!CSidePath::Create(ar))
		return false;

	m_pTraj = (CScpTraj*)MakeTraj();
	return (m_pTraj != NULL);
}

#ifdef _MFC_VER

void CScpPath::Draw(CScreenReference& ScrnRef, CDC* pDc, COLORREF cr, int nWidth)
{
	CPoint pnt1 = ScrnRef.GetWindowPoint(GetStartPnt());
	CPoint pnt2 = ScrnRef.GetWindowPoint(GetEndPnt());;

   CPen Pen(PS_SOLID, nWidth, cr);
   CPen* pOldPen;
   pOldPen = pDc->SelectObject(&Pen);

	// 起点
	pDc->MoveTo(pnt1);

	float fRange = m_pTraj->GetRange();
	float fRate = fRange/SCP_DRAWING_POINTS;
	float fProgress;

	// 依次描点
	for (int i = 0; i < SCP_DRAWING_POINTS; i++)
	{
		// 计算轨迹点
		fProgress = i * fRate;
		m_pTraj->SetProgress(fRate, fProgress);
		CPnt pt = m_pTraj->TrajFun();

		// 转换到窗口坐标
		CPoint pnt = ScrnRef.GetWindowPoint(pt);

		// 连线
		pDc->LineTo(pnt);
	}

	// 连到终点
	pDc->LineTo(pnt2);

   pDc->SelectObject(pOldPen);
}

#elif defined QT_VERSION
void CScpPath::Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, int nWidth)
{
    QPen pen(crColor);
    pen.setWidth(nWidth);
    pPainter->setPen(pen);

    QPoint pnt1 = ScrnRef.GetWindowPoint(GetStartPnt());
    QPoint pnt2 = ScrnRef.GetWindowPoint(GetEndPnt());;

    float fRange = m_pTraj->GetRange();
    float fRate = fRange/SCP_DRAWING_POINTS;
    float fProgress;

    // 依次描点
    for (int i = 0; i < SCP_DRAWING_POINTS; i++)
    {
        // 计算轨迹点
        fProgress = i * fRate;
        m_pTraj->SetProgress(fRate, fProgress);
        CPnt pt = m_pTraj->TrajFun();

        // 转换到窗口坐标
        QPoint pnt = ScrnRef.GetWindowPoint(pt);

        // 连线
        pPainter->drawLine(pnt1, pnt);
        pnt1 = pnt;
    }

    // 连到终点
    pPainter->drawLine(pnt1, pnt2);
}
#endif
