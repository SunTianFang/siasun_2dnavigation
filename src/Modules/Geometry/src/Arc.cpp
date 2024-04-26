//                         - ARC.CPP -
//
//   Implementation of class "CArc", which depicts arc curve.
//
//   Author: Zhang Lei
//   Date:   2001. 5. 21
//

#include "stdafx.h"
#include <math.h>

#ifdef QT_VERSION
#include <QPoint>
#include <QPainter>
#include <QColor>
#endif

#include "Geometry.h"
#include "ScrnRef.h"


#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define _CURVATURE_
//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CArc".

//
//   CArc: The constructor.
//
CArc::CArc(const CPnt& ptCenter, const CPnt& ptStart, const CPnt& ptEnd, CTurnDir TurnDir)
{
	m_ptCenter = ptCenter;
	m_ptStart = ptStart;
	m_ptEnd = ptEnd;
	m_TurnDir = TurnDir;

	if (TurnDir == CLOCKWISE)
	{
		m_ptStart = ptEnd;
		m_ptEnd = ptStart;
	}

	CLine StartLine(m_ptCenter, m_ptStart);
	CLine EndLine(m_ptCenter, m_ptEnd);

	m_fCurRadius = m_fRadius = StartLine.Length();
	m_angStart = StartLine.SlantAngle();

	CAngle angTurn = EndLine.SlantAngle() - m_angStart;

	// Get the curve's turn angle
	m_fTurnAngle = angTurn.m_fRad;
	m_Transform = CTransform(m_ptCenter, m_angStart);
}

//
//   SetCurAngle: Set the current turn angle to specified a point.
//
void CArc::SetCurAngle(float fPhi)
{
	if (m_TurnDir == CLOCKWISE)
		fPhi = m_fTurnAngle - fPhi;

	m_fCurvature = 1.0f / m_fRadius;

	m_angTangent = m_angStart + CAngle(PI/2+fPhi);

	CPnt ptTemp((float)(m_fRadius * cos(fPhi)), (float)(m_fRadius * sin(fPhi)));     // Local point
	m_pt = m_Transform.GetWorldPoint(ptTemp);        // World point

	// If the turn direction is "CLOCKWISE", make some adjustments
	if (m_TurnDir == CLOCKWISE)
	{
		m_angTangent = !m_angTangent;

#if defined _CURVATURE_
		m_fCurvature = -m_fCurvature;
#endif

	}
}

#ifdef _MFC_VER

//
//   在屏幕上绘制此圆。
//
void CArc::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPenStyle)
{
	CPen pen(nPenStyle, nWidth, crColor);
	CPen* pOldPen = pDC->SelectObject(&pen);

	CPnt ptLeftTop(m_ptCenter.x - m_fRadius, m_ptCenter.y + m_fRadius);
	CPnt ptRightBottom(m_ptCenter.x + m_fRadius, m_ptCenter.y - m_fRadius);

	CPoint pnt1 = ScrnRef.GetWindowPoint(ptLeftTop);
	CPoint pnt2 = ScrnRef.GetWindowPoint(ptRightBottom);

	CRect r(pnt1, pnt2);
	CLine ln1(m_ptCenter, m_angStart, m_fRadius);
	CLine ln2(m_ptCenter, m_angStart + m_fTurnAngle, m_fRadius);

	CPoint pntStart = ScrnRef.GetWindowPoint(ln1.m_ptEnd);
	CPoint pntEnd = ScrnRef.GetWindowPoint(ln2.m_ptEnd);

	pDC->Arc(r, pntStart, pntEnd);

	pDC->SelectObject(pOldPen);
}

#elif defined QT_VERSION
//
//   在屏幕上绘制此圆。
//
void CArc::Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, int nWidth, int nPenStyle)
{
	QPen pen(crColor);
	pen.setWidth(nWidth);
	QBrush brush(crColor, Qt::NoBrush);

	pPainter->setPen(pen);
	pPainter->setBrush(brush);

	CPnt ptLeftTop(m_ptCenter.x - m_fRadius, m_ptCenter.y + m_fRadius);
	CPnt ptRightBottom(m_ptCenter.x + m_fRadius, m_ptCenter.y - m_fRadius);

	QPoint pnt1 = ScrnRef.GetWindowPoint(ptLeftTop);
	QPoint pnt2 = ScrnRef.GetWindowPoint(ptRightBottom);

	QRect r(pnt1, pnt2);
	pPainter->drawEllipse(r);
}
#endif
