//                          - SIDEPATH.CPP -
//
//   Implementation of class "CPath" - a class defining a generic path in
//   AGVS map. It is the base class of other path classes.
//
//   Author: Zhang Lei
//   Date:   2000. 10. 28
//

#include "stdafx.h"

#include <stdlib.h>
#include <stdio.h>
#include "SidePath.h"
#include "Tools.h"
   
#ifdef QT_VERSION
#include <QPoint>
#include <QPainter>
#endif

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CSidePath".

//
//   GetHeading: Get the vehicle's heading angle at the specified node.
//
CAngle& CSidePath::GetHeading(CNode& /*nd*/)
{
	// The vehicle's heading is the same everywhere on the path
	return m_angHeading;
}

//
//   Size caculation function.
//
float CSidePath::SizeFun()
{
	CPnt& ptStart = GetStartPnt();
	CPnt& ptEnd = GetEndPnt();
	CLine ln(ptStart, ptEnd);
	return ln.Length();
}

//
//   Make a trajectory from the path.
//
CTraj* CSidePath::MakeTraj()
{
	CSideTraj* pSideTraj = new CSideTraj;
	
	CPnt& ptStart = GetStartPnt();
	CPnt& ptEnd = GetEndPnt();
	pSideTraj->CreateTraj(ptStart, ptEnd, m_angHeading);
	
	return pSideTraj;
}

bool CSidePath::Create(FILE *StreamIn)
{
	float fHeading;

	if (!CPath::Create(StreamIn))
		return false;

   if (fscanf(StreamIn, ",\t%f\n", &fHeading) == EOF)
      return false;

	m_angHeading = CAngle(fHeading);
	m_fSize = SizeFun();
	return true;
}

bool CSidePath::Save(FILE *StreamOut)
{
	if (!CPath::Save(StreamOut))
		return false;

   if (fprintf(StreamOut, ",\t%f\n", m_angHeading.m_fRad) == EOF)
      return false;

	return true;
}

//
//   Load line-type path's data from a binary file.
//
//   Note:
//      If the vehicle will move to its left side (steer angle is positive),
//   the move direction is considered as "FORWARD".
//
bool CSidePath::Create(CArchive& ar)
{
	float fHeading;
//	unsigned int uObstacle = 0xFFFF;
//	USHORT uDir;
	USHORT uFwdRotoScannerObstacle = 0xFFFF;
	USHORT uFwdObdetectorObstacle = 0xFFFF;
	USHORT uBwdRotoScannerObstacle = 0xFFFF;
	USHORT uBwdObdetectorObstacle = 0xFFFF;
	float fDir;

	if (!CPath::Create(ar))
		return false;

	ar >> uFwdRotoScannerObstacle
       >> uFwdObdetectorObstacle 
	   >> uBwdRotoScannerObstacle 
	   >> uBwdObdetectorObstacle
	   >> fDir;

	m_uFwdRotoScannerObstacle = uFwdRotoScannerObstacle;
	m_uFwdObdetectorObstacle = uFwdObdetectorObstacle;
	m_uBwdRotoScannerObstacle = uBwdRotoScannerObstacle;
	m_uBwdObdetectorObstacle = uBwdObdetectorObstacle;
	fHeading = fDir;
	m_angHeading = CAngle(fHeading);
	m_fSize = SizeFun();
	return true;
}

//
//   Save sidemove-type path's data to a binary file.
//
//   Note:
//      If the vehicle will move to its left side (steer angle is positive),
//   the move direction is considered as "FORWARD".
//
bool CSidePath::Save(CArchive& ar)
{
	if (!CPath::Save(ar))
		return false;

	ar << m_uFwdRotoScannerObstacle 
	   << m_uFwdObdetectorObstacle 
	   << m_uBwdRotoScannerObstacle 
	   << m_uBwdObdetectorObstacle
	   << m_angHeading.m_fRad;

	return true;
}

#ifdef _MFC_VER

//
//   Test whether the specified point is within the selection region of
//   the object.
//
int CSidePath::PointHitTest(CPoint& pnt, CScreenReference& ScrnRef)
{
    return -1;
}

void CSidePath::Draw(CScreenReference& ScrnRef, CDC* pDc, COLORREF cr, int nWidth)
{
	CPoint pnt1 = ScrnRef.GetWindowPoint(GetStartPnt());
	CPoint pnt2 = ScrnRef.GetWindowPoint(GetEndPnt());

	CPen Pen(PS_SOLID, nWidth, cr);
	CPen* pOldPen = pDc->SelectObject(&Pen);

	pDc->MoveTo(pnt1);
	pDc->LineTo(pnt2);

	pDc->SelectObject(pOldPen);
}

#elif defined QT_VERSION

//
//   Test whether the specified point is within the selection region of 
//   the object.
//
int CSidePath::PointHitTest(QPoint& pnt, CScreenReference& ScrnRef)
{
	return -1;
}

void CSidePath::Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, int nWidth)
{
    CLine ln(GetStartPnt(), GetEndPnt());
    ln.Draw(ScrnRef, pPainter, crColor, nWidth);
}
#endif
