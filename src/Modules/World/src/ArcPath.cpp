//                          - ARCPATH.CPP -
//
//   Implementation of class "CArcPath" - a class defining a arc path in
//   AGVS map.
//
//   Author: Zhang Lei
//   Date:   2000. 10. 28
//
    
#include "stdafx.h"
#include <stdlib.h>
#include "Tools.h"
#include "ArcPath.h"

#ifdef QT_VERSION
#include <QPoint>
#include <QPainter>
#include <QColor>
#endif
#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CArcPath".

CArcPath::CArcPath(USHORT uId, USHORT uStartNode, USHORT uEndNode, CPnt& ptCenter,
	CTurnDir& TurnDir, float fVeloLimit, SHORT nGuideType, USHORT uObstacle,
	USHORT uDir, float fNavParam, USHORT uExtType)
{
	Create(uId, uStartNode, uEndNode, ptCenter, TurnDir, fVeloLimit, nGuideType, uObstacle,
		uDir, fNavParam, uExtType);
}

void CArcPath::Create(USHORT uId, USHORT uStartNode, USHORT uEndNode, CPnt& ptCenter,
	CTurnDir& TurnDir, float fVeloLimit, SHORT nGuideType, USHORT uObstacle,
	USHORT uDir, float fNavParam, USHORT uExtType)
{
	CPath::Create(uId, uStartNode, uEndNode, fVeloLimit, ARC_TYPE, nGuideType, fNavParam, uExtType);

	m_ptCenter = ptCenter;
}

//
//   Init: Initialize the start/end heading angles.
//
void CArcPath::Init()
{
	CPnt& ptStart = GetStartPnt();
	CPnt& ptEnd = GetEndPnt();

	CLine StartLine(m_ptCenter, ptStart);
	m_fRadius = StartLine.Length();

	// Construct the end radius line
	CLine EndLine(m_ptCenter, ptEnd);

	// Construct an 90 degree angle
	CAngle ang90(90.0f, IN_DEGREE);

	if (m_TurnDir == COUNTER_CLOCKWISE)
	{
		m_angStartHeading = StartLine.SlantAngle() + ang90;
		m_angEndHeading = EndLine.SlantAngle() + ang90;

		m_fSize = (EndLine.SlantAngle() - StartLine.SlantAngle()).m_fRad;
	}

	else
	{
		m_angStartHeading = StartLine.SlantAngle() - ang90;
		m_angEndHeading = EndLine.SlantAngle() - ang90;
		m_fSize = (StartLine.SlantAngle() - EndLine.SlantAngle()).m_fRad;
	}
}

//
//   GetHeading: Get the vehicle's heading angle at the specified node.
//
CAngle& CArcPath::GetHeading(CNode& nd)
{
	if (nd == m_uStartNode)
		return m_angStartHeading;
	else
		return m_angEndHeading;
}

CAngle CArcPath::GetTurnAngle()
{
	if (m_TurnDir == COUNTER_CLOCKWISE)
		return m_angEndHeading - m_angStartHeading;
	else
		return m_angStartHeading - m_angEndHeading;
}

//
//   Make a trajectory from the path.
//
CTraj* CArcPath::MakeTraj()
{
	CArcTraj* pArcTraj = new CArcTraj;
	
	CPnt& ptStart = GetStartPnt();
	CPnt& ptEnd = GetEndPnt();
	pArcTraj->CreateTraj(m_ptCenter, ptStart, ptEnd, FORWARD, m_TurnDir);
	return pArcTraj;
}

bool CArcPath::Create(FILE *StreamIn)
{
	unsigned int uObstacle, uDir, uTemp;

	if (!CPath::Create(StreamIn))
		return false;

   if (fscanf(StreamIn, ",\t%u,\t%u,\t%f,\t%f,\t%u\n", 
              &uObstacle,
	           &uDir,
		        &m_ptCenter.x,
		        &m_ptCenter.y,
		        &uTemp) == EOF)
      return false;

	m_uObstacle = uObstacle;
	m_TurnDir.m_tagTurnDir = (uTemp == 0) ? COUNTER_CLOCKWISE: CLOCKWISE;

	if (uDir == NEGATIVE_HEADING)
	{
		uTemp = m_uStartNode;
		m_uStartNode = m_uEndNode;
		m_uEndNode = uTemp;
		m_TurnDir = !m_TurnDir;
		SwabWord(m_uObstacle);
	}

	Init();

	return true;
}

bool CArcPath::Save(FILE *StreamOut)
{
	// Load the fields in the base class
	if (!CPath::Save(StreamOut))
		return false;

	USHORT uTemp = (m_TurnDir == COUNTER_CLOCKWISE) ? 0 : 1;
	USHORT uHeading = POSITIVE_HEADING;

	// Load the fields specific to this class
   if (fprintf(StreamOut, ",\t%u,\t%u,\t%f,\t%f,\t%u\n", 
	            m_uObstacle,
		         uHeading,
		         m_ptCenter.x,
		         m_ptCenter.y,
		         uTemp) == EOF)
      return false;

	return true;
}

//
//   Create the characteristic part of the arc-type path's data
//   from a binary file.
//
bool CArcPath::Create(CArchive& ar)
{
	USHORT uDir;            // Positive input/negative input
	USHORT uTemp;

	if (!CPath::Create(ar))
		return false;

	ar >> m_uFwdRotoScannerObstacle
	   >> m_uFwdObdetectorObstacle 
	   >> m_uBwdRotoScannerObstacle 
	   >> m_uBwdObdetectorObstacle;
	ar >> uDir
	   >> m_ptCenter.x
	   >> m_ptCenter.y
	   >> uTemp;

	m_TurnDir.m_tagTurnDir = (uTemp == 0) ? COUNTER_CLOCKWISE: CLOCKWISE;

	if (uDir == NEGATIVE_HEADING)
	{
		uTemp = m_uStartNode;
		m_uStartNode = m_uEndNode;
		m_uEndNode = uTemp;
		m_TurnDir = !m_TurnDir;
		SwabWord(m_uObstacle);
	}

	Init();

	return true;
}

//
//   Save the characteristic part of the arc-type path's data to a binary file.
//
bool CArcPath::Save(CArchive& ar)
{
	// Load the fields in the base class
	if (!CPath::Save(ar))
		return false;

	USHORT uTemp = (m_TurnDir == COUNTER_CLOCKWISE) ? 0 : 1;
	USHORT uHeading = POSITIVE_HEADING;

	// Load the fields specific to this class
	ar << m_uFwdRotoScannerObstacle 
	   << m_uFwdObdetectorObstacle 
	   << m_uBwdRotoScannerObstacle 
	   << m_uBwdObdetectorObstacle;
	ar	<< uHeading
		<< m_ptCenter.x
		<< m_ptCenter.y
		<< uTemp;

	return true;
}

#ifdef _MFC_VER

//
//   Test whether the specified point is within the selection region of
//   the object.
//
int CArcPath::PointHitTest(CPoint& pnt, CScreenReference& ScrnRef)
{
    return -1;
}

void CArcPath::Draw(CScreenReference& ScrnRef, CDC* pDc, COLORREF cr, int nWidth)
{
	float fRadius = GetRadius();
	CPnt ptRectLeTop(m_ptCenter.x-fRadius, m_ptCenter.y-fRadius);
	CPnt ptRectRiBot(m_ptCenter.x+fRadius, m_ptCenter.y+fRadius);

	CPnt ptStart, ptEnd;
	if (m_TurnDir == COUNTER_CLOCKWISE)
	{
		ptStart = GetStartPnt();
		ptEnd = GetEndPnt();
	}
	else
	{
		ptStart = GetEndPnt();
		ptEnd = GetStartPnt();
	}

	CPoint pntRectLeTop = ScrnRef.GetWindowPoint(ptRectLeTop);
	CPoint pntRectRiBot = ScrnRef.GetWindowPoint(ptRectRiBot);
	CPoint pntStart = ScrnRef.GetWindowPoint(ptStart);
	CPoint pntEnd = ScrnRef.GetWindowPoint(ptEnd);

#if !defined _WIN32_WCE
   CPen Pen(PS_SOLID, nWidth, cr);
   CPen* pOldPen;

//   if (m_uCarrierID != NOT_AN_ID)
      pOldPen = pDc->SelectObject(&Pen);

	pDc->Arc(pntRectLeTop.x, pntRectLeTop.y, pntRectRiBot.x, pntRectRiBot.y,
				pntStart.x, pntStart.y, pntEnd.x, pntEnd.y);

//   if (m_uCarrierID != NOT_AN_ID)
      pDc->SelectObject(pOldPen);
#endif
}

#elif defined QT_VERSION

//
//   Test whether the specified point is within the selection region of
//   the object.
//
int CArcPath::PointHitTest(QPoint& pnt, CScreenReference& ScrnRef)
{
	return -1;
}
void CArcPath::Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr, int nWidth)
{
/*
    float fRadius = GetRadius();
    CPnt ptRectLeTop(m_ptCenter.x-fRadius, m_ptCenter.y-fRadius);
    CPnt ptRectRiBot(m_ptCenter.x+fRadius, m_ptCenter.y+fRadius);

    CPnt ptStart, ptEnd;
    if (m_TurnDir == COUNTER_CLOCKWISE)
    {
        ptStart = GetStartPnt();
        ptEnd = GetEndPnt();
    }
    else
    {
        ptStart = GetEndPnt();
        ptEnd = GetStartPnt();
    }

    QPoint pntRectLeTop = ScrnRef.GetWindowPoint(ptRectLeTop);
    QPoint pntRectRiBot = ScrnRef.GetWindowPoint(ptRectRiBot);
    QPoint pntStart = ScrnRef.GetWindowPoint(ptStart);
    QPoint pntEnd = ScrnRef.GetWindowPoint(ptEnd);

    pPainter->Arc(pntRectLeTop.x, pntRectLeTop.y, pntRectRiBot.x, pntRectRiBot.y,
                pntStart.x, pntStart.y, pntEnd.x, pntEnd.y);
*/
}
#endif
