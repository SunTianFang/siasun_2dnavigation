//                              - UNKNOWNPATH.CPP -
//
//   Implementation of class "CUnknownPath" - a class defining a line path in
//   AGVS map.
//
//   Author: Zhang Lei
//   Date:   2001. 7. 30
//
                 
#include "stdafx.h"
#include <stdio.h>
#include "UnknownPath.h"
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
//   Implementation of class "CUnknownPath".

//
//   CUnknownPath: The constructor.
//

CUnknownPath::CUnknownPath(USHORT uId, USHORT uStartNode, USHORT uEndNode, float fSize) :
CPath(uId, uStartNode, uEndNode, 0.0f, UNKNOWN_PATH_TYPE, TAPE_GUIDANCE, 0)
{
	m_uObstacle = 0;
	m_fSize = fSize;
}

//
//
//   Setup: Find the vehicle's headings at the 2 nodes.
void CUnknownPath::Setup()
{
}

bool CUnknownPath::Create(FILE *StreamIn)
{
	unsigned int uId, uStartNode, uEndNode;
	float fSize;

	if (fscanf(StreamIn, "%u,\t%u,\t%u,\t%f\n",
				  &uId,
				  &uStartNode,
				  &uEndNode,
				  &fSize) == EOF)
      return false;
	m_uId = uId;
	m_uStartNode = uStartNode;
	m_uEndNode = uEndNode;
	m_fSize = fSize;
   m_uGuideType = TAPE_GUIDANCE;

#ifdef WORLD_FORMAT_VER2_0
	m_uExtType = 0;
	m_fNavParam = 0;
#endif

	m_uObstacle = 0;

	// Success, return true
	return true;
}

bool CUnknownPath::Save(FILE *StreamOut)
{
	if (fprintf(StreamOut, "%u,\t%u,\t%u\n",
				  m_uId,
				  m_uStartNode,
				  m_uEndNode) == EOF)
      return false;

	// Success, return true
	return true;
}

bool CUnknownPath::Create(CArchive& ar)
{
	return true;
}

bool CUnknownPath::Save(CArchive& ar)
{
	return true;
}

#ifdef _MFC_VER

int CUnkonwPath::PointHitTest(CPoint& pnt, CScreenReference& ScrnRef)
{
    return -1;
}

void CUnknownPath::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr, int nWidth)
{
}

#elif defined QT_VERSION
int CUnknownPath::PointHitTest(QPoint& pnt, CScreenReference& ScrnRef)
{
    return -1;
}

void CUnknownPath::Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr, int nWidth)
{

}
#endif
