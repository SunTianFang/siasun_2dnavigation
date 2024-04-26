//                          - NODE.CPP -
//
//   "CNode"类的实现。该类定义了地图中“节点”的概念.
//
//

#include "stdafx.h"

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "Node.h"

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
//   Implementation of class "CNode".

//
//   CNode: The constructor.
//
CNode::CNode(USHORT uId, CPnt& pt, USHORT uType, USHORT uExtType, float fHeading, UCHAR* pTag) : CPnt(pt)
{
	m_uId = uId;
	m_uType = uType;
	m_uExtType = uExtType;
	m_fHeading = fHeading;
	m_Tag.Init(pTag);
}

//
//   IsMarkNode: Check if a mark is available at this node.
//
bool CNode::IsMarkNode()
{
	return ((m_uType & MARK_NODE) != 0);
}

//
//   Check if a temporary mark is available at this node.
//   Return: 
//     0 - No temporary mark
//     1 - Temporary mark #1
//     2 - Temporary mark #2
//
USHORT CNode::IsTempMarkNode()
{
	USHORT uMask = (USHORT)(BIT(4)|BIT(5));
	return ((m_uType & uMask) >> 4); 
}

//
//   IsConveyorNode: Check whether the node is a conveyor node.
//
bool CNode::IsConveyorNode()
{
	return ((m_uType & CONVEYOR_NODE) != 0);
}

//
//   IsSpinTurnNode: Check if this is a spin-turn node.
//
bool CNode::IsSpinTurnNode()
{
	return ((m_uType & SPINTURN_NODE) != 0);
}

//
//   IsChargerNode: Check if this node is a charger station.
//
bool CNode::IsChargerNode()
{
	return ((m_uType & CHARGER_NODE) != 0);
}

//
//   overloaded operator "=".
//
void CNode::operator = (const CNode& nd)
{
	m_uId = nd.m_uId;
	m_uType = nd.m_uType;
	m_uExtType = nd.m_uExtType;
	m_fHeading = nd.m_fHeading;
	m_Tag = nd.m_Tag;
	GetPntObject() = nd;
}

//
//   IsRFIDMarkNode: Check if a RFID mark is available at this node.
//
bool CNode::IsRFIDMarkNode()
{
	return ((m_uType & RFID_MARK_NODE) != 0);
}

bool CNode::IsComNode()
{
	return ((m_uType & COM_NODE) != 0);
}

//
//   operator "==": Check if 2 nodes are identical.
//
bool CNode::operator ==(const CNode& nd) const
{
	// If their ID are equal, they are identical
	return (m_uId == nd.m_uId);
}

//
//   operator "!=": Check if 2 nodes are not identical.
//
bool CNode::operator != (const CNode& nd) const
{
	// If their ID are different, they are not identical
	return (m_uId != nd.m_uId);
}

//
//   operator "==": Check if 2 nodes are identical.
//
bool CNode::operator ==(USHORT uNodeId)
{
	return (m_uId == uNodeId);
}

//
//   operator "!=": Check if 2 nodes are not identical.
//
bool CNode::operator !=(USHORT uNodeId)
{
	return (m_uId != uNodeId);
}

//
//    Create the node data from a text file.
//
bool CNode::Create(FILE *StreamIn)
{
	if (fscanf(StreamIn, "%u,\t%2u", &m_uId, &m_uType) == EOF)
		return false;

	// 如果这是一个“无位置信息节点”,则不再读取其它字段内容
	if (m_uType & PNT_UNKNOWN_NODE)
	{
		x = y = 0;
		m_uExtType = 0;
		m_fHeading = 0;
		fscanf(StreamIn, "\n");
		return true;
	}

	else if (fscanf(StreamIn, ",\t%f,\t%f,\t%u,\t%f\t",
						&x,
						&y

#ifdef WORLD_FORMAT_VER2_0
						,&m_uExtType
						,&m_fHeading
#endif

						) == EOF)
      return false;


#ifdef WORLD_FORMAT_VER2_1
   return m_Tag.Create(StreamIn);
#endif
}

//
//    Save: Save node data to a text file.
//
bool CNode::Save(FILE *StreamOut)
{
	if (fprintf(StreamOut, "%u,\t%2u", m_uId, m_uType) == EOF)
		return false;

	if (m_uType & PNT_UNKNOWN_NODE)
	{
		fprintf(StreamOut, "\n");
		return true;
	}
	else if (fprintf(StreamOut, ",\t%f,\t%f,\t%u,\t%f\t",
						 x,
						 y

#ifdef WORLD_FORMAT_VER2_0
						 ,m_uExtType
						 ,m_fHeading
#endif

						 ) == EOF)
      return false;

#ifdef WORLD_FORMAT_VER2_1
   return m_Tag.Save(StreamOut);
#endif
}

CArchive& operator >> (CArchive& ar, CNode& Obj)
{
	ar >> Obj.m_uId >> Obj.m_uType;
	ar >> Obj.x >> Obj.y;

#ifdef WORLD_FORMAT_VER2_0
	ar >> Obj.m_uExtType;
	ar >> Obj.m_fHeading;

#ifdef WORLD_FORMAT_VER2_1
	ar >> Obj.m_Tag;

#ifdef WORLD_FORMAT_VER3_0
	ar >> Obj.m_fChkMarkDist >> Obj.m_fChkMarkVel >> Obj.m_fMarkWidth;
#endif

#endif

#ifdef WORLD_USE_OFFSET
	ar >> Obj.m_fOffset1;
	ar >> Obj.m_fOffset2;
#endif

#ifdef WORLD_USE_MARKOFFSET
	ar >> Obj.m_fFwdMarkOffset;
	ar >> Obj.m_fBwdMarkOffset;
#endif

#endif

	return ar;
}

CArchive& operator << (CArchive& ar, CNode& Obj)
{
	ar << Obj.m_uId << Obj.m_uType;
	ar << Obj.x << Obj.y;

#ifdef WORLD_FORMAT_VER2_0
	ar << Obj.m_uExtType;
	ar << Obj.m_fHeading;

#ifdef WORLD_FORMAT_VER2_1
	ar << Obj.m_Tag;

#ifdef WORLD_FORMAT_VER3_0
	ar << Obj.m_fChkMarkDist << Obj.m_fChkMarkVel << Obj.m_fMarkWidth;
#endif

#endif

#ifdef WORLD_USE_OFFSET
	ar << Obj.m_fOffset1;
	ar << Obj.m_fOffset2;
#endif

#ifdef WORLD_USE_MARKOFFSET
	ar << Obj.m_fFwdMarkOffset;
	ar << Obj.m_fBwdMarkOffset;
#endif

#endif

	return ar;
}


#ifdef _MFC_VER
//
//   Test whether the given window point is within the selection area of the
//   node.
//
int CNode::PointHitTest(CPoint& pnt, CScreenReference& ScrnRef)
{
	CPnt& ptNode = GetPntObject();
	CPoint pntNode = ScrnRef.GetWindowPoint(ptNode);
	SIZE sizeZero = {0,0};
	CRect r(pntNode, sizeZero);   // Construct an emtpy rectangle
	r.InflateRect(2, 2);
	if (r.PtInRect(pnt))
		return 1;
	else
		return 0;
}

//
//   Draw the node.
//
void CNode::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr)
{
	CBrush Brush(cr);
	CBrush* pOldBrush = pDC->SelectObject(&Brush);

	CPoint pnt1 = ScrnRef.GetWindowPoint(GetPntObject());

	int nSize = 4;
	if (ScrnRef.m_fRatio > 100)
		nSize *= ScrnRef.m_fRatio / 100;

	CRect r(pnt1.x-nSize, pnt1.y-nSize, pnt1.x+nSize, pnt1.y+nSize);
	pDC->Ellipse(&r);

	pDC->SelectObject(pOldBrush);
}

void CNode::DrawID(CScreenReference& ScrnRef, CDC* pDC, LOGFONT* pLogFont)
{
	CPoint pnt1 = ScrnRef.GetWindowPoint(GetPntObject());
   CString str;
   str.Format(_T("%d"), m_uId);

	CFont font;
	CFont* pOldFont = NULL;

	if (pLogFont != NULL && font.CreateFontIndirect(pLogFont))
		pOldFont = pDC->SelectObject(&font);

	COLORREF clrOldText = pDC->SetTextColor(RGB(0, 255, 255));
	int nOldBkMode = pDC->SetBkMode(TRANSPARENT);

	pDC->TextOut(pnt1.x+4, pnt1.y+4, str);

	pDC->SetTextColor(clrOldText);
	pDC->SetBkMode(nOldBkMode);

	if (pOldFont != NULL)
		pDC->SelectObject(pOldFont);
	font.DeleteObject();

/*
	CPoint pnt1 = ScrnRef.GetWindowPoint(pt);
	CFont font;
	CFont* pOldFont = NULL;
	if (font.CreateFontIndirect(&logNodeFont))
		pOldFont = pDC->SelectObject(&font);
	pDC->SetBkMode(TRANSPARENT);
	pDC->SetTextColor(RGB(0,255,255));
	CString strId;
	strId.Format(_T("%d"),m_uId);
	CString str;
	str.Format(_T("%d"), m_uId);
	pDC->TextOut(pnt1.x,pnt1.y, str);
	if (pOldFont != NULL)
		pDC->SelectObject(pOldFont);
	font.DeleteObject();
*/
}

#elif defined QT_VERSION

//
//   Test whether the given window point is within the selection area of the
//   node.
//
int CNode::PointHitTest(QPoint& pnt, CScreenReference& ScrnRef)
{
    CPnt& ptNode = GetPntObject();
    QPoint pntNode = ScrnRef.GetWindowPoint(ptNode);
/*
    SIZE sizeZero = {0,0};
    CRect r(pntNode, sizeZero);   // Construct an emtpy rectangle
    r.InflateRect(2, 2);
    if (r.PtInRect(pnt))
        return 1;
    else
        return 0;
*/
    return 0;
}

//
//   Draw the node.
//
void CNode::Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr)
{
   CPnt::Draw(ScrnRef, pPainter, cr, 5, 0);    // 半径6,填充
}

void CNode::DrawID(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr)
{
    QPoint pnt = ScrnRef.GetWindowPoint(GetPntObject());
    QString str;
    str.sprintf("%d", m_uId);

    QPen pen(cr);
    pPainter->setPen(pen);

    QFont font("Ubuntu", 12, 75);
    pPainter->setFont(font);

    pPainter->drawText(pnt.x() + 10, pnt.y() - 10, str);
}
#endif

