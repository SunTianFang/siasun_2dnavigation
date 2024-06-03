//                          - GENERICPATH.CPP -
//
//   Implementation of class "CGenericPath" - a class defining a generic path in
//   AGVS map.
//
//
    
#include "stdafx.h"
#include <stdlib.h>
#include "Tools.h"
#include "GenericPath.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define BEZIER_K                     0.95f


//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CGenericPath".

// 构造函数
CGenericPath::CGenericPath(int uId, int nStartNode, int nEndNode, CPosture& pstStart, 
									CPosture& pstEnd, float fLen1, float fLen2, 
									float fVeloLimit, SHORT nGuideType,	USHORT uObstacle,
									USHORT uDir, USHORT uExtType) :
CPath(uId, nStartNode, nEndNode, fVeloLimit, GENERIC_TYPE, nGuideType, uExtType)
{
	Create(pstStart, pstEnd, fLen1, fLen2);
}

//
//   缺省的构造函数。
//
CGenericPath::CGenericPath()
{
	m_nCountCtrlPoints = 0;
	m_pptCtrl = NULL;
}

//
//   析构函数。
//
CGenericPath::~CGenericPath()
{
	if (m_pptCtrl != NULL)
		delete []m_pptCtrl;
}

//
//   生成任意阶贝塞尔曲线。
//
bool CGenericPath::Create(CPosture& pstStart, CPosture& pstEnd, int nCountCtrlPoints, CPnt* pptCtrl)
{
	m_pstStart = pstStart;
	m_pstEnd = pstEnd;
	m_nCountCtrlPoints = nCountCtrlPoints;

	// 为关键点分配空间
	m_pptCtrl = new CPnt[m_nCountCtrlPoints];
	if (m_pptCtrl == NULL)
		return false;

	// 复制控制点数据
	for (int i = 0; i < m_nCountCtrlPoints; i++)
		m_pptCtrl[i] = pptCtrl[i];

	// 初始化路径
	return Init();
}

//
//   生成三阶贝塞尔曲线路径。
//
bool CGenericPath::Create(CPosture& pstStart, CPosture& pstEnd, float fLen1, float fLen2)
{
	m_pstStart = pstStart;
	m_pstEnd = pstEnd;
	m_nCountCtrlPoints = 2;

	// 为关键点分配空间
	m_pptCtrl = new CPnt[m_nCountCtrlPoints];
	if (m_pptCtrl == NULL)
		return false;

	CLine ln1(pstStart, fLen1);
	m_pptCtrl[0] = ln1.GetEndPoint();

	CLine ln2(pstEnd, !pstEnd.GetAngle(), fLen2);
	m_pptCtrl[1] = ln2.GetEndPoint();

	Init();

	return true;
}

//
//   根据圆弧路段生成Bezier曲线路段。
//
bool CGenericPath::CreateFromArcPath(CArcPath& ArcPath)
{
	CPnt& pt1 = ArcPath.GetStartPnt();
	CPnt& pt2 = ArcPath.GetEndPnt();
    CPnt ptCenter = ArcPath.GetCenter();

	CLine ln1(ptCenter, pt1);
	CLine ln2(ptCenter, pt2);

	// 圆心到起始/终止点的方向角
	CAngle ang1 = ln1.SlantAngle();
	CAngle ang2 = ln2.SlantAngle();

	// 起始点和终止点处的方向角
	CAngle angStart, angEnd;

	if (ArcPath.GetTurnDir() == COUNTER_CLOCKWISE)
	{
		angStart = ang1 + PI / 2;
		angEnd = ang2 + PI / 2;
	}
	else
	{
		angStart = ang1 - PI / 2;
		angEnd = ang2 - PI / 2;
	}

	m_pstStart.GetPntObject() = pt1;
	m_pstStart.SetAngle(angStart);

	m_pstEnd.GetPntObject() = pt2;
	m_pstEnd.SetAngle(angEnd);

	CAngle angTurn = ArcPath.GetTurnAngle();
	float fLen = ArcPath.GetRadius() * tan(angTurn.m_fRad / 2);
	fLen *= BEZIER_K;

	// 生成三阶贝塞尔曲线路径
	if (!Create(m_pstStart, m_pstEnd, fLen, fLen))
		return false;

	m_uId = ArcPath.m_uId;
	m_uType = GENERIC_TYPE;
	m_uExtType = 0;
	m_uStartNode = ArcPath.m_uStartNode;
	m_uEndNode = ArcPath.m_uEndNode;
	m_fVeloLimit[0] = ArcPath.m_fVeloLimit[0];
	m_fVeloLimit[1] = ArcPath.m_fVeloLimit[1];
	m_uGuideType = ArcPath.m_uGuideType;
	m_fNavParam = ArcPath.m_fNavParam;
	m_uObstacle = ArcPath.m_uObstacle;
	m_uFwdRotoScannerObstacle = ArcPath.m_uFwdRotoScannerObstacle;
	m_uFwdObdetectorObstacle = ArcPath.m_uFwdObdetectorObstacle;
	m_uBwdRotoScannerObstacle = ArcPath.m_uBwdRotoScannerObstacle;
	m_uBwdObdetectorObstacle = ArcPath.m_uBwdObdetectorObstacle;
}

//
//   根据所提供的关键点数量和位置初始化此路径。
//
bool CGenericPath::Init()
{
	// 临时为所有关键点分配空间，以便生成曲线对象
	CPnt* pptKey = new CPnt[m_nCountCtrlPoints + 2];
	if (pptKey == NULL)
		return false;

	// 第一个、最后一个关键点实际上是路径的端节点
	pptKey[0] = m_pstStart.GetPntObject();
	pptKey[m_nCountCtrlPoints + 1] = m_pstEnd.GetPntObject();

	// 复制除起始节点、终止节点之外的关键点
	for (int i = 0; i < m_nCountCtrlPoints; i++)
		pptKey[i + 1] = m_pptCtrl[i];

	// 生成曲线对象
	m_Curve.Create(m_nCountCtrlPoints + 2, pptKey);

	// 释放临时空间
	delete []pptKey;

	// 初始化起点、终点处的方向角
	m_angStartHeading = m_pstStart.GetAngle();
	m_angEndHeading = m_pstEnd.GetAngle();

	return true;
}

//
//   GetHeading: Get the vehicle's heading angle at the specified node.
//
CAngle& CGenericPath::GetHeading(CNode& nd)
{
	if (nd == m_uStartNode)
		return m_angStartHeading;
	else
		return m_angEndHeading;
}

//
//   Make a trajectory from the path.
//
CTraj* CGenericPath::MakeTraj()
{
#if 0
	CBezierTraj* pBezierTraj = new CBezierTraj;
	
	CPnt& ptStart = GetStartPnt();
	CPnt& ptEnd = GetEndPnt();
	pBezierTraj->CreateTraj(ptStart, ptEnd, m_CtrlPnt[1], ptEnd, FORWARD, m_TurnDir, m_bTangency, m_angShiftHeading);
	return pBezierTraj;
#endif
	return NULL;
}

bool CGenericPath::Create(FILE *StreamIn)
{
	return true;
}

bool CGenericPath::Save(FILE *StreamOut)
{
	return true;
}

//
//   Create the characteristic part of the Bezier-type path's data
//   from a binary file.
//
bool CGenericPath::Create(CArchive& ar)
{
	USHORT uDir;            // Positive input/negative input
	USHORT uTemp;
	CPnt CtrlPnt[2];
	float angShiftHeading;
	bool bTangency;

	if (!CPath::Create(ar))
		return false;

	ar >> m_uFwdRotoScannerObstacle
		>> m_uFwdObdetectorObstacle
		>> m_uBwdRotoScannerObstacle
		>> m_uBwdObdetectorObstacle
		>> uDir;

#if 1
	// 读入关键点数量
	ar >> m_nCountCtrlPoints;
#else
	m_nCountCtrlPoints = 2;
#endif

	if (m_pptCtrl != NULL)
		delete []m_pptCtrl;

	m_pptCtrl = new CPnt[m_nCountCtrlPoints];

	for (int i = 0; i < m_nCountCtrlPoints; i++)
	{
		CPnt& pt = m_pptCtrl[i];
		ar >> pt.x >> pt.y;
	}

	ar >> bTangency;

	CPnt ptStart = GetStartNode().GetPntObject();
	CPnt ptEnd = GetEndNode().GetPntObject();
	
	m_pstStart.GetPntObject() = ptStart;
	m_pstEnd.GetPntObject() = ptEnd;

	CLine ln1(ptStart, m_pptCtrl[0]);
	CLine ln2(m_pptCtrl[m_nCountCtrlPoints - 1], ptEnd);

	m_pstStart.fThita = ln1.SlantAngle().m_fRad;
	m_pstEnd.fThita = ln2.SlantAngle().m_fRad;

	return Init();
}

//
//   Save the characteristic part of the Bezier-type path's data to a binary file.
//
bool CGenericPath::Save(CArchive& ar)
{
	// Load the fields in the base class
	if (!CPath::Save(ar))
		return false;

	USHORT uHeading = POSITIVE_HEADING;
	bool bTangency = !m_bTangency;

	// Load the fields specific to this class
	ar << m_uFwdRotoScannerObstacle
		<< m_uFwdObdetectorObstacle
		<< m_uBwdRotoScannerObstacle
		<< m_uBwdObdetectorObstacle
		<< uHeading;

	// 只保存控制点(在此不再保存两个端点)
	int nCountCtrlPoints = m_Curve.m_nCountKeyPoints - 2;
	ar << nCountCtrlPoints;

	// 各个控制点坐标
	for (int i = 0; i < nCountCtrlPoints; i++)
	{
		CPnt& pt = m_Curve.m_ptKey[i + 1];
		ar << pt.x << pt.y;
	}

	ar << bTangency;

	return true;
}

#ifdef _MFC_VER
void CGenericPath::Draw(CScreenReference& ScrnRef, CDC* pDc, COLORREF cr, int nWidth)
{
	m_Curve.Draw(ScrnRef, pDc, cr, nWidth, nWidth, false);
}


//
//   判断给定的屏幕点是否落在曲线上。
//   返回值：
//     -1: 未落在曲线上
//      0: 落在曲线的一般位置上
//    1~n: (n为关键点个数)落在某个关键点上，返回关键点序号(以1为起始序号)
//
int CGenericPath::PointHitTest(CPoint& pnt, CScreenReference& ScrnRef)
{
	SIZE sizeZero = { 0,0 };

	// 判断屏幕点是否落在某个关键点处
	for (int i = 1; i < m_Curve.m_nCountKeyPoints - 1; i++)
	{
		CPoint pntKey = ScrnRef.GetWindowPoint(m_Curve.m_ptKey[i]);

		CRect r(pntKey, sizeZero);   // Construct an emtpy rectangle
		r.InflateRect(2, 2);

		if (r.PtInRect(pnt))
			return i + 1;
	}

	// 下面判断屏幕点是否落在曲线上
	CPnt ptClosest;
	CPnt pt = ScrnRef.GetWorldPoint(pnt);

	// 计算此点在世界坐标系内到曲线的距离
	if (m_Curve.GetClosestPoint(pt, &ptClosest))
	{
		// 计算点到曲线的最近距离
		float fDist = pt.DistanceTo(ptClosest);

		// 换算到屏幕窗口距离
		int nDist = (int)(fDist * ScrnRef.m_fRatio);

		// 如果屏幕窗口距离小于3，认为鼠标触碰到路径
		if (nDist <= 3)
			return 0;
	}

	return -1;
}

//
//   画出此路径曲线的控制点。
//
void CGenericPath::DrawCtrlPoints(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr)
{
	m_Curve.DrawCtrlPoints(ScrnRef, pDC, cr, 3);
}

#endif
