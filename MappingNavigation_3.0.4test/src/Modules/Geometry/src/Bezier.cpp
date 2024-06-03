#include <stdafx.h>
#include <math.h>

#ifdef QT_VERSION
#include <QPoint>
#include <QPainter>
#include <QColor>
#endif

#include "Bezier.h"
#include "Geometry.h"
#include "ScrnRef.h"
#include "Combination.h"

#define BEZIER_CURVE_DEFAULT_SEG_COUNT         100

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////
//   辅助函数。

//
//   计算浮点数的整数次方
//
float FloatPowerInt(float x, int nRank)
{
	float f = x;

	if (nRank == 0)
		return 1;
	
	for (int i = 1; i < nRank; i++)
		f *= x;

	return f;
}

///////////////////////////////////////////////////////////////////////////////

//
//   构造任意阶贝塞尔曲线。
//
CBezier::CBezier(int nCountKeyPoints, CPnt* pptKey)
{
	m_ptKey = NULL;
	m_pSamplePoints = NULL;

	// 生成任意阶贝塞尔曲线
	Create(nCountKeyPoints, pptKey);
}

//
//   构造三阶贝塞尔曲线。
//
CBezier::CBezier(const CPosture& pstStart, const CPosture& pstEnd, float fLen1, float fLen2)
{
	m_ptKey = NULL;
	m_pSamplePoints = NULL;

	// 生成三阶贝塞尔曲线
	Create(pstStart, pstEnd, fLen1, fLen2);
}

//
//   缺省构造函数(后续需要显式调用Create函数)。
//
CBezier::CBezier()
{
	m_ptKey = NULL;
	m_pSamplePoints = NULL;
}

CBezier::~CBezier()
{
	Clear();
}

//
//   清除空间。
//
void CBezier::Clear()
{
	if (m_ptKey != NULL)
	{
		delete[]m_ptKey;
		m_ptKey = NULL;
	}

	if (m_pSamplePoints != NULL)
	{
		delete[]m_pSamplePoints;
		m_pSamplePoints = NULL;
	}
}

//
//   根据给定的关键点生成任意阶Bezier曲线。
//
bool CBezier::Create(int nCountKeyPoints, CPnt* pptKey)
{
	if (m_ptKey != NULL)
		delete[]m_ptKey;

	// 保存关键点数量
	m_nCountKeyPoints = nCountKeyPoints;

	// 为所有关键点分配空间
	m_ptKey = new CPnt[m_nCountKeyPoints];
	if (m_ptKey == NULL)
		return false;

	// 依次记录各个锚点
	for (int i = 0; i < m_nCountKeyPoints; i++)
		m_ptKey[i] = pptKey[i];

	m_nSampleCount = BEZIER_CURVE_DEFAULT_SEG_COUNT;

	// 重新生成采样点数据
	return CreateSamplePoints();
}

//
//   生成三阶贝塞尔曲线。
//
bool CBezier::Create(const CPosture& pstStart, const CPosture& pstEnd, float fLen1, float fLen2)
{
	if (m_ptKey != NULL)
		delete []m_ptKey;

	m_nCountKeyPoints = 4;
	m_ptKey = new CPnt[m_nCountKeyPoints];
	if (m_ptKey == NULL)
		return NULL;

	m_ptKey[0] = pstStart;
	m_ptKey[3] = pstEnd;

	CLine ln1(pstStart, fLen1);
	m_ptKey[1] = ln1.GetEndPoint();

	CLine ln2(pstEnd, !pstEnd.GetAngle(),fLen2);
	m_ptKey[2] = ln2.GetEndPoint();

	m_nSampleCount = BEZIER_CURVE_DEFAULT_SEG_COUNT;

	// 重新生成采样点数据
	return CreateSamplePoints();
}

//
//   根据选点常数K生成三阶Bezier曲线
//
bool CBezier::Create(const CPosture& pstStart, const CPosture& pstEnd, float K)
{
	if (m_ptKey != NULL)
		delete[]m_ptKey;

	m_nCountKeyPoints = 4;
	m_ptKey = new CPnt[m_nCountKeyPoints];
	if (m_ptKey == NULL)
		return false;

	// 先找到两个姿态的交点
	CLine lnTemp1(pstStart, 1000.0f);
	CLine lnTemp2(pstEnd, 1000.0f);

	CPnt pt;
	float fLen1, fLen2;

	// 如果两个姿态是平行的
	if (!lnTemp1.Intersect(lnTemp2, &pt.x, &pt.y, NULL, NULL, 1e-4))
	{
		lnTemp1.DistanceToPoint(false, pstEnd, NULL, &pt);
		float fDist = pstStart.DistanceTo(pt);
		if (fDist < 0.05f)
			return false;

		fLen1 = fLen2 = fDist * 0.5f;
	}

	// 如果两个姿态所在的直线有交点
	else
	{
		// 计算两个姿态所形成的夹角大小

		CAngle angDiff = pstEnd.GetAngle() - pstStart.GetAngle();
		float fPhi = angDiff.m_fRad;
		if (fPhi > PI)
			fPhi = 2 * PI - fPhi;
		if (fPhi > PI / 2)
			fPhi = PI - fPhi;
		fPhi /= 2;

		if (fPhi < PI / 9)
		{
			float fDist = pstStart.DistanceTo(pstEnd);
			fLen1 = fLen2 = fDist / 2;
		}
		else
		{
			fLen1 = pstStart.DistanceTo(pt);
			fLen2 = pstEnd.DistanceTo(pt);
			fLen1 *= fabs(tan(fPhi));
			fLen2 *= fabs(tan(fPhi));
		}
	}

	// 计算Bezier曲线的第一个和最后一个锚点
	m_ptKey[0] = pstStart;
	m_ptKey[3] = pstEnd;

	// 第二个锚点
	CLine ln1(pstStart, fLen1);
	m_ptKey[1] = ln1.GetEndPoint();

	// 第三个锚点
	CLine ln2(pstEnd, !pstEnd.GetAngle(), fLen2);
	m_ptKey[2] = ln2.GetEndPoint();

	m_nSampleCount = BEZIER_CURVE_DEFAULT_SEG_COUNT;

	// 重新生成采样点数据
	return CreateSamplePoints();
}

//
//    生成离散采样点数据。
//
bool CBezier::CreateSamplePoints()
{
	// 释放采样点数据
	if (m_pSamplePoints != NULL)
		delete []m_pSamplePoints;

	// 为各采样点数据分段空间
	m_pSamplePoints = new CCurveSamplePoint[m_nSampleCount];
	if (m_pSamplePoints == NULL)
		return false;

	// 计算曲线上各采样点的数据
	m_fTotalLen = 0;
	for (int i = 0; i < m_nSampleCount; i++)
	{
		float t = (float)i / m_nSampleCount;
		SetCurT(t);

		// 计算本小段曲线的长度
		float fSegLen = sqrt(dx1 * dx1 + dy1 * dy1) * 1.0f / m_nSampleCount;    // ?????? /m_nSampleCount是为什么?

		// 计算累计到当前点的曲线长度
		m_fTotalLen += fSegLen;   

		// 保存对应于该采样点的数据
		CCurveSamplePoint& sp = m_pSamplePoints[i];
		sp.t = t;
		sp.fProgress = m_fTotalLen;
		sp.fSegLen = fSegLen;
		sp.GetPntObject() = TrajFun();
		sp.fTangentAngle = TangentFun().m_fRad;
		sp.fCurvature = CurvatureFun();
	}

	return true;
}

//
//   重载“=”操作符。
//
void CBezier::operator = (const CBezier& Obj)
{
	Clear();

	m_nCountKeyPoints = Obj.m_nCountKeyPoints;
	Create(m_nCountKeyPoints, Obj.m_ptKey);
}

//
//   在指定的序号处增加一个控制点。
//
bool CBezier::AddKeyPoint(int nIdx, const CPnt& pt)
{
	// 增加一个关键点，需要重新为关键点分配空间
	CPnt* p = new CPnt[m_nCountKeyPoints + 1];
	if (p == NULL)
		return false;

	// 复制关键点，并增加新点
	for (int i = 0; i < nIdx; i++)
		p[i] = m_ptKey[i];

	p[nIdx] = pt;

	for (int i = nIdx + 1; i < m_nCountKeyPoints + 1; i++)
		p[i] = m_ptKey[i - 1];

	// 释放原来的关键点数据空间
	delete []m_ptKey;

	// 重新设置关键点数据区
	m_ptKey = p;

	m_nCountKeyPoints++;

	// 重新生成采样点数据
	return CreateSamplePoints();
}

//
//   将指定序号处的控制点移除。
//
bool CBezier::RemoveKeyPoint(int nIdx)
{
	// 三阶曲线，已不能再降阶了。另外，两个端点也不能删除
	if (m_nCountKeyPoints == 4 || nIdx == 0 || nIdx == m_nCountKeyPoints - 1)
		return false;

	// 删除指定的控制点
	for (int i = nIdx; i < m_nCountKeyPoints - 1; i++)
		m_ptKey[i] = m_ptKey[i + 1];

	m_nCountKeyPoints--;

	// 重新生成采样点数据
	return CreateSamplePoints();
}

//
//   根据给定的处理函数，生成曲线的用户数据。
//
void CBezier::CreateUserData(void(*pProc)(CBezier*, void*), void* pParam)
{
	UserDataCreateProc = pProc;
	UserDataCreateProc(this, pParam);
}

//
//   取得指定的用户数据。
//
float CBezier::GetUserData(int nSampleIdx, int nDataIdx) const
{
	return m_pSamplePoints[nSampleIdx].fUserData[nDataIdx];
}

//
//   核对一下，看曲线是否突破了规定的约束。
//
bool CBezier::CheckConstraints(float fMaxCurvature, float fMaxCurvatureDiff) const
{
	float t;
	for (int i = 0; i < m_nSampleCount; i++)
	{
		CCurveSamplePoint& sp = m_pSamplePoints[i];

		// 目前只验证曲率值
		if (sp.fCurvature > fMaxCurvature)
			return false;
	}

	return true;
}

//
//   设置当前进度以便确定当前点。
//
void CBezier::SetCurT(float t)
{
	float f1 = (1-t);
	float f2 = f1 * f1;
	float f3 = f2 * f1;

	// 计算轨迹点(m_pt.x, m_pt.y)
#if 0
	m_pt.x = f3 * m_ptKey[0].x + 3 * t * f2 * m_ptKey[1].x + 
				3 * t * t * f1 * m_ptKey[2].x + t * t * t * m_ptKey[3].x;

	m_pt.y = f3 * m_ptKey[0].y + 3 * t * f2 * m_ptKey[1].y + 
				3 * t * t * f1 * m_ptKey[2].y + t * t * t * m_ptKey[3].y;

	// 计算一阶导数(dx1, dy1)
	dx1 = 3 * (m_ptKey[1].x - m_ptKey[0].x) +
		6 * (m_ptKey[2].x - 2 * m_ptKey[1].x + m_ptKey[0].x) * t +
		3 * (m_ptKey[3].x - 3 * m_ptKey[2].x + 3 * m_ptKey[1].x - m_ptKey[0].x) * t * t;

	dy1 = 3 * (m_ptKey[1].y - m_ptKey[0].y) +
		6 * (m_ptKey[2].y - 2 * m_ptKey[1].y + m_ptKey[0].y) * t +
		3 * (m_ptKey[3].y - 3 * m_ptKey[2].y + 3 * m_ptKey[1].y - m_ptKey[0].y) * t * t;


#else

	int n = m_nCountKeyPoints - 1;
	m_pt.x = m_pt.y = 0;

	// 计算轨迹点(m_pt.x, m_pt.y)
	for (int i = 0; i <= n; i++)
	{
		int c = Combination(n, i);
		float p1 = FloatPowerInt(1 - t, n - i);
		float p2 = FloatPowerInt(t, i);
		float f = c * p1 * p2;
		m_pt.x += m_ptKey[i].x * f;
		m_pt.y += m_ptKey[i].y * f;
	}

	// 计算一阶导数(dx1, dy1)
	dx1 = dy1 = 0;
	for (int i = 0; i <= n; i++)
	{
		int c = Combination(n, i);
		float p1 = FloatPowerInt(1 - t, n - i - 1);
		float p2 = FloatPowerInt(t, i);
		float p3 = FloatPowerInt(t, i - 1);
		float p4 = FloatPowerInt(1 - t, n - i);

		float f = -(n-i) * p1 * p2 + i * p4 * p3;
		dx1 += c * m_ptKey[i].x * f;
		dy1 += c * m_ptKey[i].y * f;
	}

	// 计算二阶导数(dx2, dy2)
	dx2 = 6 * (m_ptKey[2].x - 2 * m_ptKey[1].x + m_ptKey[0].x) +
		6 * (m_ptKey[3].x - 3 * m_ptKey[2].x + 3 * m_ptKey[1].x - m_ptKey[0].x) * t;

	dy2 = 6 * (m_ptKey[2].y - 2 * m_ptKey[1].y + m_ptKey[0].y) +
		6 * (m_ptKey[3].y - 3 * m_ptKey[2].y + 3 * m_ptKey[1].y - m_ptKey[0].y) * t;

	// 计算二阶导数(dx2, dy2)
	float _dx2, _dy2;
	_dx2 = _dy2 = 0;
	for (int i = 0; i <= n; i++)
	{
		int c = Combination(n, i);
		float p1 = (n - i - 1 >= 0) ? FloatPowerInt(1 - t, n - i - 1) : 0;
		float p2 = FloatPowerInt(t, i);
		float p3 = FloatPowerInt(t, i - 1);
		float p4 = FloatPowerInt(1 - t, n - i);
		float p5 = (i - 2 >= 0) ? FloatPowerInt(t, i - 2) : 0;
		float p6 = (n - i - 2 >= 0) ? FloatPowerInt(1 - t, n - i - 2) : 0;

		float f = (n-i) * ((-(n-i-1)) * p6 * p2 + p1 * i * p3) + 
			        i * ((-n+i) * p1 * p3 + p4 * (i - 1) * p5);
		_dx2 += c * m_ptKey[i].x * f;
		_dy2 += c * m_ptKey[i].y * f;
	}

	float dd3 = dx2 - _dx2;
	float dd4 = dy2 - _dy2;
#endif


	// 计算曲线的曲率
	float f = dx1* dx1 + dy1 * dy1;
	f = (float)sqrt(f * f * f);
	m_fCurvature = (dx1 * dy2 - dx2 * dy1) / f;

	// 计算曲线的切线角
	m_angTangent = (float)atan2(dy1, dx1);
}

//
//   根据距起点的进度距离fLen确定对应的t值。
//
float CBezier::GetTFromProgress(float fLen)
{
	float t;

	if (fLen == 0)
		return 0;
	else if (fLen >= m_fTotalLen)
		return 1;

	// 查找对应于fLen长度的区间，并通过线性插补得到对应的t值

#if 0
	// 将来测试!
	// 通过折半查找提高速度
	int nMin, nMax, n;
	nMin = 0;
	nMax = m_nSampleCount;
	n = (nMin + nMax) / 2;

	while (nMax - nMin > 2)
	{
		float fProgress = m_pSamplePoints[n].fProgress;
		if (fProgress > fLen)
		{
			nMax = n;
		}
		n = (nMin + nMax) / 2;
	}
#endif

	for (int i = 0; i < m_nSampleCount; i++)
	{
		float fProgress = m_pSamplePoints[i].fProgress;
		if (fProgress > fLen)
		{
			float fProgress_1 = m_pSamplePoints[i - 1].fProgress;
			t = (float)(i - 1) / m_nSampleCount;
			t *= 1 + (fLen - fProgress_1) / (fProgress - fProgress_1);
			return t;
		}
	}

	return 1;
}

//
//   计算曲线外一点pt到曲线上最近距离的点。
//
bool CBezier::GetClosestPoint(const CPnt& pt, CPnt* pptClosest, float* pT)
{
	// 先假定第一点为最近点
	int nClosest = 0;
	CPnt ptClosestSample = m_pSamplePoints[0].GetPntObject();

	// 依次在各采样点中查找距离给定点pt最近的点
	float fMinDist = ptClosestSample.DistanceTo(pt);
	for (int i = 1; i < m_nSampleCount; i++)
	{
		// 取得一个采样点
		CPnt& ptSample = m_pSamplePoints[i].GetPntObject();

		// 计算采样点到给定点之间的距离
		float fDist = ptSample.DistanceTo(pt);

		// 更新最近点和最近距离
		if (fDist < fMinDist)
		{
			ptClosestSample = pt;
			fMinDist = fDist;
			nClosest = i;
		}
	}

	// 下面通过插补方法，以更高的精度计算最近点
	CPnt pt1, pt2;
	float t1, t2;

	// 如果最近采样点是第一个采样点
	if (nClosest == 0)
	{
		pt1 = m_pSamplePoints[0];
		pt2 = m_pSamplePoints[1];

		t1 = m_pSamplePoints[0].t;
		t2 = m_pSamplePoints[1].t;
	}
	// 如果最近采样点是最后一个采样点
	else if (nClosest == m_nSampleCount - 1)
	{
		pt1 = m_pSamplePoints[m_nSampleCount - 1];
		pt2 = m_ptKey[m_nCountKeyPoints - 1];

		t1 = m_pSamplePoints[m_nSampleCount - 1].t;
		t2 = 1;
	}

	// 如果最近采样点不在曲线的两端
	else
	{
		// 求得曲线与当前最近采样点相邻的两个采样点及其对应的t值
		pt1 = m_pSamplePoints[nClosest - 1];
		pt2 = m_pSamplePoints[nClosest + 1];

		t1 = m_pSamplePoints[nClosest - 1].t;
		t2 = m_pSamplePoints[nClosest + 1].t;
	}

	// 构造直线
	CLine ln(pt1, pt2);

	// 计算当前姿态到上述直线的投影点
	CPnt ptFoot;
	float fLambda;

	// 计算指定点到短直线的投影点
	ln.DistanceToPoint(false, pt, &fLambda, &ptFoot);

	if (fLambda < 0)
	{
		if (nClosest == 0 && ptFoot.DistanceTo(m_ptKey[0]) < 0.03f)
		{
			if (pptClosest != NULL)
				*pptClosest = m_ptKey[0];

			if (pT != NULL)
				*pT = 0;

			return true;
		}
		else
			return false;
	}
	else if (fLambda > 1)
	{
		if (nClosest == m_nSampleCount - 1 && ptFoot.DistanceTo(m_ptKey[m_nCountKeyPoints - 1]) < 0.03f)
		{
			if (pptClosest != NULL)
				*pptClosest = m_ptKey[m_nCountKeyPoints - 1];

			if (pT != NULL)
				*pT = 1;

			return true;
		}
		else
			return false;
	}
	else
	{
		// 计算投影点到直线两端的距离
		float d1 = ptFoot.DistanceTo(pt1);
		float d2 = ptFoot.DistanceTo(pt2);

		// 得到(修正后的)最近点所对应的t
		float t = (d1 * t2 + d2 * t1) / (d1 + d2);

		// 计算得到修正后的最近点
		SetCurT(t);

		if (pptClosest != NULL)
			*pptClosest = ptFoot;

		if (pT != NULL)
			*pT = t;

		return true;
	}
}

#ifdef _MFC_VER

//
//   从二进制文件装入曲线数据。
//
bool CBezier::Create(CArchive& ar)
{
	int nCountKeyPoints;

#if 0
	// 读入关键点数量
	ar >> nCountKeyPoints;
#else
	nCountKeyPoints = 2;
#endif

	// 临时分配空间
	CPnt* pptKey = new CPnt[nCountKeyPoints];
	if (pptKey == NULL)
		return false;

	// 读入关键点数据
	for (int i = 0; i < nCountKeyPoints; i++)
		ar >> m_ptKey[i].x >> m_ptKey[i].y;

	// 生成曲线
	if (!Create(nCountKeyPoints, pptKey))
		return false;

	// 释放临时空间
	delete []pptKey;

	return true;
}

//
//   将曲线数据保存到二进制文件。
//
bool CBezier::Save(CArchive& ar)
{
	ar << m_nCountKeyPoints;

	for (int i = 0; i < m_nCountKeyPoints; i++)
		ar << m_ptKey[i].x << m_ptKey[i].y;

	return true;
}

//
//   在屏幕上绘制此直线。
//
void CBezier::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPointSize, bool bShowKeyPoints, int nPenStyle)
{
	CPen pen(nPenStyle, nWidth, crColor);
	CPen* pOldPen = pDC->SelectObject(&pen);

	CPoint pntLast;

	for (int i = 0; i < 500; i++)
	{
		float t = i / 500.0f;

		SetCurT(t);
		CPoint pnt = ScrnRef.GetWindowPoint(m_pt);

		if (i != 0)
		{
			pDC->MoveTo(pntLast);
			pDC->LineTo(pnt);
		}
		pntLast = pnt;
	}

	m_ptKey[0].Draw(ScrnRef, pDC, crColor, nPointSize);
	m_ptKey[m_nCountKeyPoints - 1].Draw(ScrnRef, pDC, crColor, nPointSize);

	pDC->SelectObject(pOldPen);
}

//
//   画出控制点。
//
void CBezier::DrawCtrlPoints(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nPointSize)
{
	for (int i = 1; i < m_nCountKeyPoints - 1; i++)
		m_ptKey[i].Draw(ScrnRef, pDC, crColor, nPointSize);
}
#endif
