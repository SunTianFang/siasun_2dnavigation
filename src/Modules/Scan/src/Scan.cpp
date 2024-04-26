#include "stdafx.h"
#include <math.h>
#include "Scan.h"
#include "CsmMatcher.h"

#ifdef QT_VERSION
#include <QColor>
#include <QPoint>
#include <QPainter>
#endif

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

static CPosture PoseZero;

///////////////////////////////////////////////////////////////////////////////

CScan::CScan()
{
    m_bVisible = true;
}

//
//   构造具有nNum个点的CScan对象。
//
CScan::CScan(int nNum) : CScanPointCloud(nNum)
{
    m_bVisible = true;
}

CScan::CScan(CScanPointCloud *pCloud)
{
    m_bVisible = true;
    Create(pCloud);
}

CScan::~CScan() {}

//
//   重载“=”操作符
//
void CScan::operator=(const CScan &Scan)
{
    *(GetScanPointCloudPointer()) = *(((CScan &)Scan).GetScanPointCloudPointer());

    m_bVisible = Scan.m_bVisible;
    m_pstScanner = Scan.m_pstScanner;
    m_poseRelative = Scan.m_poseRelative;
    m_fStartAng = Scan.m_fStartAng;
    m_fEndAng = Scan.m_fEndAng;

    // 为何不拷贝点特征集合？
}

//
//   清除所有数据。
//
void CScan::Clear()
{
    // 清除点云数据
    CScanPointCloud::Clear();

    // 清除其它所有数据
    // ..
}

//
//   生成一个新的扫描。
//
bool CScan::Create(int nNum)
{
    // 在此应将所有数据成员初始化为0
    Clear();

    // 为点云分配空间
    if (!CScanPointCloud::Create(nNum))
        return false;

    return true;
}

//
//   根据给定点云生成一个新的扫描
//
bool CScan::Create(CScanPointCloud *pCloud)
{
    Clear();

    if (!Create(pCloud->m_nCount))
        return false;

    *(GetScanPointCloudPointer()) = *pCloud;

    return true;
}

//
//   分配空间并复制当前扫描。
//
CScan *CScan::Duplicate()
{
    CScan *scan = new CScan(m_nCount);
    if (scan == NULL)
        return NULL;

    *scan = *this;
    return scan;
}

#if 0
//
//   将全部点移动指定的距离。
//
void CScan::Move(float dx, float dy)
{
	if ((dx == 0) && (dy == 0))
		return;

	// 移动点云数据
	CScanPointCloud::Move(dx, dy);

	// 移动激光头的均值点
    m_pstScanner.Move(dx, dy);
}
#endif

static int spcmp(const void *v1, const void *v2)
{
    const CScanPoint *sp1 = (CScanPoint *)v1, *sp2 = (CScanPoint *)v2;

    /* float da = NormAngle(sp1->a - sp2->a); not a sorting order */
    float da = sp1->a - sp2->a;

    if (da < 0.0)
        return -1;
    else if (da > 0.0)
        return 1;

    return 0;
}
///////////////////////////////////////////////////////////////////////////////
//
//
//   将全部点旋转指定的姿态。
//
void CScan::RotatePos(float angle, float mx, float my)
{
    if (angle != 0.0)
    {
        float sina = sin(angle);
        float cosa = cos(angle);

        CScanPoint *sp = m_pPoints;

        m_pstScanner.fThita += angle;

        float x, y;
        for (int i = 0; i < m_nCount; i++, sp++)
        {
            x = sp->x - mx;
            y = sp->y - my;
            sp->x = mx + cosa * x - sina * y;
            sp->y = my + sina * x + cosa * y;
        }

        x = m_pstScanner.x - mx;
        y = m_pstScanner.y - my;
        m_pstScanner.x = mx + cosa * x - sina * y;
        m_pstScanner.y = my + sina * x + cosa * y;
    }
}

//
//   将全部点进行指定的平移。
//
void CScan::Move(float dx, float dy)
{
    m_pstScanner.Move(dx, dy);
    CScanPointCloud::Move(dx, dy);
}

//
//   将全部点旋转指定的姿态。
//
void CScan::Rotate(float angle)
{
    RotatePos(angle, m_pstScanner.x, m_pstScanner.y);
}

//
//   与给定的参考虑点云进行匹配
//
bool CScan::PointCloudMatch(const CScan &refScan, CFrame &trans, CCorrList &CorrList, sm_result *pResult)
{
    CCsmMatcher CsmMatcher;
    sm_result result;

//    CsmMatcher.setQualityThreshold(0.01, 0.015, 0.10, 0.10, 10, 0.2, 0.2);   // jzz: 设置质量评估阈值参数，测试时更改此处数值

    CCsmScan *pRefScan = new CCsmScan(refScan);
    CCsmScan *pSensScan = new CCsmScan(*this);
/*
    int sCurvatureNum = 0;
    int tCurvatrueNum = 0;
    for (int i = 0; i < pRefScan->m_nRays; i++)
    {
        if (pRefScan->m_pPoints[i].curvature > 1.0)
        {
            tCurvatrueNum++;
        }
    }
    for (int i = 0; i < pSensScan->m_nRays; i++)
    {
        if (pSensScan->m_pPoints[i].curvature > 1.0)
        {
            sCurvatureNum++;
        }
    }
    cout << "sCurvatureNum: " << sCurvatureNum << "\n" << "tCurvatureNum: " << tCurvatrueNum << endl;
    cout << "source num: " << pSensScan->m_nRays << "\n" << "target num: " << pRefScan->m_nRays << endl;
*/
    bool bResult = false;
    if (CsmMatcher.Match(pRefScan, pSensScan, result))
    {
        trans = result.pstMove;

        // 复制匹配表
        CorrList.CreateFromScan(*pSensScan);

        // 如果需要的话，返回匹配结果数据
        if (pResult != NULL)
            *pResult = result;

        bResult = true;
    }
//    cout << "res: " << bResult << " " << "error(p2l): " << result.error << " " << "error2(p2p): "
//         << result.error2 << " " << "nvalid: " << result.nvalid << " " << "valid_percent: " << result.valid_percent
//         << endl;

    delete pRefScan;
    delete pSensScan;

    return bResult;
}

//
//   根据给定的“强反光门限”值，标记哪些点是属于“强反光”。
//
void CScan::MarkReflectivePoints(int nReflectiveGateValue)
{
    for (int i = 0; i < m_nCount; i++)
    {
//        std::cout << "intensity" << i << ": "<< m_pPoints[i].m_nIntensity << std::endl;
        if (m_pPoints[i].r == 0)
            continue;

        m_pPoints[i].m_bHighReflective = (m_pPoints[i].m_nIntensity >= 254/*nReflectiveGateValue + 1700*/); //wt_test_intensity 20221209
    }
}

//
//   启用新的激光扫描器有效范围。
//
void CScan::ApplyNewScannerAngles(const CDataRangeSet &AngRanges)
{
    // 滤除可用角度之外的扫描数据
    for (int i = 0; i < m_nCount; i++)
    {
        float ang = m_pPoints[i].a;
        if (!AngRanges.Contain(ang))
        {
            m_pPoints[i].r = 0;
            m_pPoints[i].x = 0;
            m_pPoints[i].y = 0;
            m_pPoints[i].m_nIntensity = 0;
        }
    }
}

//
//   启用关于扫描角度范围的约束(处于给定角度范围之外的部分将被滤除)。
//
void CScan::ApplyScanAngleRule(float fMinAngle, float fMaxAngle)
{
    CDataRange range(fMinAngle, fMaxAngle);
    for (int i = 0; i < m_nCount; i++)
    {
        float ang = m_pPoints[i].a;
        if (!range.Contain(ang))
        {
            m_pPoints[i].r = 0;
            m_pPoints[i].x = 0;
            m_pPoints[i].y = 0;
            m_pPoints[i].m_nIntensity = 0;
        }
    }
}

//
//   启用关于扫描距离的约束(超出给定距离范围之外的部分将被滤除)。
//
void CScan::ApplyScanDistRule(float fMinDist, float fMaxDist)
{
    for (int i = 0; i < m_nCount; i++)
    {
        float r = m_pPoints[i].r / 1000;
        if (r < fMinDist || r > fMaxDist)
        {
            m_pPoints[i].r = 0;
            m_pPoints[i].x = 0;
            m_pPoints[i].y = 0;
            m_pPoints[i].m_nIntensity = 0;
        }
    }
}

//
//   从二进制文件中读取扫描数据。
//
bool CScan::LoadBinary(FILE *fp, const CPosture &pstRobot, const CLaserScannerParam &Param, int nFileVersion)
{
    // 先计算激光器的姿态
    CFrame frame(pstRobot);
    m_pstScanner.SetPosture(Param.m_pst);
    m_pstScanner.InvTransform(frame);

    m_poseRelative = m_pstScanner;
    m_fStartAng = Param.m_fStartAngle;
    m_fEndAng = Param.m_fEndAngle;

    if (!CScanPointCloud::LoadBinary(fp, Param.m_fStartAngle, Param.m_fEndAngle, Param.m_fMinRange, Param.m_fMaxRange, Param.m_nLineCount,
                                     nFileVersion))
        return false;

    return true;
}

//
//   将扫描数据保存到十进制文件。
//
bool CScan::SaveBinary(FILE *fp, int nFileVersion)
{
    if (!CScanPointCloud::SaveBinary(fp, nFileVersion))
        return false;

    return true;
}

//
//   对整个点云进行坐标系变换。
//
void CScan::Transform(const CFrame &frame)
{
    CScanPointCloud::Transform(frame);
    m_pstScanner.Transform(frame);
}

//
//   对整个点云进行坐标系逆变换。
//
void CScan::InvTransform(const CFrame &frame)
{
    CScanPointCloud::InvTransform(frame);
    m_pstScanner.InvTransform(frame);
}

//
//   施加虚拟环境。
//
void CScan::ApplyVirtObjects(const CLiveObjects &objs, int stepId)
{
    CPnt pt;
    CPosture pstScanner = m_pstScanner;
    float fDist;

    for (int i = 0; i < m_nCount; i++)
    {
        CLine ray;

        // 如果当前激光射线长度为0，则启用最大长度
        if (m_pPoints[i].r == 0)
        {
            CAngle ang = pstScanner.GetAngle() + m_pPoints[i].a;
            ray.Create(pstScanner, ang, 30.0f);
        }
        // 激光射线长度为非零值，则采用实际长度
        else
            ray.Create(pstScanner, m_pPoints[i]);

        if (objs.LineHit(ray, pt, fDist))
        {
            m_pPoints[i].GetPntObject() = pt;
            m_pPoints[i].m_nIntensity = 0;
            m_pPoints[i].m_bHighReflective = false;
        }
    }
}

#define ARROW_LEN 20    // 20个像素

#ifdef _MFC_VER
void CScan::Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColorPoint, COLORREF crHighLightPoint, bool bShowScanner,
                 COLORREF crScanner, int nPointSize)
{
    // 取得扫描时激光头的姿态
    CPosture &pstRef = m_pstScanner.GetPostureObject();

    CPen pen(PS_SOLID, 1, crColorPoint);
    CPen *pOldPen = pDC->SelectObject(&pen);

    CBrush Brush(crColorPoint);
    CBrush *pOldBrush = pDC->SelectObject(&Brush);

    CBrush HighLightBrush(crHighLightPoint);

    CPnt pt;

    for (int i = 0; i < m_nCount; i++)
    {
        if (m_pPoints[i].m_bDelete)
            continue;

        if (!m_pPoints[i].m_bHighReflective)
            pDC->SelectObject(&Brush);
        else
            pDC->SelectObject(&HighLightBrush);

        CPnt pt;

        // 如果需要在世界坐标系中显示，现在需要进行坐标变换
        pt = m_pPoints[i];

        CPoint pnt = ScrnRef.GetWindowPoint(pt);

        int nSize = nPointSize;
        if (m_pPoints[i].m_bDelete)
            nSize /= 2;

        // 在此，支持“微距观察模式”(即在放大倍数很大的情况下，每个点也放大显示)
        if (nSize == 1)
        {
            if (ScrnRef.m_fRatio > 500)
                nSize = 1 * ScrnRef.m_fRatio / 500;
        }

        CRect r(pnt.x - nSize, pnt.y - nSize, pnt.x + nSize, pnt.y + nSize);
        pDC->Ellipse(&r);
    }
    pDC->SelectObject(pOldPen);
    pDC->SelectObject(pOldBrush);

    if (bShowScanner)
        m_pstScanner.Draw(ScrnRef, pDC, crScanner, crScanner, 40, 150, 1);
}

#elif defined QT_VERSION

void CScan::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor, QColor crHighLightPoint, bool bShowScanner,
                 QColor crScanner, int nPointSize)
{
    if (!m_bVisible)
        return;

    QPen pen(crColor);
    pen.setWidth(1);
    pPainter->setPen(pen);

    QBrush brush(crColor);
    QBrush highLightBrush(crHighLightPoint);

    CPnt pt;

    for (int i = 0; i < m_nCount; i++)
    {
        if (m_pPoints[i].m_bDelete)
            continue;

        if (!m_pPoints[i].m_bHighReflective)
            pPainter->setBrush(brush);
        else
            pPainter->setBrush(highLightBrush);

        CPnt pt;

        // 如果需要在世界坐标系中显示，现在需要进行坐标变换
        pt = m_pPoints[i];

        QPoint pnt = ScrnRef.GetWindowPoint(pt);

        int nSize = nPointSize;
        if (m_pPoints[i].m_bDelete)
            nSize /= 2;

        // 在此，支持“微距观察模式”(即在放大倍数很大的情况下，每个点也放大显示)
        float fSize = (float)(nSize);
        if (nSize == 1)
        {
            if (ScrnRef.m_fRatio > 500)
                fSize = 1 * ScrnRef.m_fRatio / 500;
        }

        int left = (int)(pnt.x() - fSize);
        int top = (int)(pnt.y() - fSize);

        QRect r(left, top, (int)(2 * fSize), (int)(2 * fSize));

        if (fSize <= 1)
            pPainter->drawPoint(pnt);
        else
            pPainter->drawEllipse(r);
    }

    if (bShowScanner)
        m_pstScanner.Draw(ScrnRef, pPainter, crScanner, crScanner, 30, 100);
}

#endif
