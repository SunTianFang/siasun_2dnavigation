#include <stdafx.h>
#include "LineElement.h"
#include "MultiSegLine.h"

///////////////////////////////////////////////////////////////////////////////
//   一些辅助函数。

//
//   对给定的直线line进行调整，使得当以给定的姿态pstScanner进行观察它时，总时先扫描到它的起点，后扫描到它的终点。
//   并且，将所得到的扫描角分别保存到startAngle和endAngle。
//
void AdjustLine(CLine &line, const CPosture &pstScanner, float &startAngle, float &endAngle)
{
    // 以扫描器所在点为基点，构造第一条线段的起/止扫描线
    CLine startScanLine(pstScanner, line.m_ptStart);
    CLine endScanLine(pstScanner, line.m_ptEnd);

    CAngle angDiff = endScanLine.SlantAngle() - startScanLine.SlantAngle();

    // 如果起止扫描线间的夹角超过180度，则说明假定的扫描方向是错的，需要将直线段反向
    if (angDiff.NormAngle() > PI)
        line.Reverse();

    // 重新构造两条扫描线
    startScanLine.Create(pstScanner, line.m_ptStart);
    endScanLine.Create(pstScanner, line.m_ptEnd);

    // 保存起始扫描角和终止扫描角
    startAngle = startScanLine.SlantAngle().NormAngle2();
    endAngle = endScanLine.SlantAngle().NormAngle2();
}

//
//   假定两条直线段line1，line2相对于观测位姿pstScanner都是形成逆时针方向的扫描角，且line1的
//   扫描角完全覆盖line2的扫描角。此函数将生成多段线mline1和mline2，前者是line1扫描所见线段，而后者则是
//   line2扫描所见线段
//
void CreateLineSegments_Case1(const CLine line1, const CLine &line2, const CPosture &pstScanner,
                              CMultiSegLine &mline1, CMultiSegLine &mline2)
{
    mline1 = line1;
    mline2 = line2;

    // 构造对应于line2的起点、终点的长扫描线(1000米长)，以计算它们与line1的交点
    CLine line3(pstScanner, line2.m_ptStart);
    line3.SetLength(1000);

    CLine line4(pstScanner, line2.m_ptEnd);
    line4.SetLength(1000);

    CPnt pt1, pt2;           // 扫描line3、line4到line1交点
    float dist11, dist12;    // 扫描line3、line4到line1的扫描距离

    // 计算两条扫描线与line1的相交情况，得到交点(pt1, pt2)和距离(dist11, dist12)
    bool intersect1 = line3.IntersectLineAt(line1, pt1, dist11);
    bool intersect2 = line4.IntersectLineAt(line1, pt2, dist12);

    // 计算扫描器到line2两个端点的距离
    float dist21 = pstScanner.DistanceTo(line2.m_ptStart);
    float dist22 = pstScanner.DistanceTo(line2.m_ptEnd);

    // 如果line1落在line2之后
    if (dist11 - dist21 > -0.001f && dist12 - dist22 > -0.001f)
    {
        // line1被挡住了中间一部分，line2不变
        float hideFrom = line1.m_ptStart.DistanceTo(pt1);
        float hideTo = line1.m_ptStart.DistanceTo(pt2);

        CDataRange hideRange(hideFrom, hideTo);
        mline1 -= hideRange;
    }

    // 如果line2落在line1之后
    else if (dist11 - dist21 < 0.001f && dist12 - dist22 < 0.001f)
    {
        // line2被完全挡住, line1保持不变
        mline2.clear();
    }

    // 否则，line1、line2有交叉
    else
    {
        // 计算交点
        CPnt pt;
        float d;
        line1.IntersectLineAt(line2, pt, d);

        // 如果line1的前段比line2的前段距离扫描器更近
        if (dist11 - dist21 < 0.001f && dist12 - dist22 > -0.001f)
        {
            // line1中间有一段被挡住
            float hideFrom = line1.m_ptStart.DistanceTo(pt);
            float hideTo = line1.m_ptStart.DistanceTo(pt2);
            mline1 -= CDataRange(hideFrom, hideTo);

            // line2的前一段被挡住
            hideTo = line2.m_ptStart.DistanceTo(pt);
            mline2 -= CDataRange(0, hideTo);
        }
        // 如果line2的前段比line1的前段距离扫描器更近
        else
        {
            // line1中间有一段被挡住
            float hideFrom = line1.m_ptStart.DistanceTo(pt1);
            float hideTo = line1.m_ptStart.DistanceTo(pt);
            mline1 -= CDataRange(hideFrom, hideTo);

            // line2的后面有一段被挡住
            hideFrom = line2.m_ptStart.DistanceTo(pt);
            hideTo = line2.Length();
            mline2 -= CDataRange(hideFrom, hideTo);
        }
    }
}

//
//   假定两条直线段line1，line2相对于观测位姿pstScanner都是形成逆时针方向的扫描角，且line1的扫描角的后一部分与
//   line2扫描角的前一部分重合，此函数将生成多段线mline1和mline2，前者是line1扫描所见线段，而后者则是line2扫描所见线段
//
void CreateLineSegments_Case2(const CLine &line1, const CLine &line2, const CPosture &pstScanner,
                              CMultiSegLine &mline1, CMultiSegLine &mline2)
{
    float hideFrom, hideTo;

    mline1 = line1;
    mline2 = line2;

    // 构造对应于line2起点的长度为1000米长的扫描线
    CLine line3(pstScanner, line2.m_ptStart);
    line3.SetLength(1000);

    // 构造对应于line1终点的长度为1000米长的扫描线
    CLine line4(pstScanner, line1.m_ptEnd);
    line4.SetLength(1000);

    CPnt pt1;                // 扫描线line3到line1交点
    CPnt pt2;                // 扫描线line4到line1的交点
    float dist11, dist22;    // 扫描线line3、line4到line1的扫描距离

    // 计算两条扫描线与line1的相交情况，得到交点(pt1, pt2)和距离(dist11, dist12)
    bool intersect1 = line3.IntersectLineAt(line1, pt1, dist11);
    float dist12 = pstScanner.DistanceTo(line1.m_ptEnd);
    float dist21 = pstScanner.DistanceTo(line2.m_ptStart);
    bool intersect2 = line4.IntersectLineAt(line2, pt2, dist22);

    // 如果line1落在line2之后
    if (dist11 - dist21 > -0.001f && dist12 - dist22 > -0.001f)
    {
        // line1的后一段被截去，line2保持不变
        hideFrom = line1.m_ptStart.DistanceTo(pt1);
        hideTo = line1.Length();
        mline1 -= CDataRange(hideFrom, hideTo);
    }

    // 如果line2落在line1之后
    else if (dist11 - dist21 < 0.001f && dist12 - dist22 < 0.001f)
    {
        // line1保持不变，line2的前一段被挡住
        hideTo = line2.m_ptStart.DistanceTo(pt2);
        mline2 -= CDataRange(0, hideTo);
    }

    // 否则，line1、line2有交叉
    else
    {
        // 计算交点
        CPnt pt;
        float d;
        line1.IntersectLineAt(line2, pt, d);

        // 如果line1的前段比line2的前段距离扫描器更近
        if (dist11 - dist21 < 0.001f && dist12 - dist22 > -0.001f)
        {
            // line1的后一段被挡住
            hideFrom = line1.m_ptStart.DistanceTo(pt);
            hideTo = line1.Length();
            mline1 -= CDataRange(hideFrom, hideTo);

            // line2的前一段被挡住
            hideTo = line2.m_ptStart.DistanceTo(pt);
            mline2 -= CDataRange(0, hideTo);
        }
        // 如果line2的前段比line1的前段距离扫描器更近
        else
        {
            // line1的中间一段被挡住
            hideFrom = line1.m_ptStart.DistanceTo(pt1);
            hideTo = line1.m_ptStart.DistanceTo(pt);
            mline1 -= CDataRange(hideFrom, hideTo);

            // line2的中间一段被挡住
            hideFrom = line2.m_ptStart.DistanceTo(pt);
            hideTo = line2.m_ptStart.DistanceTo(pt2);
            mline2 -= CDataRange(hideFrom, hideTo);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
//   “CLineElement”类的实现。

//
//   复制对象并返回指针。
//
CObjectElement *CLineElement::Duplicate() const
{
    CLineElement *lineEle = new CLineElement(m_line);
    return lineEle;
}

//
//   判断指定的点是否触碰到该直线元素。
//
bool CLineElement::PointHit(const CPnt &pt, float fDistGate)
{
    return m_line.PointHit(pt, fDistGate);
}

//
//   以指定的扫描姿态，生成对两个直线元素的扫描分段结果。
//
void CLineElement::ApplyObstacle(const CLineElement &obstacle, const CPosture &pstScanner,
                                 CObjectElements &remainThis, CMultiSegLine &obstacleLine)
{
    CLine line1 = m_line;
    CLine line2 = obstacle.m_line;

    CMultiSegLine mline1;

    // 调整两条直线，使得它们从扫描器看到的扫描角都是从起点到终点的方向
    float ang[4];
    AdjustLine(line1, pstScanner, ang[0], ang[1]);
    AdjustLine(line2, pstScanner, ang[2], ang[3]);

    // 记录第一条直线的起始扫描角，后面有用
    float ang0 = ang[0];

    // 四个扫描角都减去第一条线段的扫描角(相当于都进行了角度旋转，使第一条线段的起始扫描角成为0度)
    for (int i = 0; i < 4; i++)
    {
        ang[i] -= ang0;
        ang[i] = NormAngle2(ang[i]);
    }

    // 如果line2起始扫描角小于0，说明line2的起始扫描角在line1起始扫描角的上游
    if (ang[2] <= 0)
    {
        // 如果line2的终止扫描角也在line1起始扫描角的上游，说明两条线段的扫描角无重合段
        if (ang[3] < 0)
        {
            mline1 = line1;
            obstacleLine = line2;
        }

        // 如果line2的起角早于line1起角，终角又大于line1的终角，说明line2扫描角完全覆盖line1的扫描角
        else if (ang[3] >= ang[1])
            CreateLineSegments_Case1(line2, line1, pstScanner, obstacleLine, mline1);

        // 否则，line2的终角落在line1的两个扫描角之间，两个夹角部分重合
        else
            CreateLineSegments_Case2(line2, line1, pstScanner, obstacleLine, mline1);
    }

    // 如果line2的起始扫描角落在line1扫描夹角当中
    else if (ang[2] < ang[1])
    {
        // 如果line2的终角也落在line1扫描角当中，说明line2的扫描角被line1的扫描角完全包含
        if (ang[3] <= ang[1])
            CreateLineSegments_Case1(line1, line2, pstScanner, mline1, obstacleLine);
        // 否则，line2的终角落在line1扫描角范围之外，说明两个扫描角部分重合
        else
            CreateLineSegments_Case2(line1, line2, pstScanner, mline1, obstacleLine);
    }

    // 如果line2的起角大于line1的终止角，说明line1、line2没有重合段
    else
    {
        mline1 = line1;
        obstacleLine = line2;
    }

    // 最后，将结果保存到remainThis和remainObstacle中
    remainThis.clear();
    for (int i = 0; i < (int)mline1.size(); i++)
    {
        CLine line;
        if (mline1.GetSegment(i, line))
            remainThis.push_back(new CLineElement(line));
    }
}

//
//   判断物体元素是否与给定的直线相交。
//
bool CLineElement::LineHit(const CLine &ray, CPnt &pt, float &dist) const
{
    if (m_line.Length() < 0.01f || ray.Length() < 0.01f)
        return false;
    else
        return ray.IntersectLineAt(m_line, pt, dist);
}

//
//   进行坐标正变换。
//
void CLineElement::Transform(const CFrame &frame)
{
    m_line.Transform(frame);
}

//
//   进行坐标逆变换。
//
void CLineElement::InvTransform(const CFrame &frame)
{
    m_line.InvTransform(frame);
}

//
//   从文本文件中装入数据。
//
bool CLineElement::LoadText(FILE *fp)
{
    return m_line.LoadText(fp);
}

//
//   将数据保存到文本文件中。
//
bool CLineElement::SaveText(FILE *fp)
{
    return m_line.SaveText(fp);
}

//
//   从二进制文件中装入数据。
//
bool CLineElement::LoadBinary(FILE *fp)
{
    return m_line.LoadBinary(fp);
}

//
//   将数据保存到二进制文件中。
//
bool CLineElement::SaveBinary(FILE *fp)
{
    return m_line.SaveBinary(fp);
}

#ifdef _MFC_VER
void CLineElement::Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nLineWidth)
{
    m_line.Draw(ScrnRef, pDC, crColor, nLineWidth);
}

#elif defined QT_VERSION
void CLineElement::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor,
                        int nLineWidth)
{
    m_line.Draw(ScrnRef, pPainter, crColor, nLineWidth);
}
#endif
