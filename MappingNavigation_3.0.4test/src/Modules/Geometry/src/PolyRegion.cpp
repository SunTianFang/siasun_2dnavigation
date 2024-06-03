//                          - POLYREGION.CPP -
//
//   Implementation of class "CLine" - which defines the geometric concept
//   "Directional Straight Line".
//
//   Author: Zhang Lei
//   Date:   2000. 4. 24
//

#include "stdafx.h"
#include <math.h>
//#include "Tools.h"
#include "Geometry.h"
#include "ScrnRef.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

CPolyRegion::CPolyRegion(int nCount, CPnt *pPnt)
{
    m_pVertex = NULL;
    Create(nCount, pPnt);
}

CPolyRegion::~CPolyRegion()
{
    Clear();
}

//
//   根据参数生成对象。
//
bool CPolyRegion::Create(int nCount, CPnt *pPnt)
{
    Clear();

    m_pVertex = new CPnt[nCount];
    if (m_pVertex == NULL)
        return false;

    m_nCount = nCount;

    if (pPnt == NULL)
        return true;

    for (int i = 0; i < nCount; i++)
        m_pVertex[i] = pPnt[i];

    return true;
}

void CPolyRegion::Clear()
{
    if (m_pVertex != NULL)
        delete[] m_pVertex;
    m_pVertex = NULL;
}

//
//   判断指定的点是否包含在此多边形区域中。
//
bool CPolyRegion::Contain(const CPnt &pt) const
{
    int i, j;
    bool bResult = false;

    for (i = 0, j = m_nCount - 1; i < m_nCount; j = i++)
    {
        if ((m_pVertex[i].y < pt.y && m_pVertex[j].y >= pt.y || m_pVertex[j].y < pt.y && m_pVertex[i].y >= pt.y) &&
            (m_pVertex[i].x <= pt.x || m_pVertex[j].x <= pt.x))
        {
            if (m_pVertex[i].x +
                    (pt.y - m_pVertex[i].y) / (m_pVertex[j].y - m_pVertex[i].y) * (m_pVertex[j].x - m_pVertex[i].x) <
                pt.x)
                bResult = !bResult;
        }
    }

    return bResult;
}

//
//   判断此区域是否包含另外一个指定的区域。
//
bool CPolyRegion::Contain(const CPolyRegion &PolyRgn) const
{
    for (int i = 0; i < PolyRgn.m_nCount; i++)
        if (!Contain(PolyRgn.m_pVertex[i]))
            return false;

    return true;
}

//
//   判断此区域是否与另外一个指定的区域有重叠区。
//
bool CPolyRegion::OverlapWith(const CPolyRegion &PolyRgn) const
{
    int i;

    for (i = 0; i < PolyRgn.m_nCount; i++)
        if (Contain(PolyRgn.m_pVertex[i]))
            return true;

    for (i = 0; i < m_nCount; i++)
        if (PolyRgn.Contain(m_pVertex[i]))
            return true;

    return false;
}

//
//   判断一个给定的点是否“触碰”到该多边形。
//
bool CPolyRegion::PointHit(const CPnt &pt, float fDistGate)
{
    // 测试一下给定点是否触碰到矩形的某条边
    for (int i = 0; i < m_nCount; i++)
    {
        CLine ln(m_pVertex[i], m_pVertex[(i + 1) % m_nCount]);
        if (ln.PointHit(pt, fDistGate))
            return true;
    }

    return false;
}

//
//   从文本文件装入数据。
//
bool CPolyRegion::LoadText(FILE *fp)
{
    int count;
    if (fscanf(fp, "%d\t", &count) != 1)
        return false;

    CPnt *p = new CPnt[count];
    if (p == NULL)
        return false;

    // 依次读入各点
    for (int i = 0; i < count; i++)
        if (fscanf(fp, "%f\t%f\t", &p[i].x, &p[i].y) != 2)
            return false;

    // 生成数据
    if (!Create(count, p))
        return false;

    delete[] p;
    return true;
}

//
//   将数据写入文本文件。
//
bool CPolyRegion::SaveText(FILE *fp)
{
    fprintf(fp, "%d\t", m_nCount);
    for (int i = 0; i < m_nCount; i++)
        fprintf(fp, "%f\t%f\t", m_pVertex[i].x, m_pVertex[i].y);

    fprintf(fp, "\n");
    return true;
}

//
//   从二进制文件装入数据。
//
bool CPolyRegion::LoadBinary(FILE *fp)
{
    int count;
    if (fread(&count, sizeof(int), 1, fp) != 1)
        return false;

    CPnt *p = new CPnt[count];
    if (p == NULL)
        return false;

    // 依次读入各点
    for (int i = 0; i < count; i++)
    {
        float f[2];
        if (fread(f, sizeof(float), 2, fp) != 2)
            return false;

        p[i].x = f[0];
        p[i].y = f[1];
    }

    // 生成数据
    if (!Create(count, p))
        return false;

    delete[] p;
    return true;
}

//
//   将数据写入二进制文件。
//
bool CPolyRegion::SaveBinary(FILE *fp)
{
    if (fwrite(&m_nCount, sizeof(int), 1, fp) != 1)
        return false;

    // 依次写入各点
    for (int i = 0; i < m_nCount; i++)
    {
        float f[2] = {m_pVertex[i].x, m_pVertex[i].y};
        if (fwrite(f, sizeof(float), 2, fp) != 2)
            return false;
    }

    return true;
}

#ifdef _MFC_VER

//
//   在屏幕上绘制此多边形区域。
//
void CPolyRegion::Draw(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nWidth, int nPointSize,
                       bool bBigVertex)
{
}
#endif
