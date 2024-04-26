#pragma once

#include <vector>
#include "ObjectElement.h"
#include "MultiSegLine.h"

#define ELEMENT_TYPE_LINE 1

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// 定义直线对象
class DllExport CLineElement : public CObjectElement
{
  public:
    CLine m_line;

  public:
    CLineElement(const CLine &line)
    {
        m_nType = ELEMENT_TYPE_LINE;
        m_line = line;
    }

    CLineElement(const CPnt &ptStart, const CPnt &ptEnd)
    {
        m_nType = ELEMENT_TYPE_LINE;
        m_line.Create(ptStart, ptEnd);
    }

    CLineElement() { m_nType = ELEMENT_TYPE_LINE; }

    // 生成直线对象
    bool Create(const CLine &line);

    // 复制对象并返回指针
    virtual CObjectElement *Duplicate() const;

    // 判断指定的点是否触碰到该直线元素
    virtual bool PointHit(const CPnt &pt, float fDistGate);

    // 以指定的扫描姿态，生成对两个直线元素的扫描分段结果
    void ApplyObstacle(const CLineElement &obstacle, const CPosture &pstScanner,
                       CObjectElements &remThis, CMultiSegLine &obstacleLine);

    // 判断物体元素是否与给定的直线相交
    virtual bool LineHit(const CLine &ray, CPnt &pt, float &dist) const;

    // 进行坐标正变换
    virtual void Transform(const CFrame &frame);

    // 进行坐标逆变换
    virtual void InvTransform(const CFrame &frame);

    // 从文本文件中装入数据
    virtual bool LoadText(FILE *fp);

    // 将数据保存到文本文件中
    virtual bool SaveText(FILE *fp);

    // 从二进制文件中装入数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据保存到二进制文件中
    virtual bool SaveBinary(FILE *fp);

#ifdef _MFC_VER
    virtual void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nLineWidth = 1);
#elif defined QT_VERSION
    virtual void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor,
                      int nLineWidth = 1);
#endif
};

