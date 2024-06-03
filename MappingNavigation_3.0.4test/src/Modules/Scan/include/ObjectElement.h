#pragma once

#include <vector>

#include "Geometry.h"
#include "Frame.h"

#ifdef QT_VERSION
#include <QColor>
#include <QPainter>
#endif

void AdjustLine(CLine &line, const CPosture &pstScanner, float &startAngle, float &endAngle);

///////////////////////////////////////////////////////////////////////////////
// 定义“物体元素”，可包括直线和圆弧
class DllExport CObjectElement
{
  public:
    int m_nType;

  public:
    CObjectElement() { m_nType = 0; }

    // 复制对象并返回指针
    virtual CObjectElement *Duplicate() const = 0;

    // 判断指定的点是否触碰到该直线元素
    virtual bool PointHit(const CPnt &pt, float fDistGate) = 0;

    // 判断物体元素是否与给定的直线相交
    virtual bool LineHit(const CLine &ray, CPnt &pt, float &dist) const = 0;

    // 进行坐标正变换
    virtual void Transform(const CFrame &frame) = 0;

    // 进行坐标逆变换
    virtual void InvTransform(const CFrame &frame) = 0;

    // 从文本文件中装入数据
    virtual bool LoadText(FILE *fp) = 0;

    // 将数据保存到文本文件中
    virtual bool SaveText(FILE *fp) = 0;

    // 从二进制文件中装入数据
    virtual bool LoadBinary(FILE *fp) = 0;

    // 将数据保存到二进制文件中
    virtual bool SaveBinary(FILE *fp) = 0;

#ifdef _MFC_VER
    virtual void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor,
                      int nLineWidth = 1) = 0;
#elif defined QT_VERSION
    virtual void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor,
                      int nLineWidth = 1) = 0;
#endif
};

typedef std::vector<CObjectElement *> CObjectElements;
