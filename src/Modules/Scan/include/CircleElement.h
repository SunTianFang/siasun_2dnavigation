#pragma once

#include "ObjectElement.h"

#define ELEMENT_TYPE_CIRCLE 2

///////////////////////////////////////////////////////////////////////////////
// 定义圆形对象
class DllExport CCircleElement : public CObjectElement
{
  public:
    CCircle m_circle;

  public:
    CCircleElement(const CCircle &circle)
    {
        m_nType = ELEMENT_TYPE_CIRCLE;
        m_circle = circle;
    }

    CCircleElement() { m_nType = ELEMENT_TYPE_CIRCLE; }

    // 生成直线对象
    bool Create(const CCircle &circle);

    // 复制对象并返回指针
    virtual CObjectElement *Duplicate() const;

    // 判断指定的点是否触碰到该圆形元素
    virtual bool PointHit(const CPnt &pt, float fDistGate);

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
