#pragma once

#include "GraphicObj.h"
#include "StaticObjects.h"

///////////////////////////////////////////////////////////////////////////////
// 定义单个的静态物体。
class DllExport CStaticObjectEditable : public CStaticObject, public CGraphicObj
{
  public:
    CStaticObjectEditable() {}

    // 生成一个副本
    virtual CStaticObject *Duplicate();

    // 判断指定的点是否触碰到该物体
    virtual bool PointHit(const CPnt &pt, float fDistGate, bool &hitRef);

#ifdef _MFC_VER
    void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nLineWidth);
#elif defined QT_VERSION
    void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor, int nLineWidth);
#endif
};

///////////////////////////////////////////////////////////////////////////////
// 定义多个静态物体。
class DllExport CStaticObjectsEditable : public CStaticObjects
{
  public:
    // 生成新的静态物体
    virtual CStaticObject *NewStaticObject();

    // 选中/不选指定的物体
    void Select(bool sel, int index = -1);

    // 判断指定的物体是否被选中
    bool IsSelected(int index);

    // 取得被选中物体的数量
    int GetNumSelected();

    // 删除所有选中的物体
    void DeleteAllSelected();

    // 判断指定的点是否触碰到该物体
    virtual int PointHit(const CPnt &pt, float fDistGate, bool &hitRef);

#ifdef _MFC_VER
    void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nLineWidth, );

    // 在屏幕上绘制此集合中所有被选中者
    void PlotSelected(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nLineWidth);

#elif defined QT_VERSION
    void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor, int nLineWidth);

    // 在屏幕上绘制此集合中所有被选中者
    void PlotSelected(CScreenReference &ScrnRef, QPainter *pPainter, QColor cr, int nLineWidth);
#endif
};
