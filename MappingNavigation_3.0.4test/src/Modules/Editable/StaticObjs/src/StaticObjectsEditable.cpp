#include <stdafx.h>
#include "StaticObjectsEditable.h"

///////////////////////////////////////////////////////////////////////////////

//
//   生成一个副本。
//
CStaticObject *CStaticObjectEditable::Duplicate()
{
    CStaticObjectEditable *obj = new CStaticObjectEditable;
    obj->GetGraphicObj() = GetGraphicObj();
    obj->GetStaticObject() = GetStaticObject();
    return obj;
}

//
//   判断指定的点是否触碰到该物体。
//
bool CStaticObjectEditable::PointHit(const CPnt &pt, float fDistGate, bool &hitRef)
{
    hitRef = false;

    if (m_pst.DistanceTo(pt) < fDistGate)
    {
        hitRef = true;    // 触碰到参考姿态
        return true;
    }

    for (int i = 0; i < (int)size(); i++)
        if (at(i)->PointHit(pt, fDistGate))
            return true;    // 触碰到物体边框

    return false;
}

#ifdef _MFC_VER

void CStaticObjectEditable::Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF clr, int nLineWidth)
{
    // 画出物体形状
    CBasObject::Plot(ScrnRef, pDC, clr, nLineWidth);

    // 画出参考姿态
    m_pst.Draw(ScrnRef, pDC, clr, clr, 20, 50);
}

#elif defined QT_VERSION
void CStaticObjectEditable::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr, int nLineWidth)
{
    // 画出物体形状
    CBasObject::Plot(ScrnRef, pPainter, clr, nLineWidth);

    // 画出参考姿态
    m_pst.Draw(ScrnRef, pPainter, clr, clr, 20, 50);
}
#endif

///////////////////////////////////////////////////////////////////////////////
//   “CStaticObjectsEditable”类的实现。

//
//   生成新的静态物体。
//
CStaticObject *CStaticObjectsEditable::NewStaticObject()
{
    return new CStaticObjectEditable;
}

//
//   选中/不选指定的物体。
//
void CStaticObjectsEditable::Select(bool sel, int index)
{
    if (index >= (int)size())
        return;

    // 如果index非负，则仅对一个物体进行选中/不选操作
    if (index >= 0)
    {
        CStaticObjectEditable *obj = dynamic_cast<CStaticObjectEditable *>(at(index));
        if (obj != NULL)
            obj->Select(sel);
    }

    // 如果index<0，则对所有物体进行选中/不选操作
    else
    {
        for (int i = 0; i < (int)size(); i++)
        {
            CStaticObjectEditable *obj = dynamic_cast<CStaticObjectEditable *>(at(i));
            if (obj != NULL)
                obj->Select(sel);
        }
    }
}

//
//   判断指定的物体是否被选中。
//
bool CStaticObjectsEditable::IsSelected(int index)
{
    CStaticObjectEditable *obj = dynamic_cast<CStaticObjectEditable *>(at(index));
    if (obj == NULL)
        return false;

    return obj->IsSelected();
}

//
//   取得被选中物体的数量。
//
int CStaticObjectsEditable::GetNumSelected()
{
    int count = 0;
    for (int i = 0; i < (int)size(); i++)
        if (IsSelected(i))
            count++;

    return count;
}

//
//   删除所有选中的物体。
//
void CStaticObjectsEditable::DeleteAllSelected()
{
    for (int i = (int)size() - 1; i >= 0; i--)
        if (IsSelected(i))
            Delete(i);
}

//
//   判断指定的点是否触碰到该物体。
//
int CStaticObjectsEditable::PointHit(const CPnt &pt, float fDistGate, bool &hitRef)
{
    hitRef = false;
    for (int i = 0; i < (int)size(); i++)
    {
        CStaticObjectEditable *obj = dynamic_cast<CStaticObjectEditable *>(at(i));
        if (obj == NULL)
            continue;

        if (obj->PointHit(pt, fDistGate, hitRef))
            return i;
    }

    return -1;
}

#ifdef _MFC_VER
void CStaticObjectsEditable::Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF clr, int nLineWidth, int stepId)
{
    for (int i = 0; i < (int)size(); i++)
        at(i).Plot(ScrnRef, pDC, clr, nLineWidth);
}

#elif defined QT_VERSION
void CStaticObjectsEditable::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr, int nLineWidth)
{
    for (int i = 0; i < (int)size(); i++)
        at(i)->Plot(ScrnRef, pPainter, clr, nLineWidth);
}

//
//   在屏幕上绘制此集合中所有被选中者。
//
void CStaticObjectsEditable::PlotSelected(CScreenReference &ScrnRef, QPainter *pPainter, QColor cr, int nLineWidth)
{
    for (int i = 0; i < (int)size(); i++)
    {
        if (!IsSelected(i))
            continue;

        at(i)->Plot(ScrnRef, pPainter, cr, nLineWidth);
    }
}
#endif
