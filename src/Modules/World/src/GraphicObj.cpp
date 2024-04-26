#include <stdafx.h>
#include "GraphicObj.h"

///////////////////////////////////////////////////////////////////////////////

//
//   唯一选中指定的图形对象，其余对象均不选。
//
void CGraphicObjs::SelectOnly(int index)
{
    Select(false);
    Select(true, index);
}

//
//   向集合中加入一个图形对象。
//
void CGraphicObjs::operator += (const CGraphicObj *obj)
{
    push_back((CGraphicObj*)obj);
}

//
//   向集合中加入另一个图形对象集合。
//
void CGraphicObjs::operator += (const CGraphicObjs &other)
{
    for (int i = 0; i < (int)other.size(); i++)
        *this += other[i];
}

//
//   显示/隐藏指定的对象。
//   说明：
//     如果index>=0，则显示/隐藏指定的那个对象；
//     如果index<0，则显示/隐藏所有对象
//
void CGraphicObjs::SetVisible(bool vis, int index)
{
    if (index >= 0)
        at(index)->SetVisible(vis);
    else
    {
        for (int i = 0; i < (int)size(); i++)
            at(i)->SetVisible(vis);
    }
}

//
//   选中/不选指定的对象。
//   说明：
//     如果index>=0，则选中/不选指定的那个对象；
//     如果index<0，则选中/不选所有对象
//
void CGraphicObjs::Select(bool sel, int index)
{
    if (index >= 0)
        at(index)->Select(sel);
    else
    {
        for (int i = 0; i < (int)size(); i++)
            at(i)->Select(sel);
    }
}

//
//   测试给定的点是否触碰到该图形对象集合中的某一个。
//
int CGraphicObjs::PointHit(const CPnt &pt, float thresh)
{
    for (int i = 0; i < (int)size(); i++)
        if (at(i)->PointHit(pt, thresh))
            return i;

    return -1;
}
#ifdef DesktopRun
//
//   绘制图形对象集合。
//
void CGraphicObjs::Draw(CScreenReference &ScrnRef, QPainter *painter)
{
    for (int i = 0; i < (int)size(); i++)
        at(i)->Draw(ScrnRef, painter);
}

#endif

