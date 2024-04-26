#include <stdafx.h>
#include "AppArea.h"

#ifdef QT_VERSION
#include <QPainter>
#include <QColor>
#endif

///////////////////////////////////////////////////////////////////////////////

CAppArea::~CAppArea()
{
    Clear();
}

// 清除全部区域
void CAppArea::Clear()
{
    for (int i = 0; i < (int)size(); i++)
        if (at(i) != NULL)
            delete at(i);

    clear();
}

//
//   重载操作符"+="。
//
CAppArea &CAppArea::operator+=(const CRectangle &r)
{
    push_back(new CRectangle(r));
    return *this;
}

//
//   重载操作符"+="。
//   注意：此方法直接加入所提供的指针，并不复制数据。
//
CAppArea &CAppArea::operator+=(const CRectangle *p)
{
    push_back((CRectangle*)p);
    return *this;
}

//
//   判断一个给定点是否处于应用范围内。
//
int CAppArea::Contain(const CPnt &pt) const
{
    for (int i = 0; i < (int)size(); i++)
        if (at(i)->Contain(pt))
            return i;

    return -1;
}

//
//   更改指定的矩形区域。
//
bool CAppArea::Update(int index, const CRectangle &r)
{
    if (index >= (int)size() || index < 0)
        return false;

    *at(index) = r;
    return true;
}

//
//   删除指定的矩形区域。
//
bool CAppArea::Delete(int index)
{
    if (index >= (int)size() || index < 0)
        return false;

    delete at(index);
    erase(begin() + index);

    return true;
}

//
//   判断一个给定的点是否“触碰”到区域中的某一个矩形。
//
int CAppArea::PointHit(const CPnt &pt, float fDistGate)
{
    for (int i = 0; i < (int)size(); i++)
        if (at(i)->PointHit(pt, fDistGate))
            return i;

    return -1;
}

//
//   从文件中装入应用区域。
//
bool CAppArea::LoadBinary(FILE *fp)
{
    Clear();

    // 先读入矩形数量
    int count;
    if (fread(&count, sizeof(int), 1, fp) != 1)
        return false;

    // 预留空间
    resize(count);

    // 再依次读入各个矩形数据
    for (int i = 0; i < (int)size(); i++)
    {
        // 生成新的矩形区域
        CRectangle* p = CreateRect();
        if (p == NULL)
            return false;

        if (!p->LoadBinary(fp))
            return false;

        at(i) = p;
    }

    return true;
}

//
//   应用区域保存到文件。
//
bool CAppArea::SaveBinary(FILE *fp)
{
    // 先写入矩形数量
    int count = (int)size();
    if (fwrite(&count, sizeof(int), 1, fp) != 1)
        return false;

    // 再依次写入各个矩形数据
    for (int i = 0; i < (int)size(); i++)
        if (!at(i)->SaveBinary(fp))
            return false;

    return true;
}

#ifdef _MFC_VER
void CAppArea::Draw(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor)
{
    for (int i = 0; i < (int)size(); i++)
        at(i)->Draw(ScrnRef, pDC, crColor, 2);
}

#elif defined QT_VERSION
void CAppArea::Draw(CScreenReference &ScrnRef, QPainter *pPainter, QColor crLine, QColor crFill)
{
    for (int i = 0; i < (int)size(); i++)
        at(i)->Draw(ScrnRef, pPainter, crLine, 1, Qt::DashLine, true, crFill);
}
#endif
