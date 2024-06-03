#ifndef __CAppArea
#define __CAppArea

#include <vector>
#include "Geometry.h"

#ifdef QT_VERSION
#include <QPainter>
#include <QColor>
#endif

///////////////////////////////////////////////////////////////////////////////
//   定义“CAppArea”，描述由若干矩形区域组成的“应用范围”。
class DllExport CAppArea : public std::vector<CRectangle *>
{
  private:
  public:
    CAppArea() {}
    virtual ~CAppArea();

    // 清除全部区域
    virtual void Clear();

    // 重载操作符"+="
    CAppArea &operator+=(const CRectangle &r);

    // 重载操作符"+="
    CAppArea &operator+=(const CRectangle *r);

    // 更改指定的矩形区域
    bool Update(int index, const CRectangle &r);

    // 删除指定的矩形区域
    bool Delete(int index);

    // 判断一个给定点是否处于应用范围内
    int Contain(const CPnt &pt) const;

    // 判断一个给定的点是否“触碰”到区域中的某一个矩形
    int PointHit(const CPnt &pt, float fDistGate);

    // 生成一个新的矩形应用区域
    virtual CRectangle *CreateRect() { return new CRectangle; }

    // 从文件中装入应用区域
    virtual bool LoadBinary(FILE *fp);

    // 应用区域保存到文件
    virtual bool SaveBinary(FILE *fp);

#ifdef _MFC_VER
    void Draw(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor);

#elif defined QT_VERSION
    void Draw(CScreenReference &ScrnRef, QPainter *pPainter, QColor crLine, QColor crFill);
#endif
};
#endif
