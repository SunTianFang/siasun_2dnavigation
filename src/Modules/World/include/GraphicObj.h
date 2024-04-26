#pragma once

#include <vector>
#ifdef DesktopRun
#include <QPainter>
#endif
#include "Geometry.h"
#include "ScrnRef.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//    定义“图形对象”基类，供所有图形对象继承。
class DllExport CGraphicObj
{
  protected:
    int objType_;         // 对象类型编号
    int objSubType_;      // 更多的类型信息
    int objStatus_;       // 物体当前的状态
    int objSubStatus_;    // 更多的状态信息
    bool visible_;        // 物体是否可见
    bool selected_;       // 物体是否被选中

  public:
    CGraphicObj()
    {
        visible_ = true;
        selected_ = false;
    }

    // 取得对象的引用
    CGraphicObj &GetGraphicObj() { return *this; }

    // 设置图形对象的类型
    void SetType(int type) { objType_ = type; }
    void SetSubType(int subType) { objSubType_ = subType; }

    // 图形对象是否可见
    bool IsVisible() { return visible_; }

    // 图形对象是否被选中
    bool IsSelected() { return selected_; }

    // 使图形对象可见/隐藏
    void SetVisible(bool visible) { visible_ = visible; }

    // 选中/不选该对象
    void Select(bool sel) { selected_ = sel; }

    // 测试给定的点是否触碰到该图形对象
    virtual bool PointHit(const CPnt &pt, float thresh) { return false; }
#ifdef DesktopRun
    // 绘制该图形对象
    virtual void Draw(CScreenReference &ScrnRef, QPainter *painter) {}
#endif


};

///////////////////////////////////////////////////////////////////////////////
//    定义“图形对象集合”类。
class DllExport CGraphicObjs : public vector<CGraphicObj *>
{
  public:
    CGraphicObjs() {}

    // 指定的图形对象是否可见
    bool IsVisible(int index) { return at(index)->IsVisible(); }

    // 指定的图形对象是否被选中
    bool IsSelected(int index) { return at(index)->IsSelected(); }

    // 显示/隐藏指定的对象
    void SetVisible(bool vis, int index = -1);

    // 选中/不选指定的对象
    void Select(bool sel, int index = -1);

    // 唯一选中指定的图形对象
    void SelectOnly(int index);

    // 向集合中加入一个图形对象
    void operator+=(const CGraphicObj *obj);

    // 向集合中加入另一个图形对象集合
    void operator+=(const CGraphicObjs &other);

    // 测试给定的点是否触碰到该图形对象集合中的某一个
    virtual int PointHit(const CPnt &pt, float thresh);
#ifdef DesktopRun
    // 绘制图形对象集合
    virtual void Draw(CScreenReference &ScrnRef, QPainter *painter);
#endif


};
