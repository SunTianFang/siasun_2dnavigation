#pragma once

#include <QPainter>
#include "LocalizationPlan.h"
#include "ScrnRef.h"

class CRoboMappingView;

///////////////////////////////////////////////////////////////////////////////
//   声明“CLocalizationPlanEditable”类，它是“CLocalizationPlan”类的可编辑版本。

class DllExport CLocalizationPlanEditable : public CLocalizationPlan
{
  private:
    bool visible_;    // 定位方案图是否可见
    int curRect_;     // 当前唯一选中的矩形区域编号
    int editState_;   // 矩形编辑状态：0-未开始; 1-待选择参考点; 2-开始移动边界; 3-开始整体平移
    int editParam_;   // 矩形编辑参数，依编辑状态不同有不同的定义
    CPnt ptRef_;      // 矩形编辑的参考点
    CPnt ptCur_;      // 矩形编辑的当前点
    CRoboMappingView* mappingView_;

  public:
    CLocalizationPlanEditable();

    // 设置Mapping视口的指针
    void SetMappingView(CRoboMappingView *view);

    // 取得指定的区域
    CLocalizationRect *GetRect(int index);

    // 设置定位方案图可见/隐藏
    void SetVisible(bool visible) { visible_ = visible; }

    // 设置当前唯一选中的定位矩形区域
    void SingleSelect(int rectId);

    // 取得当前唯一选中的矩形区域编号
    int GetSingleSelected() const { return curRect_; }

    // 从区域表中删除当前唯一选中的那个矩形区域
    bool DeleteSingleSelected();

    // 向定位方案图中加入一个新的矩形区域
    int AddNewRect(const CRectangle &r);

    // 判断一个给定点是否处于应用范围内
    int Contain(const CPnt &pt) const;

    // 进入/退出矩形编辑模式
    void SetEditMode(bool on);

    // 画出定位方案图
    void PlotMap(QPainter *pPainter);

    // 画出当前选中的区域
    void PlotSelected(QPainter *painter, CScreenReference &scrnRef);

    void PlotEditRect(QPainter *painter, CScreenReference &scrnRef);

    void OnLButtonDown_EditModel(CPnt pt, CScreenReference &scrnRef);
    void OnLButtonUp_EditModel(CPnt pt);

    void OnMouseMove_EditModel(CPnt pt);

    void OnDraw_EditModel(QPainter *painter, CScreenReference &scrnRef);
};
