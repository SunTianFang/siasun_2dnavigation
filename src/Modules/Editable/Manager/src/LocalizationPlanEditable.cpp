#include <stdafx.h>
#include "LocalizationPlanEditable.h"
#include "RoboMappingView.h"

///////////////////////////////////////////////////////////////////////////////
//   定义“CLocalizationPlanEditable”类，它是“CLocalizationPlan”类的可编辑版本。

CLocalizationPlanEditable::CLocalizationPlanEditable()
{
    visible_ = true;
    curRect_ = -1;
    editState_ = 0;
    editParam_ = 0;
}

//
//    设置Mapping视口的指针。
//
void CLocalizationPlanEditable::SetMappingView(CRoboMappingView *view)
{
    mappingView_ = view;
}

//
//   取得指定的区域。
//
CLocalizationRect *CLocalizationPlanEditable::GetRect(int index)
{
    return dynamic_cast<CLocalizationRect *>(at(index));
}

//
//   设置当前唯一选中的定位矩形区域。
//   注：如果rect < 0，则无任何区域被唯一选中。
//
void CLocalizationPlanEditable::SingleSelect(int rect)
{
    curRect_ = rect;
}

//
//   从区域表中删除当前唯一选中的那个矩形区域。
//
bool CLocalizationPlanEditable::DeleteSingleSelected()
{
    // 如果当前有唯一选中区域，则删除它
    if (curRect_ >= 0)
    {
        Delete(curRect_);
        curRect_ = -1;
        return true;    //  删除成功，返回true
    }
    else
        return false;    // 删除失败，返回false
}

//
//   向定位方案图中加入一个新的矩形区域。
//   返回值：
//     -1 : 添加失败
//    >= 0: 新添加的区域的索引号
//
int CLocalizationPlanEditable::AddNewRect(const CRectangle &r)
{
    // 如果矩形面积太小，则不允许添加
    if (r.Area() < 0.001f)
        return -1;

    CLocalizationRect *p = new CLocalizationRect(r);
    if (p == NULL)
        return -1;

    *this += p;

    // 将新加入的矩形区域设置为当前唯一选中区域
    curRect_ = (int)size() - 1;

    return curRect_;
}

//
//   判断一个给定点是否处于应用范围内。
//
int CLocalizationPlanEditable::Contain(const CPnt &pt) const
{
    if (!visible_)
        return -1;

    return CAppArea::Contain(pt);
}

//
//   进入/退出矩形编辑模式。
//
void CLocalizationPlanEditable::SetEditMode(bool on)
{
    if (on)
    {
        editState_ = 1;
        editParam_ = 0;
    }
    else
    {
        editState_ = 0;
        editParam_ = 0;
    }
}

void CLocalizationPlanEditable::OnLButtonDown_EditModel(CPnt pt, CScreenReference &scrnRef)
{
    if (editState_ != 1)
        return;

    CRectangle *r = at(curRect_);
    if (r == NULL)
        return;

    // 判断是否点击到了矩形的某一部分
    int hitRect = r->PointHit(pt, 6 / scrnRef.m_fRatio);

    // 仅当点击到某一控制点或矩形内部时，才进行相应处理
    if ((hitRect >= CRectangle::LEFT_TOP_POINT && hitRect <= CRectangle::BOTTOM_MIDDLE_POINT) ||
        hitRect == CRectangle::OTHER_INSIDE_POINT)
    {
        ptRef_ = pt;             // 记录参考点
        editState_ = hitRect;    // 根据点击的起始控制点决定编辑状态
    }
}

void CLocalizationPlanEditable::OnLButtonUp_EditModel(CPnt pt)
{
    CRectangle *r = at(curRect_);
    if (r == NULL)
        return;

    // 是否是矩形边框上的一个拖拽控制点
    bool isCtrlPoint =
        (editState_ >= CRectangle::LEFT_TOP_POINT && editState_ <= CRectangle::BOTTOM_MIDDLE_POINT);

    // 是否是矩形内部的一个点
    bool isInnerPoint = (editState_ == CRectangle::OTHER_INSIDE_POINT);

    // 如果不是上面的两种点，在此返回
    if (!isCtrlPoint && !isInnerPoint)
        return;

    CPnt ptLeftTop = r->GetLeftTopPoint();
    CPnt ptRightBottom = r->GetRightBottomPoint();

    switch (editState_)
    {
    case CRectangle::LEFT_TOP_POINT:    // 左上角
        ptLeftTop = pt;
        break;

    case CRectangle::RIGHT_TOP_POINT:    // 右上角
        ptRightBottom.x = pt.x;
        ptLeftTop.y = pt.y;
        break;

    case CRectangle::LEFT_BOTTOM_POINT:    // 左下角
        ptLeftTop.x = pt.x;
        ptRightBottom.y = pt.y;
        break;

    case CRectangle::RIGHT_BOTTOM_POINT:    // 右下角
        ptRightBottom = pt;
        break;

    case CRectangle::LEFT_MIDDLE_POINT:    // 左边线中点
        ptLeftTop.x = pt.x;
        break;

    case CRectangle::RIGHT_MIDDLE_POINT:    // 右边线中点
        ptRightBottom.x = pt.x;
        break;

    case CRectangle::TOP_MIDDLE_POINT:    // 上边线中点
        ptLeftTop.y = pt.y;
        break;

    case CRectangle::BOTTOM_MIDDLE_POINT:    // 下边线中点
        ptRightBottom.y = pt.y;
        break;

    case CRectangle::OTHER_INSIDE_POINT:    // 矩形内部其它点
    {
        CPnt ptDelta = pt - ptRef_;
        ptLeftTop += ptDelta;
        ptRightBottom += ptDelta;
    }
    break;
    }

    // 重新生成矩形
    r->Create(ptLeftTop, ptRightBottom);

    mappingView_->update();
    editState_ = 1;
    mappingView_->setCursor(Qt::ArrowCursor);
}

//
//   处理鼠标移动。
//
void CLocalizationPlanEditable::OnMouseMove_EditModel(CPnt pt)
{
    CRectangle *r = at(curRect_);
    if (r == NULL)
        return;

    // 判断是否点击到了矩形的某一部分
    CScreenReference &scrnRef = mappingView_->m_ScrnRef;

    int hitRect = r->PointHit(pt, 6 / scrnRef.m_fRatio);

    // 如果尚未开始拖拽
    if (editState_ > 1)
        hitRect = editState_;

    switch (hitRect)
    {
    case CRectangle::LEFT_TOP_POINT:        // 左上角
    case CRectangle::RIGHT_BOTTOM_POINT:    // 右下角
        mappingView_->setCursor(Qt::SizeFDiagCursor);
        break;

    case CRectangle::RIGHT_TOP_POINT:      // 右上角
    case CRectangle::LEFT_BOTTOM_POINT:    // 左下角
        mappingView_->setCursor(Qt::SizeBDiagCursor);
        break;

    case CRectangle::LEFT_MIDDLE_POINT:     // 左边线中点
    case CRectangle::RIGHT_MIDDLE_POINT:    // 右边线中点
        mappingView_->setCursor(Qt::SizeHorCursor);
        break;

    case CRectangle::TOP_MIDDLE_POINT:       // 上边线中点
    case CRectangle::BOTTOM_MIDDLE_POINT:    // 下边线中点
        mappingView_->setCursor(Qt::SizeVerCursor);
        break;

    case CRectangle::OTHER_INSIDE_POINT:    // 矩形内部其它点
        mappingView_->setCursor(Qt::SizeAllCursor);
        break;

    case CRectangle::OUTSIDE_POINT:    // 矩形内部其它点
        mappingView_->setCursor(Qt::ArrowCursor);
        break;
    }
}

//
//   画出定位方案图。
//
void CLocalizationPlanEditable::PlotMap(QPainter *painter)
{
    if (!visible_)
        return;

    CScreenReference &scrnRef = mappingView_->m_ScrnRef;

    // 显示全局应用区域表
//    QColor crAreaFill = NORNAL_CELL_COLOR;
//    crAreaFill.setAlphaF(0.2);
//    Draw(scrnRef, painter, NORNAL_CELL_COLOR, crAreaFill);

    for (int i = 0; i < (int)size(); i++)
    {
        CLocalizationRect *r = dynamic_cast<CLocalizationRect *>(at(i));
        if (r == NULL)
            continue;
        // dq
        int m = r->methodId_[0];
        QColor colors[5] = {Qt::blue, Qt::magenta, Qt::green, Qt::cyan, Qt::yellow};
        QColor crAreaFill = colors[m];
        crAreaFill.setAlphaF(0.20);

        at(i)->Draw(scrnRef, painter, Qt::black, 1, Qt::DashLine, true, crAreaFill);
    }
}

//
//   画出当前选中的区域。
//
void CLocalizationPlanEditable::PlotSelected(QPainter *painter, CScreenReference &scrnRef)
{
    if (!visible_)
        return;

    // 显示被选中的应用区域
    if (curRect_ >= 0)
        at(curRect_)->Draw(scrnRef, painter, Qt::blue, 2);
}

//
//   画出正处于编辑状态的矩形。
//
void CLocalizationPlanEditable::PlotEditRect(QPainter *painter, CScreenReference &scrnRef)
{
    CRectangle *r = at(curRect_);
    if (r == NULL)
        return;

    r->Draw(scrnRef, painter, Qt::blue, 2);

    float left = r->Left();
    float right = r->Right();
    float top = r->Top();
    float bottom = r->Bottom();

    CPnt pts[8];
    pts[0].x = left;
    pts[0].y = top;
    pts[1].x = right;
    pts[1].y = top;
    pts[2].x = left;
    pts[2].y = bottom;
    pts[3].x = right;
    pts[3].y = bottom;
    pts[4].x = left;
    pts[4].y = (top + bottom) / 2;
    pts[5].x = right;
    pts[5].y = (top + bottom) / 2;
    pts[6].x = (left + right) / 2;
    pts[6].y = top;
    pts[7].x = (left + right) / 2;
    pts[7].y = bottom;

    for (int i = 0; i < 8; i++)
    {
        pts[i].Draw(scrnRef, painter, Qt::white, 4, 0);
        pts[i].Draw(scrnRef, painter, Qt::blue, 4, 2);
    }
}

void CLocalizationPlanEditable::OnDraw_EditModel(QPainter *painter, CScreenReference &scrnRef)
{
    if (!visible_ || curRect_ < 0)
        return;

    switch (editState_)
    {
    case 0:
        break;

    case 1:
    case 2:
        PlotEditRect(painter, scrnRef);
        break;
    }
}
