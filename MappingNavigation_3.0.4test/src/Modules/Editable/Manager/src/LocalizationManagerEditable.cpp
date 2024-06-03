#include <stdafx.h>
#include "LocalizationManagerEditable.h"
#include "LocalizationMethodsEditable.h"
#include "LocalizationPlanEditable.h"
#include "TreeSelector.h"
#include "MethodEditable.h"

#include "AffinePosture.h" // By Sam Test

///////////////////////////////////////////////////////////////////////////////
//   定义“CLocalizationManagerEditable”类，它是“CLocalizationManager”类的可编辑版本。

CLocalizationManagerEditable::CLocalizationManagerEditable()
{
    methodUsedIdx_ = -1;
}

bool CLocalizationManagerEditable::Create()
{
    Clear();

    // 分配并生成可编辑的定位方法集合
    methods_ = new CLocalizationMethodsEditable;
    if (methods_ == NULL || !methods_->Create())
        return false;

    // 在此设置CLocalizationRect类的静态成员methods_
    CLocalizationRect::SetLocalizationMethods(methods_);

    plan_ = new CLocalizationPlanEditable;
    if (plan_ == NULL)
        return false;

    return true;
}

//
//   定位过程函数。
//
CMatchInfo *CLocalizationManagerEditable::Localize(Eigen::Affine3d &estimatePose)
{
    CMatchInfo *info = CLocalizationManager::Localize(estimatePose);

    // By Sam For LegMethod
    long long int legBegin = GetTickCount();
    Eigen::Affine3d aaa = estimatePose;
    bool testaa = CLocalizationManager::LegLocalize(aaa);

    long long int legEnd = GetTickCount();
    std::cout << "By Sam: testaa = " << testaa << std::endl;
    std::cout << "By Sam: LegLocalTime = " << static_cast<int>(legEnd - legBegin) << std::endl;

    if (testaa)
    {
        Eigen::Affine3d diff = estimatePose * aaa;
        CPosture diff_pst = AffineToPosture(diff);

        std::cout << "By Sam: Goods center in Map = (" << diff_pst.x << ", " << diff_pst.y <<
                     ", " << (diff_pst.fThita / 3.14) * 180 << ")" << std::endl;
    }


    // 如果定位成功，记录成功定位的方法编号(供工具软件显示)
//    methodUsedIdx_ = (info == NULL) ? -1 : info->type_;
    methodUsedIdx_ = (info->result_==CMatchInfo::MATCH_FAIL) ? -1 : info->type_;

    return info;
}

//
//   绘制所有定位方法的地图。
//
void CLocalizationManagerEditable::PlotNormalMap(QPainter *painter, CScreenReference &scrnRef)
{
    CLocalizationMethodsEditable *methods = dynamic_cast<CLocalizationMethodsEditable *>(methods_);
    if (methods == NULL)
        return;

    methods->PlotNormalMap(painter, scrnRef);
}

//
//   绘制当前定位方法的定位情况。
//
void CLocalizationManagerEditable::PlotLocalization(QPainter *painter, CScreenReference &scrnRef)
{
    if (methodUsedIdx_ < 0)
        return;

    // 取得定位方法集合
    CLocalizationMethodsEditable *methods = dynamic_cast<CLocalizationMethodsEditable *>(methods_);
    if (methods == NULL)
        return;

    // 取得当前使用的定位方法
    CMethodEditable *method = dynamic_cast<CMethodEditable *>(methods->at(methodUsedIdx_));
    if (method == NULL)
        return;

    // 绘制对应于当前定位方法的定位情况图
    method->PlotLocalization(painter, scrnRef);
}

//
//   显示当前定位的匹配数据。
//
void CLocalizationManagerEditable::ShowMatchStatus(QPainter *painter, CScreenReference &scrnRef,
                                                   int curStep, int totalSteps)
{
    if (methodUsedIdx_ < 0)
        return;

    // 取得定位方法集合
    CLocalizationMethodsEditable *methods = dynamic_cast<CLocalizationMethodsEditable *>(methods_);
    if (methods == NULL)
        return;

    // 取得当前使用的定位方法
    CMethodEditable *method = dynamic_cast<CMethodEditable *>(methods->at(methodUsedIdx_));
    if (method == NULL)
        return;

    // 显示对应于当前定位方法的匹配数据
    method->ShowMatchStatus(painter, scrnRef, curStep, totalSteps);
}

//
//   更新各种定位方法所用地图的可视属性。
//
void CLocalizationManagerEditable::UpdateVisibility(const CVisibilitySetting &vis)
{
    CLocalizationMethodsEditable *methods = dynamic_cast<CLocalizationMethodsEditable *>(methods_);
    if (methods != NULL)
        methods->UpdateVisibility(vis);

    CLocalizationPlanEditable *plan = dynamic_cast<CLocalizationPlanEditable *>(plan_);
    if (plan != NULL)
        plan->SetVisible(vis.showLocArea);
}
