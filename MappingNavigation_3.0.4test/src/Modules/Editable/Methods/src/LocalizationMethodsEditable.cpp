#include <stdafx.h>
#include "LocalizationMethodsEditable.h"
#include "NdtMethodEditable.h"
#include "FeatureMethodEditable.h"
#include "TemplateMethodEditable.h"
#include "ScanMatchMethodEditable.h"
#include "SlamMethodEditable.h"

///////////////////////////////////////////////////////////////////////////////

//
//   生成所有的定位方法。
//
bool CLocalizationMethodsEditable::Create()
{
    resize(5);

    CLocalizationMethod *method;

    // NDT方法
    method = new CNdtMethodEditable;
    if (method == NULL)
        return false;
    at(0) = method;

    // 特征方法
    method = new CFeatureMethodEditable;
    if (method == NULL)
        return false;
    at(1) = method;

    // 模板方法
    method = new CTemplateMethodEditable;
    if (method == NULL)
        return false;
    at(2) = method;

    method = new CScanMatchMethodEditable;
    if (method == NULL)
        return false;
    at(3) = method;

    method = new CSlamMethodEditable;
    if (method == NULL)
        return false;
    at(4) = method;


    return true;
}

//
//   绘制所有定位方法的地图。
//
void CLocalizationMethodsEditable::PlotNormalMap(QPainter *painter, CScreenReference &scrnRef)
{
    for (int i = 0; i < (int)size(); i++)
    {
        CMethodEditable *p = dynamic_cast<CMethodEditable *>(at(i));
        if (p == NULL)
            continue;

        p->PlotNormalMap(painter, scrnRef);
    }
}

//
//   更新各种定位方法所用地图的可视属性。
//
void CLocalizationMethodsEditable::UpdateVisibility(const CVisibilitySetting &vis)
{
    CNdtMethodEditable *ndtMethod = dynamic_cast<CNdtMethodEditable *>(at(0));
    if (ndtMethod == NULL)
        return;

    ndtMethod->SetVisible(vis.showNdtMap);

    CFeatureMethodEditable *featureMethod = dynamic_cast<CFeatureMethodEditable *>(at(1));
    if (featureMethod == NULL)
        return;
    featureMethod->SetVisible(vis.showLines, vis.showReflectors);

    CTemplateMethodEditable *templateMethod = dynamic_cast<CTemplateMethodEditable *>(at(2));
    if (templateMethod == NULL)
        return;
    templateMethod->SetVisible(vis.showStaticObjs);

    CScanMatchMethodEditable *scanmatchMethod = dynamic_cast<CScanMatchMethodEditable *>(at(3));
    if (scanmatchMethod == NULL)
        return;
    scanmatchMethod->SetVisible(vis.showGridMap);
}
