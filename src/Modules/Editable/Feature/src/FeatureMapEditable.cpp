#include <stdafx.h>
#include "FeatureMapEditable.h"
#include "PointFeatureSetEditable.h"
#include "LineFeatureSetEditable.h"

///////////////////////////////////////////////////////////////////////////////

CPointFeatureSet *CFeatureMapEditable::CreatePointFeatureSet()
{
    return new CPointFeatureSetEditable();
}

CLineFeatureSet *CFeatureMapEditable::CreateLineFeatureSet()
{
    return new CLineFeatureSetEditable();
}

#if 0
//
//   收集特征图中的所有图形对象。
//
void CFeatureMap::CollectGraphicObjs(CGraphicObjs &GraphicObjs)
{
    GraphicObjs.clear();
    for (int i = 0; i < (int)m_pPointFeatures->size(); i++)
    {

    }

    for (int i = 0; i < (int)m_pLineFeatures->size(); i++)
    {

    }

}
#endif

#ifdef _MFC_VER

//
//   绘制全局图。
//
void CFeatureMapEditable::Plot(CScreenReference& ScrnRef, CDC* pDc, COLORREF clrPoint, COLORREF clrLine)
{
    m_pLineFeatures->Plot(ScrnRef, pDc, clrLine, clrLine);
    m_pPointFeatures->Plot(ScrnRef, pDc, clrPoint, clrPoint);
}
#elif defined QT_VERSION
//
//   绘制特征图。
//
void CFeatureMapEditable::Plot(CScreenReference& ScrnRef, QPainter *pPainter, QColor clrPoint, QColor clrLine)
{
    CLineFeatureSetEditable *lines = dynamic_cast<CLineFeatureSetEditable *>(m_pLineFeatures);
    if (lines != NULL)
        lines->Plot(ScrnRef, pPainter, clrLine, clrLine);

    CPointFeatureSetEditable *reflectors = dynamic_cast<CPointFeatureSetEditable *>(m_pPointFeatures);
    if (reflectors != NULL)
        reflectors->Plot(ScrnRef, pPainter, clrPoint, clrPoint, 2, false, 40);
}
#endif
