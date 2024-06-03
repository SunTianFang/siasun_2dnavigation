#ifndef __CFlatReflectorFeature
#define __CFlatReflectorFeature

#include "ShortLineFeature.h"

//////////////////////////////////////////////////////////////////////////////
//   平面型反光板类型。
class DllExport CFlatReflectorFeature : public CShortLineFeature
{
  public:
    int m_nIntensity;    // 反光强度

  public:
    CFlatReflectorFeature(float _x, float _y);
    CFlatReflectorFeature();

    // 生成一个复本
    virtual CPointFeature *Duplicate() const;

    // 针对给定的点云，在规定的角度范围内，检测点云中是否含有该反光板特征，并返回它的中心位置
    virtual bool Detect(CPosture &pst, CScan *pScan, float fStartAngle, float fEndAngle, CPnt *ptCenter);

#ifdef _MFC_VER
    // 在屏幕上绘制此点特征
    virtual void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, COLORREF crSelected, int nSize,
                      int nShowDisabled = DISABLED_FEATURE_UNDERTONE, bool bShowSuggested = true);
#endif
};
#endif

