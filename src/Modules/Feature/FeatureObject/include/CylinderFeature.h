#ifndef __CCylinderFeature
#define __CCylinderFeature

#include "PointFeature.h"

class CScan;

///////////////////////////////////////////////////////////////////////////////
//   定义“圆柱特征”类。
class CCylinderFeature : public CPointFeature
{
  private:
    float m_fRadius;    // 柱状特征的半径

  public:
    CCylinderFeature();

    // 设置柱状特征的参数
    void SetParam(float fRadius);

    // 针对给定的点云，在规定的角度范围内，检测点云中是否含有该柱状物，并返回它的中心位置
    virtual bool Detect(CPosture &pst, CScan *pScan, float fStartAngle, float fEndAngle,
                        CPnt *ptCenter);

    // 从文本文件中装入点特征的参数
    virtual bool LoadText(FILE *fp);

    // 将点特征的参数保存到文本文件中
    virtual bool SaveText(FILE *fp);

    // 从文件中装入点特征的参数
    virtual bool LoadBinary(FILE *fp);

    // 将点特征的参数保存到文件中
    virtual bool SaveBinary(FILE *fp);

#ifdef _MFC_VER
    // 在屏幕上绘制此点特征
    virtual void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, COLORREF crSelected,
                      int nSize);
#endif
};
#endif
