#ifndef __CEdgeFeature
#define __CEdgeFeature

#include "ShortLineFeature.h"

class CScanPointCloud;

///////////////////////////////////////////////////////////////////////////////
//   边缘特征(视为一种点特征)。
class CEdgeFeature : public CShortLineFeature
{
  protected:
  public:
    CEdgeFeature();

    // 设置边缘特征的参数
    virtual void SetParam(float fMinWidth, float fAngle, int nWhichSideToUse,
                          float fMaxIncidenceAngle, int nMinNumPoints, float fMaxLength);

    // 针对给定的点云，在规定的角度范围内，检测点云中是否含有该边缘特征特征，并返回它的中心位置
    virtual bool Detect(CPosture &pst, CScan *pScan, float fStartAngle, float fEndAngle,
                        CPnt *ptCenter);

    // 生成一个复本
    virtual CPointFeature *Duplicate();

#if 0
	// 从文件中装入点特征的参数
	virtual bool Load(FILE* fp);

	// 将点特征的参数保存到文件中
	virtual bool Save(FILE* fp);
#endif

#ifdef _MSC_VER
    // 在屏幕上绘制此点特征
    virtual void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, COLORREF crSelected,
                      int nSize);
#endif
};
#endif
