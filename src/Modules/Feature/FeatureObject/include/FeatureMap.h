#ifndef __CFeatureMap
#define __CFeatureMap

#include "PointFeatureSet.h"
#include "Scan.h"
#include "LineFeatureSet.h"
#include "RectFeature.h"
#include "GraphicObj.h"

///////////////////////////////////////////////////////////////////////////////
//    定义移动机器人的“特征图”模型。
class DllExport CFeatureMap
{
  public:
    CPointFeatureSet *m_pPointFeatures;    // 点特征集合
    CLineFeatureSet *m_pLineFeatures;      // 直线特征集合
    CRectangle m_rgn;                      // 全图覆盖区域

  protected:
    // 计算特征全图所覆盖的区域(用于仿真图形显示)
    void CalcCoveringRegion();

    virtual CLineFeatureSet *CreateLineFeatureSet();
    virtual CPointFeatureSet *CreatePointFeatureSet();

  public:
    CFeatureMap();
    virtual ~CFeatureMap();

    // 初始化
    virtual bool Init();

    // 取得点特征集合
    CPointFeatureSet *GetPointFeatures() { return m_pPointFeatures; }

    // 取得直线特征集合
    CLineFeatureSet *GetLineFeatures() { return m_pLineFeatures; }

    // 取得点云区域的宽度
    float Width() { return m_rgn.Width(); }

    // 取得点云区域的高度
    float Height() { return m_rgn.Height(); }

    // 取得最左点X坐标
    float LeftMost() { return m_rgn.Left(); }

    // 取得最右点X坐标
    float RightMost() { return m_rgn.Right(); }

    // 取得最上点Y坐标
    float TopMost() { return m_rgn.Top(); }

    // 取得最下点Y坐标
    float BottomMost() { return m_rgn.Bottom(); }

    // 取得中心点
    CPnt GetCenterPoint() { return m_rgn.GetCenterPoint(); }

    // 从文件中装入特征图
    virtual bool LoadBinary(FILE *fp);

    // 将特征图写入文件
    virtual bool SaveBinary(FILE *fp);
};
#endif
