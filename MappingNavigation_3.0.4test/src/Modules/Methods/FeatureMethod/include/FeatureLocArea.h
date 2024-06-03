#pragma once

#include <vector>
#include "Geometry.h"
#include "AppArea.h"
#include "LocalizationParam.h"


using namespace std;
#define LINE_COMPARE_COUNT      5
///////////////////////////////////////////////////////////////////////////////
// 基于特征的定位方法所使用的参数
class DllExport CFeatureLocalizationParam : public CLocalizationParam
{
  public:
    float ratio;                //匹配效率（只要匹配效率大于ratio时才认为匹配成功）默认值 60.0
    float fLineEqualLimit_[LINE_COMPARE_COUNT];
    int   nLeastMatchCount_;    //最小匹配对数 默认值3
    int   nMostMatchCount_;     // 最大匹配对数(超出此数量的点特征将被扔掉) 默认值 5
    float minRange;             // 最小扫描距离(m 短于此距离的扫描点将被扔掉) 默认值100mm
    float maxRange;             // 最大扫描距离(m 超过此距离的扫描点将被扔掉) 默认值 25000mm
    float maxRefLineRange;           //最大的直线参考集范围
    float fMinLineLen_;         // 直线特征的最短长度
    float fSamePointMaxDist_;   // 对同一个点特征的两次测量所允许的最大距离
    float angleEqualLimit;    // 同一个特征点，反光板匹配时允许的角度(度) 默认值 5°
    float rangeEqualLimit;    // 同一个特征点，反光板匹配时允许的距离（m） 默认值 300mm
    int   criteritonThreshold;    // 反光板识别强度门限  默认值(倍加福R2000:700；Hokuyo：7000；SickNano：254,SICK581 254)
    bool  onlyUseInRectFeature;   //是否只应用区域内的特征定位
    int   refEfficientPointCount;    //识别为有效反光板点的最小个数(表示在识别放光板时需要连续多少个点的强度值大于反光板识别强度门限才认为是反光板) 默认值 1
    vector <CPnt>  vecSpecialPntList_;  //当前定位区域内排除的反光板集合
    vector <CLine> vecSpecialLineList_; //当前定位区域内排除的直线集合
    bool  isUseUseMultiFrameLoc;   //是否只应用多帧定位方法
    int   iMultiFrameLocDequeSize;//多帧定位时应用的帧数
    float fMaDisPowerfulValue;//马氏距离阀值
    bool isUseSinglePointLoc; //是否应用单点定位方法
    float  m_MaxDisWithoutFeature;    //单反光板区域内、未识别到反光板时可行最远距离

  public:
    CFeatureLocalizationParam();

    // 生成本数据的一个副本
    virtual CLocalizationParam *Duplicate();

    // 从二进制文件读取数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
    //   Operator "="
    void operator =(CFeatureLocalizationParam obj);

    void SetValue(const CFeatureLocalizationParam &obj);
};

///////////////////////////////////////////////////////////////////////////////
// 特征定位应用矩形区域: 定义在该矩形区域内的特征定位参数
class DllExport CFeatureLocalizationRect : public CRectangle
{
  public:
    CFeatureLocalizationParam param_;              // 特征方法所使用的参数

public:
    CFeatureLocalizationRect() {}

    // 从二进制文件读取数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
};
