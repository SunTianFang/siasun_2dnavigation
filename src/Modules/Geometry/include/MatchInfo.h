#pragma once

#include <vector>
#include "Geometry.h"

using namespace std;
///////////////////////////////////////////////////////////////////////////////
//   定义“测试结果”类型，作为各种测试结果的基类。
class CMatchInfo
{
  public:
    enum { MATCH_FAIL = 0, MATCH_OK = 1, MATCH_SLAM_OK = 2, MATCH_CORRIDOR_OK = 3 , MATCH_LOADING_MAP = 4, MATCH_LOADMAP_FAILED =5, MATCH_TO_SINGLEFEATURE =6};
    enum { LOC_NDT = 0,LOC_FEATURE = 1,LOC_TEMPLATE = 2,LOC_GRID = 3, LOC_SLAM = 4,LOC_METHOD_NUM = 5};
    int type_;        // 测试类型：0-NDT; 1-特征; 2-模板; 3-分支界定; 4-网格
    int result_;      // 测试是否成功: 0-失败; 1-成功; 2-在启用SLAM的情况下成功
    CPosture initPst_; //为定位前的初始给定位置
    CPosture pst_;    // 结果定位姿态
    float matchRatio_;         // 匹配率
    int   matchNum_;            // 匹配个数
  public:
    CMatchInfo()
    {
        type_ = 0;
        result_ = MATCH_FAIL;
    }

    virtual CMatchInfo *Duplicate() const = 0;
};

class CVectMatchInfo : public vector<CMatchInfo *>
{
  public:
    CVectMatchInfo() {}
    ~CVectMatchInfo();

    void Clear();

    // 加入一条新的测试结果
    bool Append(const CMatchInfo &info);
};
