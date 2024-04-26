#pragma once

#include "ndt_options.h"
#include "NdtBaseLocalization_oru.h"
#include "LocResult.h"
#include "ScannerParam.h"

#ifdef NDT1_USE_MINI_EIGEN
#include "ME_Transform.h"
#define Eigen MiniEigen
#else
#include <Eigen/Core>
#endif

namespace ndt_oru
{

class CNdtBaseLocalization;

class ndtInform_oru
{
  public:
    // 基于现有Map结构获得需要的信息
    // @param map                       ndtMap地图（ndt1）
    // @param localCountThr             阈值，判断在x或y方向上的本地Cell是否足够
    // @param matchRateThr              阈值，匹配率
    ndtInform_oru(CNdtBaseLocalization *solver);

    ~ndtInform_oru(){};

    void setParams(CScannerGroupParam *pParams);

    /* 从Map收集局部定位信息,产生 mvMatchedTarget*/
    // @param pos                       当前位姿
    void collectMatchInfo(Eigen::Affine3d pos);

    /* 获得临近Cell, 产生mvLocalTarget*/
    // @param pos                       当前位姿
    // @param cellReso                  单元分辨率
    // @param startLine && endLine      设定的传感器半径范围
    // @param startAngle && endAngle    设定的传感器角度范围
    void scanLocalTarget(Eigen::Affine3d pos);

    /* 获得临近Cell, 产生mvLocalTarget*/
    // @param cloud_in                  当前点云
    // @param pos                       当前位姿
    void scanLocalTarget_sam(const CPointCloud &cloud_in, Eigen::Affine3d pos);

    // 分别获得各个自由度上匹配情况
    // @param xLocal & yLocal &aLocal 是局部应匹配上的Cell个数
    // @param xMatched & yMatched & aMatched 是实际匹配上的Cell个数
    void getMatchStatus(NDTMatchInfo &results);

  private:
    bool lineCrossCell(Eigen::Vector3d Vp, Eigen::Vector3d Vn, Eigen::Matrix3d cov, Eigen::Vector3d mean,
                       Eigen::Vector3d &perpendicularPoint);

  public:
    //  实际匹配上的
    std::set<NDTCell *> mvMatchedTarget;
    // 局部的
    std::set<NDTCell *> mvLocalTarget;
    std::set<NDTCell *> samLocalTarget;
    // status
    // l:local，m:match，x,y 分别是x和y方向,o是方向不明显的
    // local总数 = lx + ly + lo
    // match总数 = mx + my + mo
    int lx, ly, lo, mx, my, mo;
    int lkx, lky, lko, mkx, mky, mko;
    int lpx, lpy, lpo, lpkx, lpky, lpko;
    int iterCount;
    int iterTime;

  public:
    ndt_oru::CNdtBaseLocalization *mpSolver;
    CScannerGroupParam *mpParams;
    CPointCloud mv_pts;

  public:
#ifndef NDT1_USE_MINI_EIGEN
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

}    // namespace ndt_oru
#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif
