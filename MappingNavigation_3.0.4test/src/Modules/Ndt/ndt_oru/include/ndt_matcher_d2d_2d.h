#pragma once

#include "ndt_map_oru.h"

#ifdef NDT1_USE_MINI_EIGEN
#include "pcl.h"
#define Eigen MiniEigen

#else
#include "pcl/point_cloud.h"
#include "Eigen/Core"
#endif

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "MatchInfo.h"

#ifdef _MFC_VER
class CDC;
class CScreenReference;
#elif defined QT_VERSION
#include <QColor>
#endif

namespace ndt_oru
{
class NDTMatchResult : public CMatchInfo
{
  public:
    double finalscore;
    int runTime;
    int countGood;
    int countSourceNDT;
    int matchCycles;

  public:
    NDTMatchResult()
    {
        type_ = 0;    // 测试类型: NDT
        Reset();
    }

    NDTMatchResult(const NDTMatchResult &another)
    {
        *this = another;
    }

    // 分配生成本对象的一个副本
    virtual CMatchInfo *Duplicate() const
    {
        return new NDTMatchResult(*this);
    }

    void Reset()
    {
        result_ = CMatchInfo::MATCH_FAIL;
        finalscore = 0;
        countGood = 0;
        countSourceNDT = 0;
        matchCycles = 0;
        runTime = 0;
    }

    bool IsOk()
    {
//        if (countGood < 8)
//            result_ = CMatchInfo::MATCH_FAIL;
//        else if (countGood < 50)
//            result_ = (int)(countGood > countSourceNDT / 2);
//        else
//            result_ = (int)(countGood > countSourceNDT / 3);

//        return (result_ != CMatchInfo::MATCH_FAIL);

        // By Sam Test
        return true;
    }
};

/*****************************************
 *  class NDTMatcherD2D_2D
 *  通过ndt方法匹配和记录状态
 ******************************************/
class NDTMatcherD2D_2D
{
  public:
    NDTMatcherD2D_2D() { this->init(true, std::vector<double>()); }
    ~NDTMatcherD2D_2D();

    /*******************************************
     * \param  target :  地图
     * \param  source :  当前扫描数据
     * \param  T :  初始姿态
     ********************************************/
    bool match(NDTMap &target, NDTMap &source, Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> &T);

    // brief 计算匹配点数，以sourceNDT中心点mean实际落到targetNDT里为准
    // author
    // param[in] targetNDT 为 target_NDT
    // param[in] sourceNDT 为 source_NDT
    // param[in] T 用于旋转平移 sourceNDT
    // return 实际落在targetNDT里的sourceNDT个数
    int calculateMatchNum(NDTMap &targetNDT, NDTMap &sourceNDT,
                          Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> &T);

  private:
    double derivativesNDT_2d(const VectNDTCells &source, NDTMap &target, Eigen::Vector3d &score_gradient,
                             Eigen::Matrix3d &Hessian, bool computeHessian);

    void HandleSpecialCase(Eigen::Matrix3d &H, double score_gradient_2d_norm);

  protected:
    // 清除中间变量及状态
    void init(bool useDefaultGridResolutions, std::vector<double> _resolutions);

    // iteratively update the score gradient and hessian
    bool updateGradientHessian_2d(Eigen::Vector3d &score_gradient, Eigen::Matrix3d &Hessian, const Eigen::Vector3d &m1,
                                  const Eigen::Matrix3d &C1, const double &likelihood, bool computeHessian);

    // iteratively update the score gradient and hessian (2d version)
    inline bool updateGradientHessian_local2d(Eigen::MatrixXd &score_gradient, Eigen::MatrixXd &Hessian,
                                              const Eigen::Vector3d &m1, const Eigen::Matrix3d &C1,
                                              const double &likelihood, const Eigen::Matrix3d &_Jest,
                                              const Eigen::Matrix<double, 9, 3> &_Hest,
                                              const Eigen::Matrix<double, 3, 9> &_Zest,
                                              const Eigen::Matrix<double, 9, 9> &_ZHest, bool computeHessian);

    // pre-computes the derivative matrices Jest, Hest, Zest, ZHest
    void computeDerivatives_2d(Eigen::Vector3d &m1, Eigen::Matrix3d C1, bool computeHessian = true);

    // pre-computes the derivative matrices Jest, Hest, Zest, ZHest
    inline void computeDerivatives_Local2d(Eigen::Vector3d &m1, Eigen::Matrix3d C1, Eigen::Matrix<double, 3, 3> &_Jest,
                                           Eigen::Matrix<double, 9, 3> &_Hest, Eigen::Matrix<double, 3, 9> &_Zest,
                                           Eigen::Matrix<double, 9, 9> &_ZHest, bool computeHessian);

    // perform line search to find the best descent rate (Mohre&Thuente)
    // adapted from NOX
    double lineSearch2D(Eigen::Matrix<double, 3, 1> &increment, VectNDTCells &source, NDTMap &target);

    // brief 该函数主要根据 sourceNDT 的范围画出 local_targetNDT，并根据 local_targetNDT 动态计算权重值的高低值
    // author
    // param[in] sourceNDT 为 source_NDT
    // param[in] targetNDT 为 target_NDT
    // param[out] weight_High 为 高权重值
    // param[out] weight_low 为 地权重值
    void calculateWeight(const VectNDTCells &sourceNDT, NDTMap &targetNDT, double &weight_High, double &weight_low);

    // brief 判断 sourceNDT 对应的 cell 是否为特征 cell，如果是特征 cell 则增加 cell 的权重值
    // author
    // param[in] input_cell 为 sourceNDT 对应的 cell
    // param[in] weight_high 为高权重值
    // param[in] weight_low 为低权重值
    // return 权重值
    double distributeWeight(NDTCell &input_cell, double weight_high, double weight_low);

  public:
    double current_resolution;

    /// max iterations, set in constructor
    int ITR_MAX;
    double ITER_TIME_MAX;    // ms
    int iterCount_;
    int iterTime_;
    /// the change in score after which we converge. Set to 1e-3 in constructor
    double DELTA_SCORE;
    // how many neighbours to use in the objective
    int n_neighbours;

    // weight value
    bool use_weight;
    bool auto_weight;
    double weight_value;
    double normal_value;
    double weight;

    NDTMatchResult result;
    VectNDTCells nextNDT;
    VectNDTCells nextNDT0;

    // initializes stuff;
  public:
#ifdef _MFC_VER
    void ShowStatus(CDC *pDC, CScreenReference &ScrnRef, int nCurStep, int nTotalSteps, int submapId,
                    unsigned long clrText);

    void Plot(CDC *pDc, CScreenReference &ScrnRef, unsigned long clrBorder, unsigned long clrFill,
              unsigned long clrBorderFail, unsigned long clrFillFail);

#elif defined QT_VERSION
    void ShowStatus(QPainter *pPainter, CScreenReference &ScrnRef, int nCurStep, int nTotalSteps, int submapId,
                    QColor clrText);

    void Plot(QPainter *pPainter, CScreenReference &ScrnRef, QColor clrBorder, QColor clrFill, QColor clrBorderFail,
              QColor clrFillFail);
#endif

    void ClearMatchStatus();

  protected:
    Eigen::Matrix<double, 3, 3> Jest;
    Eigen::Matrix<double, 9, 3> Hest;
    Eigen::Matrix<double, 3, 9> Zest;
    Eigen::Matrix<double, 9, 9> ZHest;

    // vars for gradient
    Eigen::Matrix<double, 3, 1> xtBJ, xtBZBx, Q;

    // vars for hessian
    Eigen::Matrix<double, 3, 3> JtBJ, xtBZBJ, xtBH, xtBZBZBx, xtBZhBx;
    Eigen::Matrix<double, 1, 3> TMP1, xtB;

    double lfd1, lfd2;
    std::vector<double> resolutions;

    // auxiliary functions for MoreThuente line search
    static double MoreThuente_min(double a, double b);
    static double MoreThuente_max(double a, double b);
    static double MoreThuente_absmax(double a, double b, double c);
    static int MoreThuente_cstep(double &stx, double &fx, double &dx, double &sty, double &fy, double &dy, double &stp,
                                 double &fp, double &dp, bool &brackt, double stmin, double stmax);

  public:
#ifndef NDT1_USE_MINI_EIGEN
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

}    // namespace ndt_oru

#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif
