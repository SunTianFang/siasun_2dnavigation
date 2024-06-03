#pragma once

#include "ndt_options.h"
#include "ndt_pointcloud.h"
#include "Geometry.h"

// 如果定义了NDT1_USE_MINI_EIGEN，则采用MiniEigen库
#ifdef NDT1_USE_MINI_EIGEN
#include "pcl.h"
#include "ME_Matrix.h"
#include "ME_Vector.h"
#include "ME_Transform.h"

#define Eigen MiniEigen    // 以下实现中，用"Eigen"代替"MiniEigen"

// 否则，采用Eigen库
#else
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>
#endif

#ifdef QT_VERSION
#include <QColor>
#endif

#ifdef _MFC_VER
class CDC;

#elif defined QT_VERSION
class QPainter;
class QColor;
#endif

class CPnt;
class CScreenReference;
class CEllipse;
class CRectangle;

namespace ndt_oru
{
CEllipse *CreateEllipseFromMeanCov(const CPnt &ptMean, const Eigen::Matrix2d &cov, double chi2);

///////////////////////////////////////////////////////////////////////////////
// “NDTCell”类实现了正态分布单元。它是所有NDT索引的基类，包含了正态分布的均值、
// 协方差矩阵以及协方差矩阵的特征分解结果。
class NDTCell
{
  public:
    int cellType_;
    Eigen::Vector3d viewDir_;

  private:
    const double EVAL_FACTOR = 1000;													// ???
    const int MIN_NB_POINTS_FOR_GAUSSIAN = 3;

    Eigen::Vector3d center_;    // NDT单元格的中心点(它仅与单元的位置有关，与分布无关)
    double sizeX_;              // 单元格在X方向上的边长
    double sizeY_;              // 单元格在Y方向上的边长
    Eigen::Matrix3d cov_;       // 正态分布变换(NDT)的协方差矩阵
    Eigen::Matrix3d icov_;      // 逆协方差矩阵
    Eigen::Matrix3d evecs_;     // 特征向量
    Eigen::Vector3d evals_;     // 特征值
    Eigen::Vector3d mean_;      // NDT单元的均值(与center_不同，它与分布有关)
    unsigned int N;             // 到目前为止用于估测NDT分布所用的点数
    float occ_;                 // (以"Log odds"保存的)单元的占据值
    bool canUpdate_;            // 该单元是否允许进行更新
    CPointCloud points_;               // 落入本单元的散点-更新后删除

  private:
    // 沿着给定的直线，以直线中心为中心，按给定的分辨率进行重采样
    CPointCloud Resample(CLine &ln, double reso);

  public:
    bool hasGaussian_;                 // 单元内是否含有高斯分布
    int matchStatus_;                  // 匹配状态: 0-未匹配; 1-匹配上
    Eigen::Vector3d meanMatchCell_;    // 匹配的单元的均值点
    bool isSelected_;                  // 此单元是否被选中(用于屏幕编辑)

  private:
    void InitializeVariables()
    {
        hasGaussian_ = false;
        N = 0;
        occ_ = 0;
        center_(0) = center_(1) = center_(2) = 0;
        cov_ = Eigen::MatrixXd::Identity(3, 3);
        icov_ = Eigen::MatrixXd::Identity(3, 3);
        evecs_ = Eigen::MatrixXd::Identity(3, 3);
        mean_ = Eigen::Vector3d(0, 0, 0);
        evals_ = Eigen::Vector3d(0, 0, 0);

        points_.clear();
        matchStatus_ = 0;
        meanMatchCell_ = Eigen::Vector3d(0, 0, 0);
        canUpdate_ = true;
        isSelected_ = false;
        cellType_ = 0;
    }

    // 为避免NDT单元内的协方差矩阵成为奇异矩阵，对它进行必要的调整
    void rescaleCovariance();

    //   计算一个点pt落入一个NDT单元中的似然度。
    double getLikelihood(const pcl::PointXYZ &pt) const;

  public:
    NDTCell(double xsize = 0, double ysize = 0, double zsize = 0)
    {
        InitializeVariables();
        sizeX_ = xsize;
        sizeY_ = ysize;
    }

    NDTCell(Eigen::Vector3d &center, double &xsize, double &ysize, double &zsize)
    {
        InitializeVariables();
        center_ = center;
        sizeX_ = xsize;
        sizeY_ = ysize;
    }

    // Copy构造函数
    NDTCell(const NDTCell &other)
    {
        InitializeVariables();

        center_ = other.center_;
        sizeX_ = other.sizeX_;
        sizeY_ = other.sizeY_;
        hasGaussian_ = other.hasGaussian_;
        N = other.N;
        occ_ = other.occ_;
        setMean(other.getMean());
        setCov(other.getCov());
        icov_ = other.getInverseCov();
        evals_ = other.getEvals();
        evecs_ = other.getEvecs();
        points_ = other.points_;
        matchStatus_ = other.matchStatus_;
        meanMatchCell_ = other.meanMatchCell_;
        canUpdate_ = other.canUpdate_;
    }

    virtual ~NDTCell() { points_.clear(); }

    virtual NDTCell *clone() const;
    virtual NDTCell *copy() const;

    // 设置单元的中心点
    inline void setCenter(const Eigen::Vector3d &cn) { center_ = cn; }

    // 取得单元中心点
    inline Eigen::Vector3d getCenter() const { return center_; }

    inline void getCenter(double &cx, double &cy, double &cz) const
    {
        cx = center_(0);
        cy = center_(1);
        cz = center_(2);
    }

    inline void setSize(double sizeX, double sizeY)
    {
        sizeX_ = sizeX;
        sizeY_ = sizeY;
    }

    inline void getSize(double &sizeX, double &sizeY) const
    {
        sizeX = sizeX_;
        sizeY = sizeY_;
    }

    // 取得单元所在的矩形
    CRectangle GetRect();

    // 设置协方差矩阵
    void setCov(const Eigen::Matrix3d &cov, bool rescaleCov = false)
    {
        cov_ = cov;
        if (rescaleCov)
            rescaleCovariance();
    }

    inline int getType() const { return cellType_; }

    // 取得协方差矩阵
    inline Eigen::Matrix3d getCov() const { return cov_; }

    // 取得逆协方差矩阵
    inline Eigen::Matrix3d getInverseCov() const { return icov_; }

    // 设置均值点
    inline void setMean(const Eigen::Vector3d &mean) { mean_ = mean; }

    // 取得均值点
    inline Eigen::Vector3d getMean() const { return mean_; }

    // 设置协方差矩阵的特征向量
    inline void setEvecs(const Eigen::Matrix3d &ev) { evecs_ = ev; }

    // 取得协方差矩阵的特征向量
    inline Eigen::Matrix3d getEvecs() const { return evecs_; }

    // 设置协方差矩阵的特征值
    inline void setEvals(const Eigen::Vector3d &ev) { evals_ = ev; }

    // 取得协方差矩阵的特征值
    inline Eigen::Vector3d getEvals() const { return evals_; }

    // 设置单元的占据值
    void setOccupancy(float occ_val) { occ_ = occ_val; }

    // (累加)更新单元的占据值
    void updateOccupancy(float occ_val, float max_occu = 255.0)
    {
        if (!canUpdate_)
            return;

        occ_ += occ_val;
        if (occ_ > max_occu)
            occ_ = max_occu;

        if (occ_ < -max_occu)
            occ_ = -max_occu;
    }

    void updateOccupancy1(const Eigen::Vector3d &m, const Eigen::Vector3d &origin, const Eigen::Vector3d &pt,
                          float occupancy_limit, double sensor_noise, double f1, double f2, double f3, int numpoints);

    // 取得单元的占据值
    float getOccupancy() { return occ_; }

    // 设置单元内的散点数
    void setN(int N_) { N = N_; }

    // 设置单元内的散点数
    int getN() { return N; }

    // 使能/禁止对此单元的更新
    void enableUpdate(bool update) { canUpdate_ = update; }

    // 判断此单元是否允许更新
    bool canUpdate() { return canUpdate_; }

    //   根据新提供的均值和协方差，对本单元的均值和协方差进行更新
    void updateSampleVariance(const Eigen::Matrix3d &cov2, const Eigen::Vector3d &m2,
                              unsigned int numpointsindistribution, bool updateOccupancyFlag = true,
                              float max_occu = 1024, unsigned int maxnumpoints = 1e9);

    // 根据单元内的散点计算单元的均值和协方差矩阵，并标注是否含有高斯分布
    void computeGaussian(unsigned int maxnumpoints = 1e9, float occupancy_limit = 255,
                         Eigen::Vector3d origin = Eigen::Vector3d(0, 0, 0), double sensor_noise = 0.1);

    // 向单元中加入一个散点
    void addPoint(const Eigen::Vector3d &pt) { points_.push_back(pt); }

    // 求直线(p1, p2)上的哪一点具有落入本单元的最大似然度，并返回这一最大似然度的值
    double computeMaximumLikelihoodAlongLine(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, Eigen::Vector3d &out);

    // 求直线(p1, p2)上的哪一点具有落入本单元的最大似然度，并返回这一最大似然度的值
    double computeMaximumLikelihoodAlongLine(const Eigen::Vector3d &ep1, const Eigen::Vector3d &ep2,
                                             Eigen::Vector3d &out);

    // 将NDT单元进行坐标变换
    void Transform(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr)
    {
        // 计算新的均值和协方差
        Eigen::Vector3d meanC = tr * mean_;
        Eigen::Matrix3d covC = tr.rotation() * cov_ * tr.rotation().transpose();

        mean_ = meanC;
        cov_ = covC;
    }

    // 选中/取消选中该单元
    void Select(bool bYesOrNo) { isSelected_ = bYesOrNo; }

    // 判断此单元是否被选中
    bool isSelected() { return isSelected_; }

    // 取得对应于本Cell的概率圆
    bool getEllipse(CEllipse &ellipse);

    // 以给定的采样分辨率对NDT单元进行重采样(生成一个点云)
    CPointCloud Resample(double reso);

    int writeToJFF(FILE *jffout);
    int loadFromJFF(FILE *jffin);
    void writeJFFMatrix(FILE *jffout, Eigen::Matrix3d &mat);
    void writeJFFVector(FILE *jffout, Eigen::Vector3d &vec);
    int loadJFFMatrix(FILE *jffin, Eigen::Matrix3d &mat);
    int loadJFFVector(FILE *jffin, Eigen::Vector3d &vec);

    //   判断一个给定的点是否“触碰”到该单元。
    int PointHit(const CPnt &pt, float fDistGate);

#ifndef NDT1_USE_MINI_EIGEN
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

#ifdef _MFC_VER
    void PlotCell(CScreenReference &ScrnRef, CDC *pDc, unsigned long clrLine, unsigned long clrFill);

    void PlotEllipse(CScreenReference &ScrnRef, CDC *pDc, unsigned long clrLine, unsigned long clrFill,
                     bool bShowMatched = false, unsigned long clrMatched = 0);

    // 显示单元的匹配情况(以直线显示匹配单元对)
    void PlotMatchStatus(CScreenReference &ScrnRef, CDC *pDc, unsigned long clr);

#elif defined QT_VERSION
    void PlotCell(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrLine, QColor clrFill);

    void PlotEllipse(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrLine, QColor clrFill,
                     bool bShowMatched = false, QColor clrMatched = Qt::black, int lineWidth = 1);

    // 显示单元的匹配情况(以直线显示匹配单元对)
    void PlotMatchStatus(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr);
#endif
};

// NDT单元向量类
class VectNDTCells : public std::vector<NDTCell *>
{
  public:
    Eigen::Affine3d homePose_;
    int ndtRound_;
    int ndtIter_;
    int ndtCycle_;
    double score_;
    int countGood_;

  public:
    VectNDTCells() { clear(); }

    VectNDTCells(const VectNDTCells &another) { Copy(another); }

    bool Copy(const VectNDTCells &another)
    {
        Clear();

        for (unsigned int i = 0; i < another.size(); i++)
        {
            NDTCell *cell = another[i];
            if (cell != NULL)
            {
                NDTCell *nd = (NDTCell *)cell->copy();
                if (nd == NULL)
                    return false;

                push_back(nd);
            }
        }
        homePose_ = another.homePose_;
        ndtRound_ = another.ndtRound_;
        ndtIter_ = another.ndtIter_;
        ndtCycle_ = another.ndtCycle_;
        score_ = another.score_;
        countGood_ = another.countGood_;

        return true;
    }

    void operator=(const VectNDTCells &another) { Copy(another); }

    // 清除所有单元
    void Clear()
    {
        for (unsigned int i = 0; i < size(); i++)
        {
            if (at(i) != NULL)
                delete at(i);
        }
        clear();
    }

    // 将所有NDT单元进行坐标变换
    void Transform(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr)
    {
        for (unsigned int i = 0; i < size(); i++)
            at(i)->Transform(tr);

        homePose_ = tr * homePose_;
    }

  public:
    //   判断一个给定的点是否“触碰”到该单元。
    int PointHit(const CPnt &pt, float fDistGate);

#ifdef _MFC_VER
    void PlotCell(CScreenReference &ScrnRef, CDC *pDc, unsigned long clr);

    void Plot(CScreenReference &ScrnRef, CDC *pDc, unsigned long clrLine, unsigned long clrFill,
              bool bShowMatched = false, unsigned long clrMatched = 0, bool bShowCellBox = false,
              bool bShowMatchCell = false);

#elif defined QT_VERSION
    void PlotCell(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr);

    void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrLine, QColor clrFill, bool bShowMatched = false,
              QColor clrMatched = Qt::black, bool bShowCellBox = false);
#endif
};
};    // namespace ndt_oru

#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif
