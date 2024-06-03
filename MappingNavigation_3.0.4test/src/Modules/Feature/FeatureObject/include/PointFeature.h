#ifndef __CPointFeature
#define __CPointFeature

#include <stdio.h>
#include "Geometry.h"
#include "Feature.h"

#include<Eigen/Dense>
using namespace Eigen;

#ifdef QT_VERSION
#include <QColor>
#endif

// 同类点特征距离误差门限
#define FEATURE_REG_DISTANCE_WINDOW 1000

class CScan;


class stVariousDis
{
public:
    float  m_fEuclideanDismin;//点与点之间的欧式平均距离
    float  m_fEuclideanDismax;//点与点之间的欧式最大距离
    float  m_fEuclideanDisAverage;//点与点之间的欧式最小距离
    float  m_fMaDismax;//点与点之间马氏最大距离
    float  m_fMaDismin;//点与点之间马氏最小距离
    float  m_fMaDisAverage;//点与点之间马氏的平均距离


    float  m_fToCenterEuclideanDismin;//点与中心点之间的欧式平均距离
    float  m_fToCenterEuclideanDismax;//点与中心点之间的欧式最大距离
    float  m_fToCenterEuclideanDisAverage;//点与中心点之间的欧式最小距离
    float  m_fToCenterMaDismax;//点与中心点之间马氏最大距离
    float  m_fToCenterMaDismin;//点与中心点之间马氏最小距离
    float  m_fToCenterMaDisAverage;//点与中心点之间马氏的平均距离
    Matrix2d m_cov;//协方差矩阵

public:
    stVariousDis()
    {
        m_fEuclideanDismin = 0;//点与点之间的欧式平均距离
        m_fEuclideanDismax =0;//点与点之间的欧式最大距离
        m_fEuclideanDisAverage=0;//点与点之间的欧式最小距离
        m_fMaDismax=0;//点与点之间马氏最大距离
        m_fMaDismin=0;//点与点之间马氏最小距离
        m_fMaDisAverage=0;//点与点之间马氏的平均距离

        m_fToCenterEuclideanDismin=0;//点与中心点之间的欧式平均距离
        m_fToCenterEuclideanDismax=0;//点与中心点之间的欧式最大距离
        m_fToCenterEuclideanDisAverage=0;//点与中心点之间的欧式最小距离
        m_fToCenterMaDismax=0;//点与中心点之间马氏最大距离
        m_fToCenterMaDismin=0;//点与中心点之间马氏最小距离
        m_fToCenterMaDisAverage=0;//点与中心点之间马氏的平均距离
    }
    stVariousDis(float  fEuclideanDismin,
    float  fEuclideanDismax,
    float  fEuclideanDisAverage,
    float  fMaDismax,
    float  fMaDismin,
    float  fMaDisAverage,
    float  fToCenterEuclideanDismin,
    float  fToCenterEuclideanDismax,
    float  fToCenterEuclideanDisAverage,
    float  fToCenterMaDismax,
    float  fToCenterMaDismin,
    float  fToCenterMaDisAverage,
    Matrix2d cov):m_fEuclideanDismin(fEuclideanDismin),m_fEuclideanDismax(fEuclideanDismax),
        m_fEuclideanDisAverage(fEuclideanDisAverage),m_fMaDismax(fMaDismax),
        m_fMaDismin(fMaDismin),m_fMaDisAverage(fMaDisAverage),m_fToCenterEuclideanDismin(fToCenterEuclideanDismin),
       m_fToCenterEuclideanDismax(fToCenterEuclideanDismax), m_fToCenterEuclideanDisAverage(fToCenterEuclideanDisAverage),m_fToCenterMaDismax(fToCenterMaDismax),
       m_fToCenterMaDismin(fToCenterMaDismin),m_fToCenterMaDisAverage(fToCenterMaDisAverage),m_cov(cov){}


    double CalculateMahalanobisDistance(const CPnt pStart,const CPnt pTo,const Matrix2d cov)
    {
        MatrixXd mm(2, 1);
        mm << (pStart.x - pTo.x), (pStart.y - pTo.y);
        MatrixXd temp(0, 0);
        temp = mm.transpose()*cov.inverse()*mm;
        return sqrt(sqrt(temp(0, 0)));
    }

    // 重载 "+="
    void operator=(const stVariousDis &ostVariousDis)
    {
          m_fEuclideanDismin = ostVariousDis.m_fEuclideanDismin;//点与点之间的欧式平均距离
          m_fEuclideanDismax =ostVariousDis.m_fEuclideanDismax;//点与点之间的欧式最大距离
          m_fEuclideanDisAverage=ostVariousDis.m_fEuclideanDisAverage;//点与点之间的欧式最小距离
          m_fMaDismax=ostVariousDis.m_fMaDismax;//点与点之间马氏最大距离
          m_fMaDismin=ostVariousDis.m_fMaDismin;//点与点之间马氏最小距离
          m_fMaDisAverage=ostVariousDis.m_fMaDisAverage;//点与点之间马氏的平均距离

          m_fToCenterEuclideanDismin=ostVariousDis.m_fToCenterEuclideanDismin;//点与中心点之间的欧式平均距离
          m_fToCenterEuclideanDismax=ostVariousDis.m_fToCenterEuclideanDismax;//点与中心点之间的欧式最大距离
          m_fToCenterEuclideanDisAverage=ostVariousDis.m_fToCenterEuclideanDisAverage;//点与中心点之间的欧式最小距离
          m_fToCenterMaDismax=ostVariousDis.m_fToCenterMaDismax;//点与中心点之间马氏最大距离
          m_fToCenterMaDismin=ostVariousDis.m_fToCenterMaDismin;//点与中心点之间马氏最小距离
          m_fToCenterMaDisAverage=ostVariousDis.m_fToCenterMaDisAverage;//点与中心点之间马氏的平均距离
          m_cov=ostVariousDis.m_cov;//协方差矩阵
    }

    //
    //   从二进制文件中装入点特征的参数。
    //
    bool LoadBinary(FILE* fp)
    {
        // 读入点的坐标
        float f[12];
        if (fread(f, sizeof(float), 12, fp) != 12)
        return false;

        m_fEuclideanDismin= f[0];
        m_fEuclideanDismax= f[1];
        m_fEuclideanDisAverage= f[2];
        m_fMaDismax= f[3];
        m_fMaDismin= f[4];
        m_fMaDisAverage= f[5];
        m_fToCenterEuclideanDismin= f[6];
        m_fToCenterEuclideanDismax= f[7];
        m_fToCenterEuclideanDisAverage= f[8];
        m_fToCenterMaDismax= f[9];
        m_fToCenterMaDismin= f[10];
        m_fToCenterMaDisAverage= f[11];

        float dtemp[4];

        if (fread(&dtemp, sizeof(float), 4, fp) != 4)
            return false;

        m_cov(0, 0) = dtemp[0];
        m_cov(1, 0) = dtemp[1];
        m_cov(0, 1) = dtemp[2];
        m_cov(1, 1) = dtemp[3];

        return true;
    }

    //
    //   将点特征的参数保存到二进制文件中。
    //
    bool SaveBinary(FILE* fp)
    {
        float f[12] = {m_fEuclideanDismin, m_fEuclideanDismax,m_fEuclideanDisAverage,m_fMaDismax,m_fMaDismin,m_fMaDisAverage,
                     m_fToCenterEuclideanDismin,m_fToCenterEuclideanDismax,m_fToCenterEuclideanDisAverage,m_fToCenterMaDismax,
                     m_fToCenterMaDismin,m_fToCenterMaDisAverage};

        if (fwrite(f, sizeof(float), 12, fp) != 12)
        return false;

        float dtemp[4];
        dtemp[0] = m_cov.coeff(0, 0);
        dtemp[1] = m_cov.coeff(1, 0);
        dtemp[2] = m_cov.coeff(0, 1);
        dtemp[3] = m_cov.coeff(1, 1);

        if (fwrite(dtemp, sizeof(float), 4, fp) != 4)
        return false;


        return true;
    }

};

///////////////////////////////////////////////////////////////////////////////
//   定义“点特征”基类。

class DllExport CPointFeature : public CFeature, public CPnt
{
  private:
//    bool m_bSelected;

  public:
    CPointFeature(float _x = 0, float _y = 0);
    CPointFeature(const CPnt &pt);

    // 设置中心位置
    void SetCenterPoint(const CPnt &ptCenter) { GetPntObject() = ptCenter; }

    // 生成一个复本
    virtual CPointFeature *Duplicate() const;

    // 判断该点特征是否被指定的扫描线照到，并返回扫描点坐标
    virtual bool HitByLineAt(CLine &lnRay, CPnt &ptHit, float &fDist);

    // 针对给定的点云，在规定的角度范围内，检测点云中是否含有该点特征，并返回它的中心位置
    virtual bool Detect(CPosture &pst, CScan *pScan, float fStartAngle, float fEndAngle,
                        CPnt *ptCenter);

    // 判断指定的入射光线是否可用
    virtual bool CheckInRay(CLine &lnRay) { return true; }

#if 0
    // 返回当前特征点的选中/未选中状态
    bool IsSelected() { return m_bSelected; }

    // 选中/不选该特征
    void Select(bool sel) { m_bSelected = sel; }
#endif

    // 从文本文件中装入点特征的参数
    virtual bool LoadText(FILE *fp);

    // 将点特征的参数保存到文本文件中
    virtual bool SaveText(FILE *fp);

    // 从二进制文件中装入点特征的参数
    virtual bool LoadBinary(FILE *fp);

    // 将点特征的参数保存到二进制文件中
    virtual bool SaveBinary(FILE *fp);

    // By Sam: 对特征点进行坐标变换
    void PointTransform(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr);


    // 重载 "=="
    bool operator==(const CPointFeature &pt) const;

public:
    stVariousDis m_stVariousDis;
};
#endif
