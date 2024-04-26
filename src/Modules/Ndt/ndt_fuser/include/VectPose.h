#pragma once

#include <stdio.h>
#include <vector>
#include "Geometry.h"
#include "ndt_options.h"

#ifdef NDT1_USE_MINI_EIGEN
#include "ME_Transform.h"
#define Eigen MiniEigen

#else
#include <Eigen/Eigen>
#endif

using namespace std;

#ifdef _MSC_VER
class CDC;
#elif defined QT_VERSION
class QPainter;
class QColor;
#endif

class CScreenReference;
class CPnt;
class CPosture;

///////////////////////////////////////////////////////////////////////////////

class DllExport CStatusPosture : public CPosture
{
  public:
    int m_nStatus;    // 0 - Odometry; 1 - scan matched; 2-slam localize

  public:
    CStatusPosture(const CPosture p, int nStatus = 0)
    {
        SetPosture(p);
        m_nStatus = nStatus;
    }

    CStatusPosture(int nStatus = 0) { m_nStatus = nStatus; }

    CStatusPosture(const Eigen::Affine3d &affine);

    void operator=(const Eigen::Affine3d &affine);

    // 从文件装入位姿状态
    bool LoadBinary(FILE *fp);

    // 将位姿状态写入文件
    bool SaveBinary(FILE *fp);
};

///////////////////////////////////////////////////////////////////////////////
//   历史姿态记录
class CVectPose : public vector<CStatusPosture>
{
  private:
    bool m_bShowPoses;
    bool m_bShowSelected;
    int m_nSelected;

  public:
    CVectPose()
    {
        m_bShowPoses = false;
        m_bShowSelected = true;
        m_nSelected = -1;
    }

    // 选择指定的姿态
    void Select(int nIdx) { m_nSelected = nIdx; }

    // 设置显示选项
    void SetOption(bool bPosesOn, bool bSelectedOn)
    {
        m_bShowPoses = bPosesOn;
        m_bShowSelected = bSelectedOn;
    }

    // 取得指定序号的姿态
    CPosture GetPosture(int nIdx);

    bool LoadBinary(FILE *fp);
    bool SaveBinary(FILE *fp);

    // 判断指定的点是否触碰到某个位姿
    int PointHit(const CPnt &pt, float fDistGate);

#ifdef _MFC_VER
    //   绘制整个位姿曲线和姿态
    void Plot(CDC *pDc, CScreenReference &ScrnRef, unsigned long clrTraj, unsigned long clrPoses,
              unsigned long clrSelected, unsigned long clrUnmatched);

    void PlotLatestOnly(CDC *pDc, CScreenReference &ScrnRef, unsigned long clrPose);

#elif defined QT_VERSION
    //   绘制整个位姿曲线和姿态
    void Plot(QPainter *pPainter, CScreenReference &ScrnRef, QColor clrTraj, QColor clrPoses, QColor clrSelected,
              QColor clrUnmatched);

    void PlotLatestOnly(QPainter *pPainter, CScreenReference &ScrnRef, QColor clrPose);
#endif
};

#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif
