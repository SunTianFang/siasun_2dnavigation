#pragma once

#include <vector>
#include "DatasetLocalization.h"
#include "PointFeatureSet.h"
#include "NdtMapsEditable.h"
#include "LineFeature.h"

#ifdef NDT1_USE_MINI_EIGEN
#define Eigen MiniEigen
#endif

#ifdef _MFC_VER
#include <afxmt.h>
class CDC;
class CScreenReference;
#elif defined QT_VERSION
#include <QtCore>
#include <QColor>
#endif

// 定义SLAM模式
#define SLAM_MODE_STEP_RUN 0          // 单步运行(建模/定位)
#define SLAM_MODE_QUICK_BUILD 1       // 连续建模
#define SLAM_MODE_BATCH_LOCALIZE 2    // 连续定位

class CSlamDataSet;

using namespace std;

///////////////////////////////////////////////////////////////////////////////

namespace ndt_oru
{
#ifdef QT_VERSION

class CMapFuser;

class DllExport CMapFuserThread : public QThread
{
  private:
    QMutex m_mutex;

  public:
    ndt_oru::CMapFuser *m_pFuser;
    bool m_bKillThread;
    bool m_bThreadDead;

  public:
    CMapFuserThread();

    void run() override;
    void Start(ndt_oru::CMapFuser *pFuser);
    void Stop();
    void MutexLock();
    void MutexUnlock();
};
#endif

class DllExport CMapFuser : public CDatasetLocalization
{
  private:
    // 地图范围及分辨率
    double map_size_x;
    double map_size_y;

    int m_nSlamTargetStep;
    bool m_bLocalizeBeforeMapping;    // 是否在建图前先进行定位

  public:
    int m_nSlamMode;            // 0-断续式运行; 1-连续建模; 2-连续定位

#ifdef _MFC_VER
    static HANDLE m_hKillThread;
    static HANDLE m_hThreadDead;
    CCriticalSection m_crit;

#elif defined QT_VERSION
    CMapFuserThread m_Thread;
#endif

  public:
    int m_nCurStep;
    CPointFeatureSet highIntensPoints;      // 高亮点集合(非特征，此处借用CPointFeaturueSet数据结构)
    CPointFeatureSet refPoints;             // 经用户确认的反光板集合

  public:
    CMapFuser(double map_size_x = DEFAULT_MAP_SIZE_X, double map_size_y = DEFAULT_MAP_SIZE_Y,
              double map_reso = DEFAULT_MAP_RESO);

    ~CMapFuser();

    // 设置建图模式(是否在建图前先进行定位)
    void SetMappingMode(bool bLocalizeBeforeMapping) { m_bLocalizeBeforeMapping = bLocalizeBeforeMapping; }

    // 设置数据集
    virtual void SetDataSet(CSlamDataSet *pDataset);

    NDTMapsEditable *GetMaps() { return (NDTMapsEditable *)maps_; }

    // 取得当前的活动子图
    NDTMapEditable *GetCurSubmap() { return (NDTMapEditable *)map; }

    // 从文件中装入地图
    virtual bool LoadBinary(FILE *fp);

    // 将地图保存到文件
    virtual bool SaveBinary(FILE *fp);

    ////////////////////////////////////////////////////////////////////////////

    // 根据数据集进行单步建模
    bool StepBuild();

    bool StepLocalize();
    bool InitBuild(Eigen::Affine3d origin);
#ifdef _MFC_VER
    // 后台支持线程
    static UINT SupportProc(LPVOID pParam);
#elif defined QT_VERSION
    // 运行支撑函数
    bool SupportRoutine();
#endif

    // 启动连续SLAM过程
    bool StartSlamThread(int nSlamMode, int nTargetStep);

    // 结束连续SLAM过程
    void StopSlamThread();

    void MutexLock();
    void MutexUnlock();

    void ToggleSubmapSelect(int nId)
    {
        NDTMapEditable *submap = (NDTMapEditable *)maps_->at(nId);
        submap->isSelected_ = !submap->isSelected_;
    }

    void ToggleSubmapVisible(int nId)
    {
        NDTMapEditable *submap = (NDTMapEditable *)maps_->at(nId);
        submap->isVisible_ = !submap->isVisible_;
    }

    // 对所有选中的子图进行坐标变换
    void TransformSelectedSubmaps(const Eigen::Affine3d &tr);

    // 对所有选中的NDT单元进行坐标变换
    void TransformSelectedCells(const Eigen::Affine3d &tr);

    // 尝试对所有选中的单元进行直线拟合
    bool SelectedCellsFitLineFeature(CLineFeature *line);

    // 更新匹配结果的统计数据
    void UpdateMatchStatistics(int &averageTime, int &maxTime, int &minTime);

#ifdef _MSC_VER
    void PlotModelMap(CDC *pDc, CScreenReference &ScrnRef, unsigned long clrCellFill, bool bShowMatched = false,
                      unsigned long clrMatched = 0);

    void PlotPoses(CDC *pDC, CScreenReference &ScrnRef, unsigned long clrTraj, unsigned long clrPoses,
                   unsigned long clrSelected, unsigned clrUnmatched);

    // 显示所有选中子图
    void PlotSelectedSubmap(CDC *pDC, CScreenReference &ScrnRef, unsigned long clr);

    // 显示当前的匹配子图
    void PlotSubmapPair(CDC *pDC, CScreenReference &ScrnRef, int nSourceId, int nTargetId, unsigned long clrSource,
                        unsigned long clrTarget);

#elif defined QT_VERSION
    void PlotModelMap(QPainter *pPainter, CScreenReference &ScrnRef, QColor clrCellFill, bool bShowMatched = false,
                      QColor clrMatched = Qt::black);

    void PlotPoses(QPainter *pPainter, CScreenReference &ScrnRef, QColor clrTraj, QColor clrPoses, QColor clrSelected,
                   QColor clrUnmatched);

    // 显示所有选中子图
    void PlotSelectedSubmap(QPainter *pPainter, CScreenReference &ScrnRef, QColor clr);

    // 显示指定子图的应用区域
    void PlotSubmapAppArea(QPainter *pPainter, CScreenReference &ScrnRef, int submapId, QColor clr);

    // 显示当前的匹配子图
    void PlotSubmapPair(QPainter *pPainter, CScreenReference &ScrnRef, int nSourceId, int nTargetId, QColor clrSource,
                        QColor clrTarget);

    // 显示匹配数据
    void ShowMatchStatus(QPainter *pPainter, CScreenReference &ScrnRef, QColor clrText);
    // 显示统计数据
//    void ShowMatchStatistics(QPainter *pPainter);
#endif

  public:
#ifndef NDT1_USE_MINI_EIGEN
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};
}    // namespace ndt_oru

#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif
