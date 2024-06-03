#ifndef __CLocalizationDevel
#define __CLocalizationDevel

#include "NdtExtLocalization_oru.h"
#include "SlamDataSet.h"
#include "ndt_pointcloud.h"

#ifdef NDT1_USE_MINI_EIGEN
#define Eigen MiniEigen
#endif

namespace ndt_oru
{
///////////////////////////////////////////////////////////////////////////////
//   基于NDT的定位开发实验类。
class DllExport CDatasetLocalization : public CNdtExtLocalization
{
#ifndef NDT1_USE_MINI_EIGEN
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

  public:
    CSlamDataSet *m_pDataset;
    CVectPose correctedPoses;    // 校正后的历史姿态
    CVectMatchInfo m_vectMatchInfo;

  protected:
    // 接收一步数据，并做好定位准备
    virtual void collectStepData(const CSlamStepData &Step, bool *pScannersUsed);

    // 重载CNdtLocalization::Localize()函数，以便添加运行姿态记录
    virtual bool Localize(const CPointCloud &cloud_in, const Eigen::Affine3d &Tinit, Eigen::Affine3d &odometry);

  public:
    CDatasetLocalization(double map_reso = DEFAULT_MAP_RESO);

    // 设置数据集
    virtual void SetDataSet(CSlamDataSet *pDataset);

    // 接收一步数据，并做好定位准备
    virtual void collectStepData(int stepId);

    // 进行异步的定位并返回姿态结果
    virtual bool AsynLocalize(Eigen::Affine3d &pose, bool realTimeRunning = true, bool updateMap = false,
                              bool restoreWhenFail = false);

    // 处理新的一帧数据
    virtual int BuildMap(CPointCloud &cloud_in, Eigen::Affine3d &odometry, bool localizeBeforeMapping);

    //yu
    virtual bool CreateMap(Eigen::Affine3d &odometry);

    // 根据机器人位姿和接收到的点云，进行相应的更新处理
    virtual Eigen::Affine3d UpdateMap(Eigen::Affine3d &odometry, CPointCloud &cloud, bool localizeBeforeMapping,
                                      bool &matched);

    // 进行扩展定位并返回姿态结果
    virtual int LocalizeEx(Eigen::Affine3d &pose);

    // 取得对应于某一步的合成点云
    CStampedPointCloud GetStepPointCloud(int stepId);

#ifdef _MFC_VER
    void PlotLocalization(CDC *pDc, CScreenReference &ScrnRef, bool bShowSource, bool bShowTarget, bool bShowPoses,
                          bool bShowCellBox);
    void PlotCloudAdjusted(CDC *pDc, CScreenReference &ScrnRef);

#elif defined QT_VERSION
    void PlotLocalization(QPainter *pPainter, CScreenReference &ScrnRef, bool bShowSource, bool bShowTarget,
                          bool bShowPoses, bool bShowCellBox);
    void PlotCloudAdjusted(QPainter *pPainter, CScreenReference &ScrnRef);
#endif
};
}    // namespace ndt_oru

#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif

#endif
