#ifndef __CNdtExtLocalization
#define __CNdtExtLocalization

#include "NdtBaseLocalization_oru.h"
#include "ndt_maps_oru.h"
#include "ndt_pointcloud.h"
#include "StampedAffine.h"
#include "ScannerParam.h"

#ifdef NDT1_USE_MINI_EIGEN
#define Eigen MiniEigen
#endif

#define LOC_SLIDE_MEAN_SIZE 4

namespace ndt_oru
{
#if 0
//
//   定义具有时间戳的姿态。
//
class DllExport CStampedAffine : public Eigen::Affine3d, public CTimeStamp
{
#ifndef NDT1_USE_MINI_EIGEN
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

  public:
    CStampedAffine(Eigen::Affine3d &affine, CTimeStamp &stamp) : Eigen::Affine3d(affine), CTimeStamp(stamp) {}

    CStampedAffine(Eigen::Affine3d &affine, unsigned int uTime) : Eigen::Affine3d(affine), CTimeStamp(uTime) {}

    CStampedAffine(Eigen::Affine3d &affine) : Eigen::Affine3d(affine) {}

    CStampedAffine() {}

    Eigen::Affine3d &GetAffine3dObject()
    {
        Eigen::Affine3d *p = static_cast<Eigen::Affine3d *>(this);
        return *p;
    }

    Eigen::Affine3d operator*(const Eigen::Affine3d &af)
    {
        Eigen::Affine3d *p = static_cast<Eigen::Affine3d *>(this);
        Eigen::Affine3d r = *p * af;
        return r;
    }
};
#endif

//////////////////////////////////////////////////////////////////////////////
//   适用于多激光扫描器的定NDT定位。

///////////////////////////////////////////////////////////////////////////////
//
//   CNdtExtLocalization封装了对具有多个激光器的机器人进行定位的方法。
//
//   说明：
//      1. 机器人可以拥有多个激光器(各自有不同的安装姿态)
//      2. 定位只针对机器人本体，不针对任何一个激光器
//      3. 里程数据、激光扫描数据都具有时间戳，并且它们可以异步到来
//      4. 定位算法考虑了机器人的速度因素，利用时间戳进行了补偿
//      5. 定位算法支持使用多个子图，每个子图可以具有不同的分辨率和区域
//      6. 定位算法支持使用SLAM方式，即，当使用环境模型进行定位失败时，可以通过临时建图实现定位
//
///////////////////////////////////////////////////////////////////////////////

class DllExport CNdtExtLocalization : public CNdtBaseLocalization
{
#ifndef NDT1_USE_MINI_EIGEN
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

  private:
    long long int m_tmStart;                               // 进行插补之前的时刻
    CStampedAffine m_odometry;                             // 未经插补的里程姿态
    vector<CLabeledPointCloud> m_clouds;                   // 未经插补的点云集合
    Eigen::Affine3d last_odom;                             // 上一次的里程姿态(用来计算姿态变化量)
    Eigen::Affine3d m_recentPoses[LOC_SLIDE_MEAN_SIZE];    // 用于输出姿态平滑滤波的阶数
    int m_nSlideDataCount;                                 // 已经存入姿态平滑缓冲区的数据的个数

  public:   // protected:
    CScannerGroupParam *m_pScannerGroupParam;    // 激光器组参数
    Eigen::Affine3d vel;                         // 速度向量
    Eigen::Affine3d odomAdjusted;                // 插补之后得到的里程姿态
    CStampedPointCloud cloudAdjusted;            // 插补之后得到的汇总点云

    CStampedPointCloud cloudCopy;
    Eigen::Affine3d poseCopy;

    Eigen::Affine3d Tnow;                        // 定位后的结果姿态
    unsigned int count_clouds;                   // 已收到的点云的数量
    NDTMaps *maps_;                              // 指向多子图数据的指针
    NDTMap *tempMap_;
    Eigen::Vector3d localMapSize;

  public:
    Eigen::Affine3d pose_;
    // 机器人里程姿态
    Eigen::Affine3d Tlast_fuse;

    CSubmapParam submapParam;
    int m_curSubmapId;
    bool m_bLocSlam;    // 当前是否处于SLAM定位状态

    double translation_fuse_delta;
    double rotation_fuse_delta;

  private:
    // 对计算出的结果姿态进行平滑滤波，并输出滤波后的结果
    Eigen::Affine3d FilterOutput(const Eigen::Affine3d &pose);

  protected:
    virtual NDTMaps *CreateMaps() { return new NDTMaps; }
    virtual NDTMap *CreateMap() {return new NDTMap;}

    // 接收到里程数据后的回调函数
    Eigen::Affine3d getOdomMove(Eigen::Affine3d &odometry);

    // 处理已收到的里程和激光扫描数据，并根据机器人速度对它们进行插补
    virtual void CollectOdomLaserData(long long int tmStart);

    // 为当前NDT子图设置初始姿态及第一帧点云
    virtual bool InitMap(Eigen::Affine3d initPos, CPointCloud &cloud, NDTMap *reuseMap = NULL);

    // 根据机器人位姿和接收到的点云，进行相应的更新处理
    virtual Eigen::Affine3d UpdateMap(Eigen::Affine3d &odometry, CPointCloud &cloud, bool localizeBeforeMapping,
                                      bool &matched);

  public:
    CNdtExtLocalization(double map_reso = DEFAULT_MAP_RESO);
    virtual ~CNdtExtLocalization();

    // 由外部提供NDT图指针
    void SetMaps(ndt_oru::NDTMaps *maps) { maps_ = maps; }

    // 取得地图指针
    ndt_oru::NDTMaps *GetMaps() {return maps_; }

    // 根据给定的NDT子图编号设置应用NDT子图
    void SetMapByID(int submapId);

    // 设置激光扫描器组参数
    void SetScannerGroupParam(CScannerGroupParam *pParam);

    bool AddScannerParam(CLaserScannerParam& param);


    // 设置定位初始姿态
    virtual void SetPose(const Eigen::Affine3d &initPos);

    // 设置机器人的瞬时速度
    void SetVelocity(const Eigen::Affine3d &v) { vel = v; }

    // 当收到里程变化数据时进行的回调
    void OnReceiveOdometry(const CStampedAffine &odomMove);

    // 当收到激光数据时进行的回调
    void OnReceiveLaserScan(int nScannerId, const CStampedPointCloud &cloud);

    // 进行异步的定位并返回姿态结果
    virtual bool AsynLocalize(Eigen::Affine3d &pose, bool realTimeRunning = true, bool UpdateMap = false,
                              bool restoreWhenFail = false);

    // 开始启用一个新的子图
    void StartNewSubmap(CSubmapParam *param);

    // 定位函数例程
    int LocalizationProc(Eigen::Affine3d &pose, bool bSlamModeEnabled);

    // By Sam: SLAM 函数例程
    bool SlamProc(Eigen::Affine3d &pose, bool init);

    // 通过处理新的一帧数据来建立(或更新)地图
    virtual int BuildMap(CPointCloud &cloud_in, Eigen::Affine3d &odometry, bool localizeBeforeMapping);

    //yu
    virtual bool CreateMap(Eigen::Affine3d &odometry);

    // 在指定的范围内进行扩展定位(静态定位)
    //	virtual int LocalizeEx(CPclPointCloud &cloud_in, Eigen::Affine3d& odometry);

    // 根据所提供的全局应用区域表，生成仅用于NDT定位的应用区域表

    // 从文件中装入地图
    virtual bool LoadBinary(FILE *fp);

    // 将地图保存到文件
    virtual bool SaveBinary(FILE *fp);

    void ClearScannerParam();

};
}    // namespace ndt_oru
#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif

#endif
