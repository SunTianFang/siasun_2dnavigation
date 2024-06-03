#pragma once

#include "LocalizationMethods.h"
#include "LocalizationPlan.h"
#include "StampedAffine.h"
#include "ScannerParam.h"
#include "MatchInfo.h"
#include "RawScan.h"
#include "type.h"



Eigen::Affine3d operator*(const Eigen::Affine3d &v, double k);

namespace ndt_oru
{
void transformPointCloudInPlace(
    const Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> &Tr, CPointCloud &pc);
}

///////////////////////////////////////////////////////////////////////////////
// 声明“CLocalizationManager”类，为将多种定位方式融合提供统一的框架。
class CLocalizationManager
{
  private:
    long long int m_tmStart;                         // 进行插补之前的时刻

//    vector<ndt_oru::CLabeledPointCloud> m_clouds;    // 未经插补的点云集合    By Sam Delete
    Eigen::Affine3d vel_;         // 速度向量
    long long int uLatestTime_;    // 一个处理周期中，最后一个里程、激光数据的时间
  protected:
    Eigen::Affine3d Tnow_;        // 当前姿态
    ndt_oru::CStampedAffine m_odometry;              // 未经插补的里程姿态
    Eigen::Affine3d odomAdjusted;    // 插补之后得到的里程姿态
    ndt_oru::CStampedPointCloud cloudAdjusted;    // 插补之后得到的汇总点云
  public:
    Eigen::Affine3d last_odom;    // 上一次的里程姿态(用来计算姿态变化量)
    CScannerGroupParam *m_pScannerGroupParam;    // 激光器组参数
    double sensor_max_range;                     // 扫描最远距离
    double sensor_min_range;                     // 扫描最近距离

    // By Sam change private to public
    vector<ndt_oru::CLabeledPointCloud> m_clouds;    // 未经插补的点云集合


    bool           m_UnuseScanMatchMethod;// dq 11.28

    int            m_lastMethod;
    bool           m_bFirstFeatureFail = true;

  protected:
    CLocalizationMethods *methods_;
    CLocalizationPlan *plan_;

  public:
    // 对输入的点云进行处理，滤除过近的点，并对Z坐标进行初始化
    void FilterCloud(const ndt_oru::CPointCloud &cloud_in, ndt_oru::CPointCloud &cloud_filtered);

    // 处理已收到的里程和激光扫描数据，并根据机器人速度对它们进行插补
    void CollectOdomLaserData();
    // By Sam: For PlayBack, 处理已收到的里程和激光扫描数据
    void CollectOdomLaserData_PlayBack();

    // 接收到里程数据后的回调函数，返回相对于上一次调用时的姿态变化量
    Eigen::Affine3d getOdomMove(Eigen::Affine3d &odometry);

    bool TransformCloudToMsg(const CScan &scan,const CPosture &odom);

  public:
    CLocalizationManager();
    virtual ~CLocalizationManager();

    // 清除数据
    void Clear();

    // 生成对象
    virtual bool Create();

    virtual bool UnloadMap() ; //lishen

    // 取得指向定位方法集合的指针
    CLocalizationMethods *GetMethods() { return methods_; }

    // 取得指向定位方案的指针
    CLocalizationPlan *GetPlan() { return plan_; }

    virtual void SetScannerGroupParam(CScannerGroupParam *pParam);

    // 设置当前的姿态
    void SetPose(const Eigen::Affine3d &pose);

    // 设置当前的速度
    void SetVelocity(const Eigen::Affine3d &v);

    // 当收到里程变化数据时进行的回调
    void OnReceiveOdometry(const ndt_oru::CStampedAffine &odomMove);

    // 当收到激光数据时进行的回调
    virtual void OnReceiveLaserScan(int nScannerId, const ndt_oru::CStampedPointCloud &cloud);

    // 定位过程函数
    virtual CMatchInfo *Localize(Eigen::Affine3d &estimatePose);

    // By Sam:
    bool LegLocalize(Eigen::Affine3d &estimatePose);

    // 定位过程函数
    virtual CMatchInfo *PlayBack(Eigen::Affine3d &estimatePose);

    // 从二进制文件装载数据
    virtual bool LoadBinary(FILE *fp, string filename,int floor, bool bChangeFloor);

    // 将数据保存到二进制文件
    virtual bool SaveBinary(FILE *fp, string filename);
    virtual bool SaveFeatureBinary(FILE *fp, string filename);
    void ClearScannerParam();

    bool AddScannerParam(CLaserScannerParam param);

};
