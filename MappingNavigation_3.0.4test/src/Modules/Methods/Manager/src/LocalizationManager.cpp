#include <stdafx.h>
#include "LocalizationManager.h"
#include "AffinePosture.h"

#include "RoboLocClnt.h"
#include "BaseOdometry.h"  // By Sam Test
#include "ScanMatchMethod.h"   // By Sam Test
#include "NdtMethod.h"  //BY DQ 9.1
#include "FeatureMethod.h"
#include "SlamMethod.h"
#include "TemplateMethod.h"   // By Sam For LegMethod
// Fix me: 下面的两个宏定义应改为可由用户设置的参数(从文件调入)，待实现!!

#define MAX_SENSOR_RANGE 40     // 传感器最远有效距离
#define MIN_SENSOR_RANGE 0.01    // 传感器最近有效距离 // By yu : Change sensor_min_range


extern bool readJffFormat;

///////////////////////////////////////////////////////////////////////////////
//   定义“CLocalizationManager”类，为将多种定位方式融合提供统一的框架。

CLocalizationManager::CLocalizationManager()
{
    methods_ = NULL;
    plan_ = NULL;
    Tnow_.setIdentity();
    vel_.setIdentity();
    uLatestTime_ = 0;
    sensor_max_range = MAX_SENSOR_RANGE;
    sensor_min_range = MIN_SENSOR_RANGE;

    m_UnuseScanMatchMethod = true;//dq 11.28

}

CLocalizationManager::~CLocalizationManager()
{
    Clear();
}

//
//   清除数据。
//
void CLocalizationManager::Clear()
{
    if (methods_ != NULL)
    {
        delete methods_;
        methods_ = NULL;
    }

    if (plan_ != NULL)
    {
        delete plan_;
        plan_ = NULL;
    }
}

//
//   生成对象。
//
bool CLocalizationManager::Create()
{
    Clear();

    methods_ = new CLocalizationMethods;
    if (methods_ == NULL)
        return false;

    // 在此设置CLocalizationRect类的静态成员methods_
    CLocalizationRect::SetLocalizationMethods(methods_);

    plan_ = new CLocalizationPlan;
    if (plan_ == NULL)
        return false;

    return true;
}
bool CLocalizationManager::UnloadMap()
{
    if (methods_ == NULL)
        return false;

    return methods_->UnloadMap();

}


void CLocalizationManager::SetScannerGroupParam(CScannerGroupParam *pParam)
{
    m_pScannerGroupParam = pParam;
    for(int i = 0;i<methods_->size();i++)
    {
        methods_->at(i)->SetScannerGroupParam(pParam);
    }
}

//
//   设置当前的姿态。
//
void CLocalizationManager::SetPose(const Eigen::Affine3d &pose)
{
    Tnow_ = pose;
    last_odom = pose;
    ((CFeatureMethod *)methods_->at(1))->ReSetMethod();
}

//
//   设置当前的速度。
//
void CLocalizationManager::SetVelocity(const Eigen::Affine3d &v)
{
    vel_ = v;
}

//
//   当收到里程变化数据时进行的回调。
//
void CLocalizationManager::OnReceiveOdometry(const ndt_oru::CStampedAffine &odomMove)
{
    m_odometry.GetAffine3dObject() = last_odom * odomMove;
    uLatestTime_ = odomMove.m_dwTimeStamp;

    CPosture odometry_pst = AffineToPosture (m_odometry);



}

//
//   当收到激光数据时进行的回调。
//
void CLocalizationManager::OnReceiveLaserScan(int nScannerId,
                                              const ndt_oru::CStampedPointCloud &cloud)
{
    // 标记传感器编号、过滤点云，并标记时间戳
    ndt_oru::CLabeledPointCloud labeledCloud(nScannerId);
    FilterCloud(cloud, labeledCloud);
    labeledCloud.Stamp(cloud.m_dwTimeStamp);

    if (cloud.m_dwTimeStamp > uLatestTime_)
        uLatestTime_ = cloud.m_dwTimeStamp;

    // 将标记好传感器编号的点云暂存到点云集合中
    m_clouds.push_back(labeledCloud);

}
//
// by lishen 将点云 等转为 msg结构形式
//
bool CLocalizationManager::TransformCloudToMsg(const CScan &scan,const CPosture &odom)
{

     CSlamMethod *pSlamMethod  = (CSlamMethod *)(methods_->at(4));
    static bool firstScan = true;



    if(firstScan)
    {
        short laser_id = 0;

        pSlamMethod->m_scanMsg.sensor_id = (std::string)("scan0");
        pSlamMethod->m_scanMsg.angle_increment = m_pScannerGroupParam->at(laser_id).m_fReso;
        pSlamMethod->m_scanMsg.angle_max = m_pScannerGroupParam->at(laser_id).m_fEndAngle;;
        pSlamMethod->m_scanMsg.angle_min = m_pScannerGroupParam->at(laser_id).m_fStartAngle;
        pSlamMethod->m_scanMsg.range_max = m_pScannerGroupParam->at(laser_id).m_fMaxRange;
        pSlamMethod->m_scanMsg.range_min = m_pScannerGroupParam->at(laser_id).m_fMinRange;
        pSlamMethod->m_scanMsg.scan_time = 0.02;   //50hz
        pSlamMethod->m_scanMsg.time_increment = 0.000005;

        Pose lasertf;
        lasertf.x       = m_pScannerGroupParam->at(laser_id).m_pst.x;
        lasertf.y       = m_pScannerGroupParam->at(laser_id).m_pst.y;
        lasertf.theta   = m_pScannerGroupParam->at(laser_id).m_pst.fThita ;

        pSlamMethod->m_scanMsg.laser_tf = lasertf;
        firstScan = false;
    }


  /*  m_scanMsg.ranges.clear();
    for(auto r: pRawCloud->distance)
    {
        m_scanMsg.ranges.push_back(float(r)/1000.0);  //???
    }*/

    pSlamMethod->m_scanMsg.stamp = scan.m_dwTimeStamp;

    pSlamMethod->m_scanMsg.ranges.clear();
    for(int m=0;m< scan.m_nCount;m++)
    {
        float r =  scan.m_pPoints[m].r/1000.0;
                               //r<???
          pSlamMethod->m_scanMsg.ranges.push_back(r);
    }



    pSlamMethod->m_odomMsg.pos.x      = odom.x;
    pSlamMethod->m_odomMsg.pos.y      = odom.y;
    pSlamMethod->m_odomMsg.pos.theta  = odom.fThita;
    pSlamMethod->m_odomMsg.stamp = (double)(pSlamMethod->m_scanMsg.stamp - 2) / 1000.0;     // sec



    // std::cout << "angle_min"<< lscan.angle_min<<" "<<lscan.angle_max<<" "<<lscan.angle_increment<<std::endl;
    // std::cout << "laser tf "<<lasertf.x<<" "<<lasertf.y<<" "<<lasertf.theta<<std::endl;
     //std::cout << "m_scanMsg.stamp "<<m_scanMsg.stamp<<std::endl;

    // std::cout << "m_odomMsg.stamp "<<m_odomMsg.stamp<<std::endl;
    // std::cout << "odom 1 "<< odom.x<<" "<<odom.y<<" "<<180.0*odom.fThita/3.1415<<std::endl;
    // odom.stamp = (double)(rawScan.odom_data.time_stamp - 1) / 1000.0;     // sec

   // m_OdomMsg.stamp = (double)(lscan.stamp - 2) / 1000.0;     // sec

    // std::cout << "rawScan.odom_data.time_stamp "<<rawScan.odom_data.time_stamp<<std::endl;



}

//
//   对输入的点云进行处理，滤除过近的点，并对Z坐标进行初始化。
//
void CLocalizationManager::FilterCloud(const ndt_oru::CPointCloud &cloud_in,
                                       ndt_oru::CPointCloud &cloud_filtered)
{
    const double varz = 0.05 / (double)INT_MAX;

    cloud_filtered.clear();
    cloud_filtered.resize(cloud_in.size());

    cloud_filtered.vecIntensity.clear();
    cloud_filtered.vecIntensity.resize(cloud_in.size());

    int effective_point_num = 0;  // By Sam test

    for (int i = 0; i < cloud_in.size(); i++)
    {
        const Eigen::Vector3d &pt = cloud_in[i];
        double d = pt.norm();
        if (std::isnan(pt(0)) || std::isnan(pt(1)) || std::isnan(pt(2)))
            continue;

        // 滤除距离太近/太远的点
        if (d < sensor_min_range || d > sensor_max_range)
        {
            cloud_filtered[i] << 0, 0, 0;
            cloud_filtered.vecIntensity[i] = 0;
        }
        else
        {
            cloud_filtered[i] = pt;
            cloud_filtered[i](2) += varz * rand();

            cloud_filtered.vecIntensity[i] = cloud_in.vecIntensity[i];

            effective_point_num++;
        }
    }
}

//
//   处理已收到的里程和激光扫描数据，并根据机器人速度对它们进行插补。
//
void CLocalizationManager::CollectOdomLaserData()
{
    // 取得当前时间
    m_tmStart = uLatestTime_;

    // 计算从取得里量程时到当前的时延
    long long int tmElapse = m_tmStart - m_odometry.m_dwTimeStamp;

    // 计算得到里程后的延时期间产生的姿态变化，并把它叠加到里程数据中去
    Eigen::Affine3d TFromOdom = vel_ * (tmElapse / 1000.0);
    odomAdjusted = m_odometry * TFromOdom;

    // 计算汇总的点云
    cloudAdjusted.clear();
    for (size_t i = 0; i < m_clouds.size(); i++)
    {
        CLaserScannerParam &ScannerParam = m_pScannerGroupParam->at(i);
        Eigen::Affine3d sensor_pose = PostureToAffine(ScannerParam.m_pst);

        // 计算点云数据的过期时间
        ndt_oru::CLabeledPointCloud &cloud = m_clouds[i];
        tmElapse = m_tmStart - cloud.m_dwTimeStamp;

        // 根据数据对传感器位姿进行插补
        Eigen::Affine3d TFromScan = vel_ * (tmElapse / 1000.0);
        sensor_pose = sensor_pose * TFromScan;

        // 将点云变换到机器人参考系内
        ndt_oru::transformPointCloudInPlace(sensor_pose, cloud);

        // 合成点云
        cloudAdjusted += cloud;
    }

    m_clouds.clear();
}

//
//   处理已收到的里程和激光扫描数据，并根据机器人速度对它们进行插补。
//
void CLocalizationManager::CollectOdomLaserData_PlayBack()
{
    // 取得当前时间
    m_tmStart = uLatestTime_;

    odomAdjusted = m_odometry;

    // 计算汇总的点云
    cloudAdjusted.clear();
    for (size_t i = 0; i < m_clouds.size(); i++)
    {
        CLaserScannerParam &ScannerParam = m_pScannerGroupParam->at(i);
        Eigen::Affine3d sensor_pose = PostureToAffine(ScannerParam.m_pst);

        // 计算点云数据的过期时间
        ndt_oru::CLabeledPointCloud &cloud = m_clouds[i];

        // 将点云变换到机器人参考系内
        ndt_oru::transformPointCloudInPlace(sensor_pose, cloud);

        // 合成点云
        cloudAdjusted += cloud;
    }

    m_clouds.clear();
}

//
//   接收到里程数据后的回调函数。
//   返回值：相对于上一次调用时的姿态变化量。
//
Eigen::Affine3d CLocalizationManager::getOdomMove(Eigen::Affine3d &odometry)
{
    Eigen::Affine3d Tm = last_odom.inverse() * odometry;
    last_odom = odometry;

    return Tm;
}

//
//    定位过程函数。
//    返回值：
//      NULL-定位失败;
//      非NULL-定位成功时的匹配结果信息
//

//by DQ 9.1 test
/*
CMatchInfo *CLocalizationManager::Localize(Eigen::Affine3d &estimatePose)
{
    std::cout << "By Sam: In Manager::Localize()" << std::endl;
    // 汇总已收到的所有里程、激光数据
    CollectOdomLaserData();

    // 计算里程姿态的变化量
    Eigen::Affine3d Tm = getOdomMove(odomAdjusted);

    // 根据校正后的里程姿态和里程变化量，估算出新的里程姿态
    Tnow_ = Tnow_ * Tm;

    // By Sam: For gridMethod
    CLocalizationMethod *method = NULL;
    std::shared_ptr<ProbabilityGrid> gridMap = ((CScanMatchMethod *)methods_->at(3))->map_;
    if (gridMap != nullptr)
    {
        method = methods_->at(3);

        int locModel = 1;

        // 进行定位
        int ok = method->LocalizeProc(locModel, Tnow_, cloudAdjusted, estimatePose);
        last_odom = odomAdjusted;
        if (ok)
        {
            // 评估定位质量
            float score;
            if (method->EvaluateQuality(score))
            {
                Tnow_ = estimatePose;          // 它与method->GetMatchInfo()->m_pst一致
                last_odom =  estimatePose;
                return method->GetMatchInfo();    // 成功，则不再尝试后续定位方法
            }
        }
    }


    // 取得对应于当前位姿的“定位指令”
    CPosture pst = AffineToPosture(Tnow_);
    CLocalizationInst *locInst = plan_->GetInstructions(pst);

    // 如果所有已定位区域中都不含有当前位姿，则定位失败
    if (locInst == NULL)
    {
        std::cout << "By Yu : In empty area!! Please add Loc area!!" << std::endl;
        return NULL;

    }

//    CLocalizationMethod *method = NULL;
    // 根据定位指令进行定位
    for (int i = 0; i < MAX_METHODS_PER_RECT; i++)
    {
        CLocalizationInst &inst = locInst[i];
        int methodId = inst.methodId_;

        // 方法编号应为非负数，且不应超出最大的定位方法编号
        if (methodId < 0 || methodId > NUM_LOCALIZATION_METHODS - 1)
            continue;

        // 采用指定的定位方法
        method = methods_->at(methodId);
        if (method == NULL)
            continue;

        // 应用指定的定位参数
        CLocalizationParam *param = static_cast<CLocalizationParam *>(inst.param_);
        if (param == NULL)
            continue;
        method->ApplyParam(param);

        CRectangle r = static_cast<CRectangle>(inst.localizationRect_);
        method->ApplyLocRect(&r);

        int locModel = 1;
        // 进行定位
        int ok = method->LocalizeProc(locModel, Tnow_, cloudAdjusted, estimatePose);
        last_odom = odomAdjusted;
        if (ok)
        {
            // 评估定位质量
            float score;
            if (method->EvaluateQuality(score))
            {
                Tnow_ = estimatePose;          // 它与method->GetMatchInfo()->m_pst一致
                last_odom =  estimatePose;
//                return method->GetMatchInfo();    // 成功，则不再尝试后续定位方法
                break;
            }
        }
    }

    return method->GetMatchInfo();
}*/

#if 0

CMatchInfo *CLocalizationManager::Localize(Eigen::Affine3d &estimatePose)
{
    // 汇总已收到的所有里程、激光数据
    CollectOdomLaserData();

    // 计算里程姿态的变化量
    Eigen::Affine3d Tm = getOdomMove(odomAdjusted);

    // 根据校正后的里程姿态和里程变化量，估算出新的里程姿态
    Tnow_ = Tnow_ * Tm;



    // By Sam: For gridMethod
    CLocalizationMethod *method = NULL;
    std::shared_ptr<ProbabilityGrid> gridMap = ((CScanMatchMethod *)methods_->at(3))->map_;

    // 取得对应于当前位姿的“定位指令”
    CPosture pst = AffineToPosture(Tnow_);
    CLocalizationInst *locInst = plan_->GetInstructions(pst);

    // 如果所有已定位区域中都不含有当前位姿，则定位失败

    // by lishen init slam method
    static bool bInitSlam = false;
    if(bInitSlam == false)
    {
        CSlamMethod *pSlamMethod  = (CSlamMethod *)(methods_->at(4));
        pSlamMethod->InitLocate();
        bInitSlam = true;
    }
    int methodId = -1;

    if(locInst==NULL)
        m_UnuseScanMatchMethod = 1;

    if (locInst != NULL && m_UnuseScanMatchMethod)
    {

        // 根据定位指令进行定位
        for (int i = 0; i < MAX_METHODS_PER_RECT; i++)
        {
            CLocalizationInst &inst = locInst[i];

            // 方法编号应为非负数，且不应超出最大的定位方法编号
            if (inst.methodId_ < 0 || inst.methodId_ > NUM_LOCALIZATION_METHODS - 1)
                continue;

            // 采用指定的定位方法
            method = methods_->at(inst.methodId_);
            if (method == NULL)
                continue;

            // 应用指定的定位参数
            CLocalizationParam *param = static_cast<CLocalizationParam *>(inst.param_);

            methodId = inst.methodId_;

            // by dq
            m_UnuseScanMatchMethod = ((CSlamMethod *)methods_->at(4))->ChangeToScanMatchMethod(methodId,true);


            method->ApplyParam(param);

            CRectangle r = static_cast<CRectangle>(inst.localizationRect_);
            method->ApplyLocRect(&r);

            int locModel = 1;

            // 进行定位
            int ok = method->LocalizeProc(locModel, Tnow_, cloudAdjusted, estimatePose);
            last_odom = odomAdjusted;
            if (ok)
            {
                // 评估定位质量
                float score;
                if (method->EvaluateQuality(score))
                {
                    std::string strLocFucntion("");
                    switch(method->type_)
                    {
                        case CMatchInfo::LOC_NDT:
                            strLocFucntion = "NDT";
                            break;
                        case CMatchInfo::LOC_FEATURE:
                            strLocFucntion = "Feature";
                            break;
                        case CMatchInfo::LOC_TEMPLATE:
                            strLocFucntion = "Template";
                            break;
                    }
                    Tnow_ = estimatePose;          // 它与method->GetMatchInfo()->m_pst一致
                    last_odom =  estimatePose;

                    CPosture local_pst = AffineToPosture(estimatePose);
                    std::cout << "Loc Success Types:"<< strLocFucntion <<
                                 " initPose: x_=" << pst.x << ",y_=" << pst.y << ",theta_=" << (pst.fThita / 3.14) * 180 << std::endl;
                    std::cout << "Loc Success Types:"<< strLocFucntion <<
                                 " locaPose: x_=" << local_pst.x << ",y_=" << local_pst.y <<
                                 ",theta_= " << (local_pst.fThita / 3.14) * 180 << std::endl;

                    break;    // 成功，则不再尝试后续定位方法
                }
            }

          /*  else
            {
                 estimatePose = Tnow_;
                 std::string strLocFucntion("");
                 int iType =  method->type_;
                 switch(method->type_)
                 {
                     case CMatchInfo::LOC_NDT:
                         strLocFucntion = "NDT";
                         break;
                     case CMatchInfo::LOC_FEATURE:
                         strLocFucntion = "Feature";
                         break;
                     case CMatchInfo::LOC_TEMPLATE:
                         strLocFucntion = "Template";
                         break;
                 }
                 std::cout << "Loc Fail Types:"<< strLocFucntion <<" initPose: x_=" << pst.x << ",y " << pst.y << ",theta_=" << pst.fThita<< std::endl;

            }*/
        }
    }
    else
    {
        // by lishen  无地图时采用栅格定位
        methodId = 3;
        method = methods_->at(3);
        if (gridMap != nullptr)
        {

            CScanMatchMethod *pScanMatcher = (CScanMatchMethod *)method;

            pScanMatcher->SetFastMatch(&(((CNdtMethod *)methods_->at(0))->m_fastMatcher));

            int locModel = 1;

            // 进行定位
            int ok = method->LocalizeProc(locModel, Tnow_, cloudAdjusted, estimatePose);
            if (ok)
            {
                // dq reset slam
                ((CSlamMethod *)methods_->at(4))->ResetOdom();
                ((CSlamMethod *)methods_->at(4))->SetLastPos(cloudAdjusted.m_dwTimeStamp,estimatePose);

                // 评估定位质量
                //float score;
               // if (method->EvaluateQuality(score))
                {
                    std::string strLocFucntion("");
                    strLocFucntion = "Gird";

                    Tnow_ = estimatePose;
                    CPosture local_pst = AffineToPosture(estimatePose);




                    std::cout << "Loc Success Types:"<< strLocFucntion <<
                                 " locaPose: x_=" << local_pst.x << ",y_=" << local_pst.y <<
                                 ",theta_=" << (local_pst.fThita / 3.14) * 180 << std::endl;
                   // return method->GetMatchInfo();    // 成功，则不再尝试后续定位方法
                }
            }
            else
            {
                // dq 11.28
                m_UnuseScanMatchMethod = true;
            }

        }



    }

    return method->GetMatchInfo();
}

#endif
CMatchInfo *CLocalizationManager::Localize(Eigen::Affine3d &estimatePose)
{
    std::string strLocType[NUM_LOCALIZATION_METHODS] = {"NDT","Feature","Template","GRID","SLAM"};

    // 汇总已收到的所有里程、激光数据
    CollectOdomLaserData();

    // 计算里程姿态的变化量
    Eigen::Affine3d Tm = getOdomMove(odomAdjusted);

    // 根据校正后的里程姿态和里程变化量，估算出新的里程姿态
    Tnow_ = Tnow_ * Tm;


    CLocalizationMethod *method = NULL;

    CScanMatchMethod *pScanMatcher = (CScanMatchMethod *)methods_->at(3);
    pScanMatcher->SetFastMatch(&(((CNdtMethod *)methods_->at(0))->m_fastMatcher));

    // 取得对应于当前位姿的“定位指令”
    CPosture pst = AffineToPosture(Tnow_);
    CLocalizationInst *locInst = plan_->GetInstructions(pst);

    // 如果所有已定位区域中都不含有当前位姿，则定位失败

    // by lishen init slam method
    static bool bInitSlam = false;
    if(bInitSlam == false)
    {
        CSlamMethod *pSlamMethod  = (CSlamMethod *)(methods_->at(4));
        pSlamMethod->InitLocate();
        bInitSlam = true;
    }
    int methodId = -1;
    int maxUseMethodNum = MAX_METHODS_PER_RECT;
    bool bWithSlam = false;
    bool bWithFeature = false;
    static CPosture pstFeatureFail;

   // 根据定位指令进行定位
    for (int i = 0; i < maxUseMethodNum; i++)
    {
        if(locInst != NULL)
        {
            CLocalizationInst &inst = locInst[i];

            // 方法编号应为非负数，且不应超出最大的定位方法编号
            if (inst.methodId_ < 0 || inst.methodId_ > NUM_LOCALIZATION_METHODS - 1)
                continue;

            // 采用指定的定位方法
            method = methods_->at(inst.methodId_);
            if (method == NULL)
                continue;

                // 应用指定的定位参数
            CLocalizationParam *param = static_cast<CLocalizationParam *>(inst.param_);
            if (param == NULL && inst.methodId_!=3)
                continue;

            CLocalizationInst &secondinst = locInst[1];
            if(secondinst.methodId_==4)
                bWithSlam = true;

            method->ApplyParam(param);

            CRectangle r = static_cast<CRectangle>(inst.localizationRect_);
            method->ApplyLocRect(&r);



            if(m_lastMethod == 4)
            {
                bool bflag = ((CSlamMethod *)methods_->at(4))->IsChangeToOtherMethod();
                if(!bflag && i==0)
                {
                    continue;
                }
            }
            methodId = inst.methodId_;

            if((inst.methodId_ == 3 && secondinst.methodId_ == 1)||(inst.methodId_ == 0 && secondinst.methodId_ == 1))
                bWithFeature = true;
        }
        else
        {
            methodId = 3;
            method = methods_->at(3);
            maxUseMethodNum = 1;
            method->ApplyParam(NULL);
        }

        // By Sam
        if( i == 1 && locInst[i].methodId_ == 1)
        {
            bool legMethod = ((CFeatureMethod *)methods_->at(1))->GetLocParam();

            if (!legMethod)
            {
                std::cout << "~~~~~~~~~~~~ By Sam: Second loc, but in leg, so not loc" << std::endl;
                continue;
            }
        }

        // 进行定位
        int locModel = 1;
        int ok = method->LocalizeProc(locModel, Tnow_, cloudAdjusted, estimatePose);
        string strLocResult;
        CPosture local_pst = AffineToPosture(estimatePose);

        last_odom = odomAdjusted;

        if (ok)
        {
            m_bFirstFeatureFail = true;
            // 评估定位质量
            float score;
            if (method->EvaluateQuality(score))  //??
            {
                Tnow_ = estimatePose;          // 它与method->GetMatchInfo()->m_pst一致
                last_odom =  estimatePose;
                strLocResult ="Loc Success! ";

                if(methodId != 4)
                {
                    ((CSlamMethod *)methods_->at(4))->ResetOdom();
                    ((CSlamMethod *)methods_->at(4))->SetLastPos(cloudAdjusted.m_dwTimeStamp,estimatePose);
                }
                std::cout << strLocResult<< "Types:"<< strLocType[methodId] <<
                             " localPose: x_=" << local_pst.x << ",y_=" << local_pst.y <<
                             ",theta_= " << (local_pst.fThita / 3.14) * 180 << std::endl;
                break;    // 成功，则不再尝试后续定位方法
            }
        }
        else
        {

            int matchresult = method->GetMatchInfo()->result_;

            if(matchresult != CMatchInfo::MATCH_TO_SINGLEFEATURE&& ! bWithFeature)
                  estimatePose = Tnow_;

            if(matchresult == CMatchInfo::MATCH_LOADMAP_FAILED)
                   std::cout <<"gridmap is null\n";

            if(bWithFeature && i==1)
            {
                if(m_bFirstFeatureFail)
                {
                    pstFeatureFail = pst;
                    m_bFirstFeatureFail = false;
                     ((CFeatureMethod *)methods_->at(1))->matchInfo_.result_ = CMatchInfo::MATCH_OK;
                }
                else
                {
                    std::cout<<"Local failed ----*************pstFeatureFail = "<<pstFeatureFail.x<<std::endl;
                    double dis = sqrt((pstFeatureFail.x-pst.x)*(pstFeatureFail.x-pst.x)+
                            (pstFeatureFail.y-pst.y)*(pstFeatureFail.y-pst.y));
                    CFeatureLocalizationParam *feature_param = static_cast<CFeatureLocalizationParam *>(locInst[1].param_);
                    if( dis > feature_param->m_MaxDisWithoutFeature)
                    {
                        ok = 3;
                        ((CFeatureMethod *)methods_->at(1))->matchInfo_.result_ = CMatchInfo::MATCH_FAIL;
                    }
                    else
                        ((CFeatureMethod *)methods_->at(1))->matchInfo_.result_ = CMatchInfo::MATCH_OK;

                    std::cout<<"*************dis = "<<dis<<std::endl;
                }
            }
            else if (!(bWithFeature&&i==0))
                m_bFirstFeatureFail = true;

             strLocResult ="Loc Fail! ";



        }
        std::cout <<"i = "<<i<< strLocResult<< "Types:"<< strLocType[methodId] <<
                     " initPose: x_=" << pst.x << ",y_=" << pst.y << ",theta_=" << (pst.fThita / 3.14) * 180 << std::endl;
        std::cout << "i = "<<i<< strLocResult<< "Types:"<< strLocType[methodId] <<
                     " localPose: x_=" << local_pst.x << ",y_=" << local_pst.y <<
                     ",theta_= " << (local_pst.fThita / 3.14) * 180 << std::endl;


    }

    CPosture lastpos = AffineToPosture(estimatePose);

    std::cout << "output pose  : " <<
                 " estimatePose: x_=" << lastpos.x << ",y_=" << lastpos.y <<
                 ",theta_= " << (lastpos.fThita / 3.14) * 180 << std::endl;
    lastpos = AffineToPosture(Tnow_);


    std::cout << "Tnow_ pose  : " <<
                 " Tnow_: x_=" << lastpos.x << ",y_=" << lastpos.y <<
                 ",theta_= " << (lastpos.fThita / 3.14) * 180 << std::endl;

    m_lastMethod = methodId;
    if(!bWithSlam)
         ((CSlamMethod *)methods_->at(4))->ResetOdom();

    return method->GetMatchInfo();
}


bool CLocalizationManager::LegLocalize(Eigen::Affine3d &estimatePose)
{
//    // Init
//    Eigen::Affine3d Tinit = estimatePose;

//    CLocalizationMethod *method = NULL;

//    // 取得对应于当前位姿的“定位指令”
//    CPosture pst = AffineToPosture(Tinit);
//    CLocalizationInst *locInst = plan_->GetInstructions(pst);

//    int locModel = 1;

//    if (locInst != NULL)
//    {
//        CLocalizationInst &inst = locInst[1];

//        // 判断是否为反光板方法
//        if (inst.methodId_ == 1)
//        {
//            // 采用指定的定位方法
//            method = methods_->at(inst.methodId_);
////            if (method == NULL)
////                return false;

//            CLocalizationParam *param = static_cast<CLocalizationParam *>(inst.param_);

////            if (param == NULL)
////                return false;

//            method->ApplyParam(param);

//            CRectangle r = static_cast<CRectangle>(inst.localizationRect_);
//            method->ApplyLocRect(&r);

//            bool legMethod = ((CFeatureMethod *)methods_->at(1))->GetLocParam();

//            if (!legMethod)
//            {
//                bool ok = method->LocalizeProc(locModel, Tinit, cloudAdjusted, estimatePose);

//                if (ok)
//                {
////                    legPose = estimatePose;
//                    std::cout << "By Sam: Legloc, success" << std::endl;
//                    return true;
//                }
//            }
//            else
//                ((CFeatureMethod *)methods_->at(1))->ReSetFeature();
//        }
//        else
//            ((CFeatureMethod *)methods_->at(1))->ReSetFeature();
//    }
//    else
//        ((CFeatureMethod *)methods_->at(1))->ReSetFeature();

//    estimatePose = PostureToAffine(0, 0, 0);

//    return false;


    // For TemplateLeg
    Eigen::Affine3d Tinit = estimatePose;

    CLocalizationMethod *method = NULL;

    // 取得对应于当前位姿的“定位指令”
    CPosture pst = AffineToPosture(Tinit);
    CLocalizationInst *locInst = plan_->GetInstructions(pst);

    int locModel = 1;

    bool legMethod = false;

    if (locInst != NULL)
    {
        CLocalizationInst &inst = locInst[1];

        // 判断是否为模板方法
        if (inst.methodId_ == 2 || inst.methodId_ == 1)
        {
            legMethod = true;
            // 采用指定的定位方法
            method = methods_->at(inst.methodId_);

            CLocalizationParam *param = static_cast<CLocalizationParam *>(inst.param_);

            method->ApplyParam(param);

            CRectangle r = static_cast<CRectangle>(inst.localizationRect_);
            method->ApplyLocRect(&r);

            bool legOK = method->LocalizeProc(locModel, Tinit, cloudAdjusted, estimatePose);

            if (legOK)
            {
                std::cout << "By Sam: Use LegMethod, Loc SUCCESS !!!" << std::endl;
                return true;
            }
            else
            {
                std::cout << "By Sam: Use LegMethod, Loc FAILE !!!" << std::endl;
                return false;
            }
        }
    }

    if (!legMethod)
    {
        std::cout << "By Sam: Not Use LegMethod, Reset LegMethod!" << std::endl;

        estimatePose = PostureToAffine(0, 0, 0);
        ((CFeatureMethod *)methods_->at(1))->ReSetMethod();
       // ((CTemplateMethod *)methods_->at(2))->ReSetMethod();

        return false;
    }
}


// By Sam
//
//    定位过程函数。
//    返回值：
//      NULL-定位失败;
//      非NULL-定位成功时的匹配结果信息
//
CMatchInfo *CLocalizationManager::PlayBack(Eigen::Affine3d &estimatePose)
{
    // 汇总已收到的所有里程、激光数据
    CollectOdomLaserData_PlayBack();

    // 计算里程姿态的变化量
    Eigen::Affine3d Tm = getOdomMove(odomAdjusted);

    // 根据校正后的里程姿态和里程变化量，估算出新的里程姿态
    Tnow_ = Tnow_ * Tm;

    //Tnow_ = odomAdjusted;

//    estimatePose = Tnow_;

    return NULL;    // 定位失败
}

//
//   从二进制文件装载数据。
//
bool CLocalizationManager::LoadBinary(FILE *fp, string filename,int floor, bool bChangeFloor)
{
    // 先读入各种导航方法所使用的地图
    if (!methods_->LoadBinary(fp,filename,floor,bChangeFloor))
        return false;


    // 再读入全局应用区域表
    plan_->Clear();

    // .jff格式不支持全局区域表，需要跳过(临时措施!)
    if (!readJffFormat)
    {
        if (!plan_->LoadBinary(fp))
            return false;
    }

    return true;
}

//
//   将数据保存到二进制文件。
//
bool CLocalizationManager::SaveBinary(FILE *fp, string filename)
{
    // 写入地图
    if (!methods_->SaveBinary(fp,filename))
        return false;

    // 写入全局应用区域表
    if (!plan_->SaveBinary(fp))
        return false;

    return true;
}
bool CLocalizationManager::SaveFeatureBinary(FILE *fp, string filename)
{
    // 写入地图
    if (!methods_->SaveFeatureBinary(fp,filename))
        return false;

    // 写入全局应用区域表
    if (!plan_->SaveBinary(fp))
        return false;

    return true;
}

void CLocalizationManager::ClearScannerParam()
{
    m_pScannerGroupParam->clear();
}
bool CLocalizationManager::AddScannerParam(CLaserScannerParam param)
{
    m_pScannerGroupParam->push_back(param);
}
