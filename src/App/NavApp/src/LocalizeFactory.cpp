//
//   The interface of class "CLocalizeFactory".
//
#include "LocalizeFactory.h"
#include "Tools.h"
#include "RoboLocClnt.h"
#include "BaseOdometry.h"
#include "SensorFamily.h"
#include "RawMap.h"
#include "AutoOutPutBlackBox.h"
#include "blackboxhelper.hpp"
#include "ParameterObject.h"
#include "AffinePosture.h"
#include "ScanMatchMethod.h"   // By Sam Test
#include "HttpCommunicationGlobalData.h"
//by dq muban
#include "LineElement.h"
#include "StaticObjects.h"
#include "CircleElement.h"
#include "SlamMethod.h"
#include "type.h"
#include "navigation.h"
#include "DDSTask.h"



#pragma GCC push_options
#pragma GCC optimize ("O0")

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

#define NDT_LOCALIZE_MIN_CTRL_CYCLE 10 //10ms
#define NDT_LOCALIZE_MAX_CTRL_CYCLE 50 //50ms
#define FEATURE_LOCALIZE_MIN_CTRL_CYCLE 10 //10ms
#define FEATURE_LOCALIZE_MAX_CTRL_CYCLE 50 //50ms

#define  MIN_QUALITY_LEVEL          30 //30%
#define  MIN_MATCH_NUM              15
#define  MIN_FEATURE_MATCH_NUM      3
#define  LOC_GOOD_KEEP_DIST         0.1 //0.1m
#define  LOC_ERROR_TOLERATE_DIST    0.5 //0.5m
#define  LOC_FAIL_TOLERATE_DIST     0.3 //0.3m

//for test once initpos///
bool b_initpos = false;


// By Sam test
Eigen::Affine3d legPose;
int legOK;

extern bool readJffFormat;

//#define INDEPENDENT_LOAD_MAP  //每一个定位方法独立loadmap
//////////////////////////////////////////////////////////////////////////////
//   The support routine of ndt localization.
void* LocalizeSupportProc(LPVOID pParam)
{
    //FIXME:需要根据执行主机的实际CPU配置并行计算.
    cpu_set_t maskLoc;
    CPU_ZERO(&maskLoc);
    CPU_SET(4, &maskLoc);
    CPU_SET(5, &maskLoc);
    pthread_setaffinity_np(pthread_self(), sizeof(maskLoc), &maskLoc);
    usleep(1e6);
    int time_diff = 0;
    int ctrl_cycle = 0;
    robo::CLocalizeFactory* pLocFac = reinterpret_cast<robo::CLocalizeFactory*>(pParam);
    while (WaitForSingleObject(pLocFac->m_hLocKillThread, 0) != WAIT_OBJECT_0)
    {
        //主要执行函数
        pLocFac->LocalizationProc();
        //根据执行时间,灵活设置休眠时间
        time_diff = static_cast<int>(pLocFac->m_LocEndTime.load() - pLocFac->m_LocCurTime.load());
        if(time_diff < 0){
            time_diff = 0;
        }

        ctrl_cycle = NDT_LOCALIZE_MAX_CTRL_CYCLE - time_diff;
        if(ctrl_cycle < 0){
            ctrl_cycle = 0;
        }
        ctrl_cycle = std::min(ctrl_cycle, NDT_LOCALIZE_MAX_CTRL_CYCLE);
        ctrl_cycle = std::max(ctrl_cycle, NDT_LOCALIZE_MIN_CTRL_CYCLE);

        Sleep(ctrl_cycle);
    }

    SetEvent(pLocFac->m_hLocThreadDead);
    pthread_exit(NULL);

    return NULL;
}
//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CLocalizeFactory".
namespace robo {

CLocalizeFactory::CLocalizeFactory()
{
    localize_method = 0;    // 默认ndt算法
    m_LocCurTime = 0;
    m_LocPreTime = 0;
    m_LocEndTime = 0;
    m_bLocFirstTime = true;
    m_bLocStarted = false;
    simulate_ = false;
    m_nCurSlamStep = 0;

    m_PosNow.Update();    // By Sam Add

    m_bInitLocSuccess = false; //lishen

    m_addPosTimes = 0;
    m_xVar = 0.0;
    m_yVar = 0.0;
    m_fThitaVar = 0.0;
    m_nSetPosCounter = 0;
    m_bLoadMapSuccess = true;

    m_UnuseScanMatchMethod = true;//dq 11.28
    m_curFloorNo = 0;
    m_bFlagCameraLoc = false;

    m_bRecordImg = false;

    m_topVisionMode = 0;
    m_topVisionDefaultMode = 0;
}

CLocalizeFactory::~CLocalizeFactory()
{
}

bool CLocalizeFactory::Create()
{
    Clear();

    methods_ = new CLocalizationMethods;
    if (methods_ == NULL|| !methods_->Create())
        return false;

    // 在此设置CLocalizationRect类的静态成员methods_
    CLocalizationRect::SetLocalizationMethods(methods_);

    plan_ = new CLocalizationPlan;
    if (plan_ == NULL)
        return false;

    return true;
}
int CLocalizeFactory::StartRecordImage()
{
    std::cout<<"*******************StartRecordImage******************"<<std::endl;

    if(!m_bFlagCameraLoc)
        return RECORDIMG_DISABLE;
    auto pRecTvImg = CRecTopvisionImageSingleton::GetInstance();

    int res = pRecTvImg->StartRecord();
    if(res == RECORDIMG_START_SUCCESS )
    {
        m_bRecordImg = true;

    }
    return res;
}

int CLocalizeFactory::StopRecordImage()
{
    if(!m_bFlagCameraLoc)
        return RECORDIMG_DISABLE;

    if(m_bRecordImg)
    {
        auto pRecTvImg = CRecTopvisionImageSingleton::GetInstance();

        int res =pRecTvImg->StopRecord();

        m_bRecordImg = false;

        return res;

    }
    return RECORDIMG_ISNOT_RECORDING;
}


void CLocalizeFactory::SetPose(double init_x, double init_y, double init_theta, unsigned long long init_raw_time)
{
    std::cout << "By Sam: Set pose, x = " << init_x << ", y = " << init_y <<
                 ", theta = " << init_theta << std::endl;
#if defined USE_BLACK_BOX
    FILE_BlackBox(LocBox, "SetPos:", init_x,",", init_y, ",", init_theta);
#endif
        CPosture initpos(init_x, init_y, init_theta);
        Eigen::Affine3d initpos_affine = PostureToAffine(initpos);
//        CLocalizationManager::SetPose(initpos_affine);    // By Sam Delete, In CLocalizeFactory no need use CLocalizationManager


//        CStampedPos    PosNow;
//        PosNow.x = init_x;
//        PosNow.y = init_y
//        PosNow.fThita = init_theta;
//        PosNow.m_dwTimeStamp = init_raw_time;

        // By Sam Add
        m_PosNow.x = init_x;
        m_PosNow.y = init_y;
        m_PosNow.fThita = init_theta;
        m_PosNow.m_dwTimeStamp = init_raw_time;

        // 重置缓存位姿
        m_nSlideDataCount = 0;
        m_RecentPoses.clear();
        m_PosNowFiltered = m_PosNow;

        m_addPosTimes = 0;
        m_xVar = 0.0;
        m_yVar = 0.0;
        m_fThitaVar = 0.0;

        if(m_bFlagCameraLoc  && (!m_bRecordImg))
        {
            m_nSlideDataCount_Cam = 0;
            m_RecentPoses_Cam.clear();
            m_PosNowFiltered_Cam = m_PosNow;
        }
}

void CLocalizeFactory::SetPose(CStampedPos locPos)
{
    CPosture initpos(locPos.x, locPos.y, locPos.fThita);
    Eigen::Affine3d initpos_affine = PostureToAffine(initpos);
    CLocalizationManager::SetPose(initpos_affine);
}

CStampedPos CLocalizeFactory::GetCurPose(void)
{
   std::lock_guard<std::mutex> lock(loc_mtx);

   return m_PosNow;
}



bool CLocalizeFactory::LoadMap(string map_name)
{

    std::lock_guard<std::mutex> lock(loc_mtx);

    FILE* fp = fopen((WORK_PATH+map_name+".map").c_str(), "rb");
    if (!fp)
        return false;

    // 先读取文件的版本和日期
    if (fread(version_, sizeof(char), 8, fp) != 8 || fread(date_, sizeof(char), 6, fp) != 6)
    {
        fclose(fp);
        return false;
    }
    // 初始化所有的定位方法

    CLocalizationRect::SetLocalizationMethods(methods_);
    LoadBinary(fp,WORK_PATH,0, false);
    fclose(fp);
    //dq 10-19
    //readJffFormat = true;

    m_bLoadMapSuccess = true;
   return  true;

}

bool CLocalizeFactory::LoadFloorMap(string map_name,int floor)
{
    m_bLoadMapSuccess = false;

    map_name = map_name+"_"+to_string(floor);


    FILE* fp = fopen((WORK_PATH+map_name+".map").c_str(), "rb");
    if (!fp)
        return false;
    // 先读取文件的版本和日期
    if (fread(version_, sizeof(char), 8, fp) != 8 || fread(date_, sizeof(char), 6, fp) != 6)
    {
        fclose(fp);
        return false;
    }
    // 初始化所有的定位方法
    string fileName = WORK_PATH;

    fileName = fileName+to_string(floor);

    CLocalizationRect::SetLocalizationMethods(methods_);
    LoadBinary(fp,fileName,floor, true);  //?????
    fclose(fp);
    //dq 10-19
    //readJffFormat = true;

    m_curFloorNo = floor;
    m_bLoadMapSuccess = true;

   return  true;
}

bool CLocalizeFactory::Initialize(bool simulate)
{


    Create();
    simulate_ = simulate;
    LoadMap();
    if(simulate_)
    {
        m_nCurSlamStep = 0 ;
    }
    else
    {
        // 设置激光传感器参数
        auto pAFamily = SensorFamilySingleton::GetInstance();
        sensor::CSensorData* sensor_data = NULL;
        ScannerParam_.clear();
        for(unsigned int m = 0; m< pAFamily->GetCount(); m++) {
            sensor_data = pAFamily->GetSensorData(m);
            if(sensor_data && sensor_data->parm) {
                ScannerParam_.push_back(*(sensor_data->parm));
            }
        }
        SetScannerGroupParam(&(ScannerParam_));
        InitializeFilterParms();
        // 初始化定位评估参数
        m_Evaluate.min_quality_level = MIN_QUALITY_LEVEL;
        m_Evaluate.min_match_num = MIN_MATCH_NUM;
        m_Evaluate.min_feature_match_num = MIN_FEATURE_MATCH_NUM;
        m_Evaluate.good_keep_dist = LOC_GOOD_KEEP_DIST;
        m_Evaluate.error_tolerate_dist = LOC_ERROR_TOLERATE_DIST;
        m_Evaluate.fail_tolerate_dist = LOC_FAIL_TOLERATE_DIST;
        auto pParameterObject = ParameterObjectSingleton::GetInstance();
        pParameterObject->GetParameterValue("Evaluate_MinQualityLevel", m_Evaluate.min_quality_level);
        pParameterObject->GetParameterValue("Evaluate_MinMatchNum", m_Evaluate.min_match_num);
        pParameterObject->GetParameterValue("Evaluate_LocGoodKeepDist", m_Evaluate.good_keep_dist);
        pParameterObject->GetParameterValue("Evaluate_LocErrorTolerateDist", m_Evaluate.error_tolerate_dist);
        pParameterObject->GetParameterValue("Evaluate_LocFailTolerateDist", m_Evaluate.fail_tolerate_dist);

        // By Sam: For diagnosis tool
        GData::getObj().evaluate_uG = m_Evaluate.min_quality_level;
        GData::getObj().evaluate_uN = m_Evaluate.min_match_num;

        //　初始化诊断参数
        m_Diagnosis.Initialize();
        pParameterObject->GetParameterValue("Diagnosis_UseCoreDump", m_Diagnosis.use_core_dump);
        pParameterObject->GetParameterValue("Diagnosis_AutoSaveLog", m_Diagnosis.auto_save_log);
        pParameterObject->GetParameterValue("Diagnosis_PublishPointCloud", m_Diagnosis.publish_point_cloud);
        pParameterObject->GetParameterValue("Diagnosis_PublishNdtCell", m_Diagnosis.publish_ndt_cell);
        pParameterObject->GetParameterValue("Diagnosis_PublishReflector", m_Diagnosis.publish_reflector);

        pParameterObject->GetParameterValue("TopVision_Enable", m_bFlagCameraLoc);
        pParameterObject->GetParameterValue("TopVision_DefaultFusionMode", m_topVisionDefaultMode);


        std::cout<<"m_bFlagCameraLoc = "<<m_bFlagCameraLoc<<std::endl;

        if(m_Diagnosis.publish_point_cloud){
            LCMTask::GetLcmInstance().LCMInit();
            DDSTask::GetDDSInstance().DDSInit();
        }
    }

    return true;
}

bool CLocalizeFactory::StopLocalize()
{
    if (!m_bLocStarted){
        return false;
    }

    CSlamMethod *pSlamMethod  = (CSlamMethod *)(methods_->at(4));
    pSlamMethod->StopSlamLocate();


    SetEvent(m_hLocKillThread);
    WaitForSingleObject(m_hLocThreadDead, 5000);
    PthreadJoin(m_pLocThread);

    if (m_hLocKillThread != NULL){
        CloseHandle(m_hLocKillThread);
        m_hLocKillThread = NULL;
    }

    if (m_hLocThreadDead != NULL){
        CloseHandle(m_hLocThreadDead);
        m_hLocThreadDead = NULL;
    }

    m_pLocThread = 0;
    m_bLocStarted = false;
    return true;
}


bool CLocalizeFactory::TransformRawScanToLaserMsg(const sensor::CRawScan& rawScan)
{


    CSlamMethod *pSlamMethod  = (CSlamMethod *)(methods_->at(4));


    pSlamMethod->m_scanMsg.sensor_id = (std::string)("scan0");
    pSlamMethod->m_scanMsg.angle_increment = m_pScannerGroupParam->at(0).m_fReso;
    pSlamMethod->m_scanMsg.angle_max = m_pScannerGroupParam->at(0).m_fEndAngle;;
    pSlamMethod->m_scanMsg.angle_min = m_pScannerGroupParam->at(0).m_fStartAngle;
    pSlamMethod->m_scanMsg.range_max = m_pScannerGroupParam->at(0).m_fMaxRange;
    pSlamMethod->m_scanMsg.range_min = m_pScannerGroupParam->at(0).m_fMinRange;
    pSlamMethod->m_scanMsg.scan_time = 0.02;   //50hz
    pSlamMethod->m_scanMsg.time_increment = 0.000005;

    Pose lasertf;
    lasertf.x       = m_pScannerGroupParam->at(0).m_pst.x;
    lasertf.y       = m_pScannerGroupParam->at(0).m_pst.y;
    lasertf.theta   = m_pScannerGroupParam->at(0).m_pst.fThita ;

    pSlamMethod->m_scanMsg.laser_tf = lasertf;

    pSlamMethod->m_scanMsg.ranges.clear();


    //By yu.protected null pointcloud.
    if(rawScan.point_cloud.size() <= 0 || rawScan.point_cloud[0] == NULL ){
        return false;
    }

    for(auto r: rawScan.point_cloud[0]->distance)
    {
        pSlamMethod->m_scanMsg.ranges.push_back(float(r)/1000.0);  //???
    }

    pSlamMethod->m_scanMsg.stamp = rawScan.point_cloud[0]->timestamp_raw;

    pSlamMethod->m_odomMsg.pos.x      = rawScan.odom_data.global_pst.x;
    pSlamMethod->m_odomMsg.pos.y      = rawScan.odom_data.global_pst.y;
    pSlamMethod->m_odomMsg.pos.theta  = rawScan.odom_data.global_pst.fThita;

    // std::cout << "lscan.stamp "<<lscan.stamp<<std::endl;
    // std::cout << "odom 1 "<< odom.pos.x<<" "<<odom.pos.y<<" "<<180.0*odom.pos.theta/3.1415<<std::endl;
    // odom.stamp = (double)(rawScan.odom_data.time_stamp - 1) / 1000.0;     // sec

    pSlamMethod->m_odomMsg.stamp = (double)(pSlamMethod->m_odomMsg.stamp - 2) / 1000.0;     // sec




}
//
//   By Sam: For Local use (处理已收到的里程和激光扫描数据，并根据机器人速度对它们进行插补。)
//
void CLocalizeFactory::CollectLaserCloud()
{
    long long int tmStart = GetTickCount();

    //　分配空间
    cloudAdjusted.clear();
    int sumPointSize(0);

    //std::cout<<"m_clouds.size(): "<<m_clouds.size()<<std::endl;
    int temp_cloudssize =1;
    //for (size_t i = 0; i < m_clouds.size(); i++)
    for (size_t i = 0; i < temp_cloudssize; i++)
    {
        sumPointSize += m_clouds[i].size();
    }
    cloudAdjusted.reserve(sumPointSize);

    // 根据时间变化和传感器安装方式更新点云坐标
   // for (size_t i = 0; i < m_clouds.size();  i++)
    for (size_t i = 0; i < temp_cloudssize;  i++)
    {
//        // 点云ID不存在
//        if(m_clouds[i].m_nScannerId >= m_pScannerGroupParam->size())
//            continue;

//        // 点云时间滞后
//        if(m_clouds[i].m_dwTimeStamp <= (m_PosNow.m_dwTimeStamp - 200) )
//           continue;
//        // 传感器安装位姿
//        CLaserScannerParam& ScannerParam = m_pScannerGroupParam->at(m_clouds[i].m_nScannerId);
//        Eigen::Affine3d sensor_pose = PostureToAffine(ScannerParam.m_pst);



//        // 获取点云时间到当前时间的里程
//        CPosture odomTrans;
//        auto pOdometry = BaseOdomSingleton::GetInstance();
//        pOdometry->GetLocalOdomTrans(m_clouds[i].m_dwTimeStamp, tmStart, odomTrans);
//        Eigen::Affine3d TFromScan = PostureToAffine(odomTrans);
//        // 增量变化先于传感器安装位姿发生
//        sensor_pose = TFromScan.inverse() * sensor_pose;

//        std::cout << "By Sam: Before match, Sensor_pose.x = " << sensor_pose.translation().x() <<
//                     ", y = " << sensor_pose.translation().y() << ", thita = " << sensor_pose.rotation().eulerAngles(0, 1, 2)(2);

//        // 将点云变换到机器人参考系内
//        ndt_oru::transformPointCloudInPlace(sensor_pose, m_clouds[i]);

//        // 合成点云
//        cloudAdjusted += m_clouds[i];

        ndt_oru::CLabeledPointCloud &cloud = m_clouds[i];

        // 点云ID不存在
        if(cloud.m_nScannerId >= m_pScannerGroupParam->size())
            continue;
        // 点云时间滞后
        if(cloud.m_dwTimeStamp <= (m_PosNow.m_dwTimeStamp - 200) )
           continue;
        // 传感器安装位姿
        CLaserScannerParam& ScannerParam = m_pScannerGroupParam->at(cloud.m_nScannerId);
        Eigen::Affine3d sensor_pose = PostureToAffine(ScannerParam.m_pst);

        // 获取点云时间到当前时间的里程
        CPosture odomTrans;
        auto pOdometry = BaseOdomSingleton::GetInstance();
        pOdometry->GetLocalOdomTrans(cloud.m_dwTimeStamp, tmStart, odomTrans);
        Eigen::Affine3d TFromScan = PostureToAffine(odomTrans);
        // 增量变化先于传感器安装位姿发生
        sensor_pose = TFromScan.inverse() * sensor_pose;

        // 将点云变换到机器人参考系内
        ndt_oru::transformPointCloudInPlace(sensor_pose, cloud);

        // 合成点云
        cloudAdjusted += cloud;
        cloudAdjusted.m_dwTimeStamp = cloud.m_dwTimeStamp;

    }

    m_clouds.clear();
}


bool CLocalizeFactory::HandleSetPoseCmd()
{
    unsigned char init_map_id = 0;
    double init_x = 0;
    double init_y = 0;
    double init_theta = 0;
    unsigned long long init_raw_time = 0;
    short setbypad = 0;
    bool new_init_pos = false;

    auto pRoboClnt = RoboClntSingleton::GetInstance();
    if(pRoboClnt){
        new_init_pos = pRoboClnt->GetRoboInitPosition(init_map_id, init_x, init_y, init_theta, init_raw_time, setbypad);
    }
    if(new_init_pos==0)
        return false;

    if(setbypad == 0)
    {
        //change map
        //std::cout<<"LoadMap"<<(int)init_map_id<<std::endl;
        if(m_curFloorNo!=init_map_id)
        {
             SetPose(init_x, init_y, init_theta, init_raw_time);
             CScanMatchMatchInfo results;

             results.type_ = 3;
             results.result_ = CMatchInfo::MATCH_LOADING_MAP;

             const CPosture pos(0,0,0);
             const CTimeStamp stamp(m_LocCurTime.load());
             const CStampedPos leg_pose(pos, stamp);

             DealWithLocResult(m_PosNow,leg_pose,&results);

            if(LoadFloorMap("FeatureMap",(int)init_map_id))
            {
                //load map success

                ((CScanMatchMethod *)methods_->at(3))->SetPose_byPad(false);
                SetPose(init_x, init_y, init_theta, init_raw_time);
                m_bInitLocSuccess = false;  // ?????  featuremap
                m_nSetPosCounter = 0;
                std::cout<<"LoadMap success**********!!!!!"<<(int)init_map_id<<std::endl;
                #ifdef USE_BLACK_BOX
                    FILE_BlackBox(LocBox, "***Load map success!  floor no =  ",(int) init_map_id );
                #endif
                return true;
            }
            else
            {
                std::cout<<"LoadMap failed**********!!!!!"<<std::endl;
                #ifdef USE_BLACK_BOX
                    FILE_BlackBox(LocBox, "***Load map failed!  floor no =  ",(int) init_map_id );
                #endif

                results.type_ = 0;
                results.result_ = CMatchInfo::MATCH_LOADMAP_FAILED;
                DealWithLocResult(m_PosNow,leg_pose,&results);
                 return false;
            }
        }
        else
        {

            ((CScanMatchMethod *)methods_->at(3))->SetPose_byPad(false);
            if(pow(init_x-m_PosNow.x,2)+pow(init_y-m_PosNow.y,2) > 0.0025 || fabs(init_theta-m_PosNow.fThita) > 5*3.14/180)
            {
                std::cout<<"!!!!!!!!!!!!!SET POSE BY AUTO!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                SetPose(init_x, init_y, init_theta, init_raw_time);
                m_nSetPosCounter = 0;
                #ifdef USE_BLACK_BOX
                    FILE_BlackBox(LocBox, "*********************SET POSE BY RC AFTER TURN AROUND*****************: ", init_x , ",", init_y,",",
                                  (init_theta / 3.14) * 180);
                #endif
            }
            else
                std::cout<<"!!!!!!!!!!!!!!!!!!!!Ignore the Transformation!!!!!!!!!!!!!!!!!"<<std::endl;

             return true;
        }
    }
    else if(setbypad == 1)
    {
        std::cout<<"!!!!!!!!!!!!!!!!SET POSE BY USER!!!!!!!!!!!!!!!!!!!!"<<std::endl;

            ((CScanMatchMethod *)methods_->at(3))->SetPose_byPad(true);
            SetPose(init_x, init_y, init_theta, init_raw_time);
            m_reloc = true;
            m_bInitLocSuccess = false;
            m_nSetPosCounter = 0;
             return true;

    }
	
    else if(setbypad == 2)
    {
        // By Sam: Use For ReSet LegMethod !
        std::cout<<"!!!!!!!!!!!!!!!!SET POSE BY USER!!!!!!!!!!!!!!!!!!!!"<<std::endl;

        ((CFeatureMethod *)methods_->at(1))->ReSetMethod();
        ((CTemplateMethod *)methods_->at(2))->ReSetMethod();

#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "AGV ReSet LegMethod !!!!");
#endif

        return true;
    }

    return true;
}


bool CLocalizeFactory::LocalizationProc()
{
    unsigned long long timeNow = GetTickCount();

    std::lock_guard<std::mutex> lock(loc_mtx);

    bool filter_enable = false;
    sensor::COdometryData odom_data;
    sensor::CRawScan pRawScan;
    bool bActiveCloud = true;
    auto pRoboClnt = RoboClntSingleton::GetInstance();
    auto pRawMap = RawMapSingleton::GetInstance();
    std::shared_ptr<sensor::CRawPointCloud> pRawCloud;

    bool setpose_bypad = false;
    ((CScanMatchMethod *)methods_->at(3))->SetPose_byPad(setpose_bypad);

    sensor::CRawScan firstScan;

    if(simulate_)
    {
    }
    else
    {
        if(m_bLocFirstTime){
            m_LocPreTime = timeNow;
            m_bLocFirstTime = false;
        }
        m_LocCurTime = timeNow;
        // odometry
        CPosture odomTrans;

        mapping::StVelocity base_vel;
        auto pOdometry = BaseOdomSingleton::GetInstance();
        pOdometry->GetLocalOdomTrans(m_LocPreTime.load(), m_LocCurTime.load(), odomTrans);
        odom_data.odom_flag = pOdometry->GetOdomFlag();
        odom_data.time_stamp = m_LocCurTime.load();
        base_vel = pOdometry->GetBaseVelocity();
        odom_data.velocity.fXLinear = base_vel.Vx / 1000.0;
        odom_data.velocity.fYLinear = base_vel.Vy / 1000.0;
        odom_data.velocity.fAngular = base_vel.Vtheta / 1000.0;
        odom_data.local_pst = odomTrans;

//    #ifdef USE_BLACK_BOX
//        FILE_BlackBox(LocBox, "Version:", VER_IN_BLACKBOX);
//    #endif

        // point cloud
        auto pAFamily = SensorFamilySingleton::GetInstance();
        unsigned int  nStartTime = 0;
        sensor::CRawScan pFrontScan_;
        if(pRawMap->GetFrontRawScan(pFrontScan_)) {
            nStartTime = static_cast<unsigned int>(pFrontScan_.odom_data.time_stamp - 200);
        }
        pRawMap->SetStartTime(nStartTime);
        std::lock_guard<std::mutex> lock(ndt_mtx);
        for(unsigned int m = 0; m< pAFamily->GetCount(); m++) {
            if((!pAFamily->GetState(m)) || (pAFamily->GetType(m) == LEIMOUF30)){
                continue;
            }
            if(!pAFamily->GetRawPointCloud(m, pRawCloud)) {
                bActiveCloud = false;
            }
            // 更新点云数据
            OnReceiveLaserRawScan(m, pRawCloud);
            // 更新数据集
            pRawScan.PushBackPointCloud(pRawCloud);

            if(m==0)
                firstScan.PushBackPointCloud(pRawCloud);
        }
        if(pRawScan.point_cloud.size() <= 0 || firstScan.point_cloud.size() <= 0 ||firstScan.point_cloud[0] == NULL)
        {
            std::cout<<"By yu : Point cloud is Null!!!!! Please Check Laser!!!! "<<std::endl;
        }

        // 更新里程计变换量
        Eigen::Affine3d odom = PostureToAffine(odomTrans);
        CTimeStamp stamp(m_LocCurTime.load());
        ndt_oru::CStampedAffine stamp_pos(odom,stamp);
        OnReceiveOdometry(stamp_pos);
        // 设置全局初始位姿

        if(!b_initpos)
        {
            SetPose(0.0f, 0.0f, 0.0f, GetTickCount());
            if(m_bFlagCameraLoc && (!m_bRecordImg))
            {
                 // dq VISION set initpos
                 CLocDataSet init_pos_cam;
                 init_pos_cam.Initialize();
                 pRoboClnt->SetInitPosition_Cam(init_pos_cam.m_PositionData,0);
            }
            b_initpos = true;
            m_bInitLocSuccess = false;
            m_nSetPosCounter = 0;
        }

        HandleSetPoseCmd();

        pFrontScan_.Clear();
    }

    // by lishen
    static bool bSendPointCloud = true;
    if(m_clouds.size()>=2 && (!bSendPointCloud))
    {
        ndt_oru::CLabeledPointCloud bottomcloud(m_clouds[1]);
        CLaserScannerParam& ScannerParam = m_pScannerGroupParam->at(1);
        Eigen::Affine3d sensor_pose = PostureToAffine(ScannerParam.m_pst);

        // 将点云变换到机器人参考系内
        ndt_oru::transformPointCloudInPlace(sensor_pose, bottomcloud);
        LCMTask::GetLcmInstance().SendBottomLaserPoindCloud(m_PosNow,bottomcloud);

    }


    //定位结果
    long long int tmStart = GetTickCount();
    Eigen::Affine3d estimatePose;  //定位结果
    // dq VISION get cam_pos
    STPositionData CamPose_Data;    
    Eigen::Affine3d estimatePose_cam;


    if(m_bFlagCameraLoc&& (!m_bRecordImg))
    {
        pRoboClnt->GetLocPosition_Cam(CamPose_Data);
        estimatePose_cam = PostureToAffine((float)CamPose_Data.nX/1000, (float)CamPose_Data.nY/1000, (float)CamPose_Data.nTheta/1000);
    }


    // 从上次定位结果到现在里程变化
    CPosture odomTrans_one;
    auto pOdometry = BaseOdomSingleton::GetInstance();
    pOdometry->GetLocalOdomTrans(m_PosNow.m_dwTimeStamp, tmStart, odomTrans_one);
    Eigen::Affine3d TFromScan_one = PostureToAffine(odomTrans_one);
   // std::cout << "By Sam Test: TFromScan_one (" << TFromScan_one.translation().x() <<
   //              ", " << TFromScan_one.translation().y() <<
   //              ", " << (TFromScan_one.rotation().eulerAngles(0, 1, 2)(2) / 3.14) * 180 <<
   //              ") " <<m_PosNow.m_dwTimeStamp<<", "<< tmStart<<std::endl;
    estimatePose = PostureToAffine(m_PosNow.x, m_PosNow.y, m_PosNow.fThita);
    //std::cout << "By Sam Test: Before TF(" << estimatePose.translation().x() <<
    //             ", " << estimatePose.translation().y() <<
    //             ", " << (estimatePose.rotation().eulerAngles(0, 1, 2)(2) / 3.14) * 180 <<
    //             ")" << std::endl;
    estimatePose = estimatePose * TFromScan_one;
    Eigen::Affine3d initPos = estimatePose;
	
    std::cout << "By Sam Test: InitPose_1 (" << estimatePose.translation().x() <<
                 ", " << estimatePose.translation().y() <<
                 ", " << (estimatePose.rotation().eulerAngles(0, 1, 2)(2) / 3.14) * 180 <<
                 ")" << std::endl;

    #ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "estimatePos: ", estimatePose.translation().x() , ", ", estimatePose.translation().y(),", ",
                  (estimatePose.rotation().eulerAngles(0, 1, 2)(2) / 3.14) * 180);
    #endif
    long long int loc1Start = GetTickCount();

    CPosture odom_pst;
    bool bRet = pOdometry->GetOdomPosture(loc1Start, odom_pst);

    firstScan.odom_data.global_pst = odom_pst;

    bool bSuccess = TransformRawScanToLaserMsg(firstScan);


    if(m_bRecordImg)
    {
        auto pRecTvImg = CRecTopvisionImageSingleton::GetInstance();
        pRecTvImg->RecordLaserAndPos(firstScan,m_PosNow);
    }


    CMatchInfo* results = Localize(estimatePose);

    Eigen::Affine3d estimatePoseTemp;
    estimatePoseTemp = estimatePose;
    CPosture PoseTemp = AffineToPosture(estimatePoseTemp);

    long long int loc1End = GetTickCount();

    // By Sam add
    legPose = estimatePose/*PostureToAffine(0, 0, 0)*/;
    legOK = LegLocalize(legPose);  // By Sam: For legMethod
    std::cout << "By Sam: legOK = " << legOK << std::endl;

    long long int loc2End = GetTickCount();
    std::cout << "Local1_Time =   " << static_cast<int>(loc1End - loc1Start) <<
                 "Leg_Time =   " << static_cast<int>(loc2End - loc1End) <<std::endl;
    #ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "LocalTime = ", static_cast<int>(loc1End - loc1Start));
    #endif

	// dq VISION estmPos_cam
    CPosture estmPos;
    CPosture estmPos_cam;

    // 如果需要补偿定位运算时间，需要在此对结果姿态进行插补 再对上面的定位结果进行插补
    long long int tmSecond = GetTickCount();
    CPosture odomTransInsert;  
    auto pOdometryInsert = BaseOdomSingleton::GetInstance();
    pOdometryInsert->GetLocalOdomTrans(tmStart, tmSecond, odomTransInsert);      
    // std::cout<<"tmSecond - cam_time = "<<tmSecond-CamPose_Data.lRawTime<<", cam_time: "<<CamPose_Data.lRawTime<<", tmSecond: "<<tmSecond<<std::endl;
    Eigen::Affine3d TFromScan = PostureToAffine(odomTransInsert);  
    estimatePose = estimatePose * TFromScan;
    estmPos = AffineToPosture(estimatePose);

     if(m_bFlagCameraLoc&& (!m_bRecordImg))
     {
         CPosture odomTransInsert_cam;
         pOdometryInsert->GetLocalOdomTrans(CamPose_Data.lRawTime, tmSecond, odomTransInsert_cam);
         Eigen::Affine3d TFromScan_cam = PostureToAffine(odomTransInsert_cam);
         estimatePose_cam = estimatePose_cam * TFromScan_cam;
         estmPos_cam = AffineToPosture(estimatePose_cam);

     }

	    // By Sam For Leg, LegPose no need add velocity pose
    CPosture legCPosture;;
//    legPose = legPose * TFromScan;
//    if (legOK)
//        legPose = legPose * TFromScan;
//    else
//        legPose = PostureToAffine(0, 0, 0);

    if (legOK!=1)
        legPose = PostureToAffine(0, 0, 0);


    legCPosture = AffineToPosture(legPose);


    // 补偿量
    #ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "AGV xCompensate:", odomTransInsert.x, ", yCompensate:", odomTransInsert.y, ", radCompensate:", odomTransInsert.fThita, ", tmStart:", (int)tmStart, ", tmSecond:", (int)tmSecond);
    #endif

    //Add By Yu.--2022.4.9
    // agv停车判断
    double xVarAverage, yVarAverage, fThitaVarAverage;
    double xVarDelta, yVarDelta, fThitaVarDelta;

    if(m_bInitLocSuccess)
    {
        CPosture odomTrans = odomTransInsert;
        double odomX = fabs(odomTrans.x - 0);
        double odomY = fabs(odomTrans.y - 0);
        double odomThita = fabs(odomTrans.fThita - 0);
       if(odomX > 0.000001 || odomY > 0.000001 || odomThita > 0.000001)
        {
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "AGV move!!");
#endif
            m_addPosTimes = 0;
            m_xVar = 0;
            m_yVar = 0;
            m_fThitaVar = 0;

        }
        else
        {          
            if(m_addPosTimes < 5)
            {
#ifdef USE_BLACK_BOX
       FILE_BlackBox(LocBox, "AGV stop, m_addPosTimes: ", m_addPosTimes);
#endif
                m_xVar += estmPos.x;
                m_yVar += estmPos.y;
                m_addPosTimes++;
            }
            else
            {
                xVarAverage = m_xVar / 5;
                yVarAverage = m_yVar / 5;

                xVarDelta = fabs(estmPos.x - xVarAverage);
                yVarDelta = fabs(estmPos.y - yVarAverage);

                if(xVarDelta < 0.05 && yVarDelta < 0.05)
                {
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "AGV stop, pos not change!!");
#endif
                }
                else
                {                    
                    estmPos.x = xVarAverage;
                    estmPos.y = yVarAverage;
//                    estmPos.fThita = fThitaVarAverage;
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "AGV stop, pos change, out averagePos:", estmPos.x, ",", estmPos.y, ",", estmPos.fThita);
#endif
                }
            }
        }
    }

    CStampedPos legPosNow(legCPosture, tmSecond);

    //位姿滤波
    CStampedPos pose_filtered,pose_filtered_cam;
    CStampedPos stampedPosNow(estmPos, tmSecond);
    CStampedPos stampedPosNow_cam(estmPos_cam, tmSecond);

    FilterPose(stampedPosNow);    
    GetFilteredPose(filter_enable, pose_filtered);

    if(m_bFlagCameraLoc&& (!m_bRecordImg))
    {
        FilterPose_Cam(stampedPosNow_cam);
        GetFilteredPose_Cam(filter_enable, pose_filtered_cam);
    }

    if(filter_enable)
    {
        m_PosNow = pose_filtered;
    }
    else
    {
        m_PosNow = stampedPosNow;
    }


    if(m_bFlagCameraLoc&& (!m_bRecordImg))
    {
        std::cout<<"************************ estmPos: "<<estmPos.x<<", "<<estmPos.y<<", "<<estmPos.fThita<<std::endl;
        std::cout<<"************************ estmPos_cam: "<<estmPos_cam.x<<", "<<estmPos_cam.y<<", "<<estmPos_cam.fThita<<std::endl;
        //std::cout<<"@@@@@@@@@@@@@@@@@@@@@@@@filtered: "<<pose_filtered.x<<", "<<pose_filtered.y<<", "<<pose_filtered.fThita<<std::endl;
        //std::cout<<"@@@@@@@@@@@@@@@@@@@@@@@@filtered_cam: "<<pose_filtered_cam.x<<", "<<pose_filtered_cam.y<<", "<<pose_filtered_cam.fThita<<std::endl;

        double diff_dis_x = estmPos.x-estmPos_cam.x;
        double diff_dis_y = estmPos.y-estmPos_cam.y;
        double diff_theta = 180*(estmPos.fThita-estmPos_cam.fThita)/PI;
        double Diff = sqrt(pow(diff_dis_x,2)+pow(diff_dis_y,2));
        std::cout<<"***************************************Diff_dis: "<<diff_dis_x<<", "<<diff_dis_y<<", "<<diff_theta<<std::endl;
        if (Diff > 0.15 || diff_theta > 10)
            std::cout<< "\033[31;1m###################################### DIFF Between Cam&Laser is too much!!!!!!!!!!!! ###################################### "<<Diff<<"\033[0m"<<std::endl;
    #ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "****** Filterpose:", pose_filtered.x, ",", pose_filtered.y, ",", pose_filtered.fThita);
    #endif

    #ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "****** Filterpose_cam:", pose_filtered_cam.x, ",", pose_filtered_cam.y, ",", pose_filtered_cam.fThita, "," ,CamPose_Data.uErrCode);
    #endif
        //发送定位结果并进行特别处理
        // DealWithLocResult(m_PosNow,results);
        if(m_reloc)
             CamPose_Data.uErrCode = 0;

        DealWithLocResult_Cam(m_PosNow,legPosNow,results,pose_filtered_cam,CamPose_Data.uErrCode);
    }
    else
    {
        //发送定位结果并进行特别处理
        DealWithLocResult(m_PosNow, legPosNow, results);   // By Sam Add For LegMethod
    }
		 


    std::cout << "By Sam Test: FinalPose_2 (" << m_PosNow.x <<
                 ", " << m_PosNow.y << ", " << (m_PosNow.fThita / 3.14) * 180 << ")" << std::endl;

    // 更新数据集全局位姿
    CPosture GlobalEstPst(m_PosNow.x,m_PosNow.y,m_PosNow.fThita);

    odom_data.global_pst = GlobalEstPst;
    if(bActiveCloud){
        pRawScan.SetOdometry(odom_data);
        pRawMap->AddRawScan(pRawScan);
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "odom_data.local_pst:", pRawScan.odom_data.local_pst.x,",",pRawScan.odom_data.local_pst.y,",",pRawScan.odom_data.local_pst.fThita);
#endif
    }

    if(m_Diagnosis.publish_point_cloud){
     // printf("LocalizeFactory, m_PosNow.m_dwTimeStamp = %lld \n", m_PosNow.m_dwTimeStamp);
//      pRawScan.odom_data.time_stamp = m_PosNow.m_dwTimeStamp;  /////////////////
//      LCMTask::GetLcmInstance().GetLocalizationMsg(pRawScan);
      DDSTask::GetDDSInstance().GetLocalizationMsg(pRawScan,m_PosNow);//改为DDS通讯 2023-09-26

    }


    if(bSendPointCloud)
    {
        LCMTask::GetLcmInstance().SendGlobalPoindCloud(m_PosNow,cloudAdjusted);
        bSendPointCloud = false;
    }
    else
    {
        bSendPointCloud = true;     
    }

    if(m_Diagnosis.publish_reflector)
    {
        if(results && results->type_ == CMatchInfo::LOC_FEATURE)
        {
            LCMTask::GetLcmInstance().SendFeatureMatchInfo(results);
        }
    }
    if(pRawCloud) {
        pRawCloud.reset();
        pRawCloud = nullptr;
    }
    m_LocPreTime = m_LocCurTime.load();
    m_LocEndTime = GetTickCount();
    pRawScan.Clear();
    firstScan.Clear();

//    if((m_LocEndTime - m_LocPreTime)<100)
//    {
//        int tt = (100-(m_LocEndTime - m_LocPreTime))*1000;
//        usleep(tt);

//    }
//    else {
//        usleep(10*1000);
//    }
    if(m_reloc)
    {
        m_reloc = false;
        usleep(1000*1000);
    }


        std::cout << "Circle_Time = " << static_cast<int>(m_LocEndTime - m_LocPreTime) << std::endl;
   //  #ifdef USE_BLACK_BOX
   //     FILE_BlackBox(LocBox, "LocalTime = ", static_cast<int>(loc1End - loc1Start));
   //  #endif


}

//
// 对输入的原始点云转换成笛卡尔坐标系.
//
bool CLocalizeFactory::TransformCloud(const std::shared_ptr<sensor::CRawPointCloud>& pRawCloud,
                                        ndt_oru::CStampedPointCloud& cloud_trans)
{
    cloud_trans.Clear();
    if(!pRawCloud){
        return false;
    }
    //修改为激光同步时间戳
//    cloud_trans.m_dwTimeStamp = pRawCloud->timestamp_raw;
    cloud_trans.m_dwTimeStamp = pRawCloud->timestamp_sync;
    float first_angle = pRawCloud->start_angle;
    unsigned int num_pts = pRawCloud->num_points;
    short laser_id = pRawCloud->laser_id;
    float angular_increment = 0.0;
    if(num_pts > 0) {
        angular_increment = (pRawCloud->end_angle - pRawCloud->start_angle) / num_pts;
    }
    if(num_pts > pRawCloud->distance.size()){
        return false;
    }
    if(laser_id >= m_pScannerGroupParam->size()){
        return false;
    }

    double min_range = m_pScannerGroupParam->at(laser_id).m_fMinRange;
    double max_range = m_pScannerGroupParam->at(laser_id).m_fMaxRange;
    int iCriteritonThreshold = 254;    // 反光板识别强度门限  默认值254

    Eigen::Vector3d pt;
    Eigen::Vector3d ptraw;

    float r = 0.0;
    float a = 0.0;
    unsigned short int intensity = 0.0;

    cloud_trans.reserve(num_pts);
    // 引入Z方向的噪声
    double varz = 0.05 / (double)INT_MAX;
    for (int i = 0 ;i < num_pts; i++){

        r = (float)pRawCloud->distance[i] / 1000.0;  //原始数据距离以mm为单位，需要转换为m
        a = first_angle + i*angular_increment;

        intensity = pRawCloud->intensity[i];
        //统一处理了强度，大于各个反光板强度门限后，处理成同样的强度值2550(和数据集一样)，可以将不同的激光头混用
        if(intensity > iCriteritonThreshold)
        {
            intensity = 255;
        }
        // 过滤掉不满足距离的点
        if(r < min_range || r > max_range){
            r = 0.0;
        }

        // 过虑掉不满足可视角度的点
        if(!m_pScannerGroupParam->at(laser_id).m_AppAngleRange.Contain(a)){
            r = 0.0;
        }

        // 极径大于0，表示数据有效
        if(r >= 0.0){
            pt(0) = r * cos(a);
            pt(1) = r * sin(a);
//            pt(2) = varz*rand();
            pt(2) = 0;   // By Sam
            cloud_trans.push_back(pt);
            cloud_trans.vecIntensity.push_back(intensity);
        }
    }
    return true;
}

void CLocalizeFactory::OnReceiveLaserRawScan(unsigned int nScannerId, const std::shared_ptr<sensor::CRawPointCloud> &pCloud)
{
    std::lock_guard<std::mutex> lock(cloud_mtx);
    ndt_oru::CStampedPointCloud cloud;
    TransformCloud(pCloud, cloud);
    OnReceiveLaserScan(nScannerId,cloud);
}

bool CLocalizeFactory::StartLocalize()
{

    if (m_bLocStarted) {
        return true;
    }


    // Init signal events
    m_hLocKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hLocKillThread == NULL)
        return false;

    m_hLocThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hLocThreadDead == NULL)
        return false;

    // Start the support procedure
    pthread_attr_t attr;
    SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attr);
    if(pthread_create(&m_pLocThread, &attr, LocalizeSupportProc, reinterpret_cast<LPVOID>(this)) != 0){
        std::cout<<"Creat LocDataControl_ Pthread Failed"<<std::endl;
        return false;
    }
    else{
        std::cout<<"Creat LocDataControl_ Pthread OK"<<std::endl;
    }
    pthread_attr_destroy(&attr);
    std::cout<<"StartLocalize "<<std::endl;

    CSlamMethod *pSlamMethod  = (CSlamMethod *)(methods_->at(4));
    pSlamMethod->InitLocate();

    m_bLocStarted = true;
    m_bLocFirstTime = true;
    m_lastMethod = -1;
    m_bFirstFeatureFail = true;


    return true;
}
bool CLocalizeFactory::DealWithLocResult(const CStampedPos& pose, const CStampedPos& leg_pose, CMatchInfo* results)
{

    bool confident   = false;
    int  match_count = 0;
    float match_ratio = 0.0;
    if(results)
    {
        if(results->result_ > 0)
            confident = true;
         match_count = results->matchNum_;
         match_ratio = results->matchRatio_;
    }
    // 本周期内的位移变化量
    double cycle_dist = 0.0;
    long long cycle_time = static_cast<long long>(m_LocCurTime.load() - m_LocPreTime.load());
    if(cycle_time <= 0) {
        cycle_time = 0;
    }

    auto pOdometry = BaseOdomSingleton::GetInstance();
    mapping::StVelocity base_vel = pOdometry->GetBaseVelocity();
    CVelocity cur_vel;
    cur_vel.fXLinear = static_cast<double>(base_vel.Vx) / 1000.0;
    cur_vel.fYLinear = static_cast<double>(base_vel.Vy) / 1000.0;
    cur_vel.fAngular = static_cast<double>(base_vel.Vtheta) / 1000.0;
    cur_vel.fLinear = sqrt(cur_vel.fXLinear * cur_vel.fXLinear + cur_vel.fYLinear * cur_vel.fYLinear);

    // 当有线速度时,计算距离;当只有角速度时,计算弧度;
    if(fabs(cur_vel.fLinear) > 0.0001){
        cycle_dist = fabs(cur_vel.fLinear) * cycle_time / 1000.0;
    }
    else if (fabs(cur_vel.fAngular) > 0.0001) {
        cycle_dist = fabs(cur_vel.fAngular) * cycle_time / 1000.0;
    }
    else {
    }

    int x_ = static_cast<int>(pose.x * 1000.0);
    int y_ = static_cast<int>(pose.y * 1000.0);
    int theta_ = static_cast<int>(pose.fThita * 1000.0);

    // By Sam for legMethod
    int leg_x = static_cast<int>(leg_pose.x * 1000.0);
    int leg_y = static_cast<int>(leg_pose.y * 1000.0);
    int leg_theta = static_cast<int>(leg_pose.fThita * 1000.0);

    short uG = match_ratio;
    if(match_ratio >100)
        uG = 100;

    unsigned short uN = static_cast<unsigned short>(match_count);
    unsigned long long pos_time = pose.m_dwTimeStamp;  // 应该是当前定位结果的时间
    short error_code = RLP_NO_ERROR;
    unsigned char pos_mode = RLP_CONTINUOUS_POSITIONING;

    auto pRoboClnt = RoboClntSingleton::GetInstance();
    auto pAFamily = SensorFamilySingleton::GetInstance();
    // 当定位失败或者定位结果质量很差,超过一定距离且机器人的速度矢量为0,自动保存日志文件和数据集
    static double loc_fail_dist = 0.0;
    static double loc_good_dist = 0.0;
    static double loc_error_dist = 0.0;
    static bool match_fail = false;
    static bool allow_save = true;
    unsigned int min_match_num = m_Evaluate.min_match_num;


    if(!m_bInitLocSuccess)
        error_code = RLP_POSITIONING_FAILED;

    if(results)
    {
        switch(results->type_)
        {
            case CMatchInfo::LOC_NDT:
               min_match_num = m_Evaluate.min_match_num;
            break;
            case CMatchInfo::LOC_FEATURE:
                min_match_num = m_Evaluate.min_feature_match_num;
                uN+=50;
            break;
            case CMatchInfo::LOC_TEMPLATE:
                break;
            case CMatchInfo::LOC_GRID:
                min_match_num = 0;
                m_Evaluate.min_quality_level = 0;
            break;

              default:
                break;
        }
    }


    if(!confident || uG < m_Evaluate.min_quality_level || uN < min_match_num){
        match_fail = true;
        loc_fail_dist += cycle_dist;
        loc_good_dist = 0.0;
        pos_mode = RLP_VIRTUAL_POSITIONING;
    }
    else {
        loc_good_dist += cycle_dist;
        loc_fail_dist = 0.0;
        match_fail = false;
        pos_mode = RLP_CONTINUOUS_POSITIONING;

        if(m_bInitLocSuccess)
             error_code = RLP_NO_ERROR;

        if(m_nSetPosCounter>5)
        {
            m_bInitLocSuccess = true;
            m_addPosTimes = 0;
            m_nSetPosCounter = 0;
            m_xVar = 0;
            m_yVar = 0;
            m_fThitaVar = 0;
        }

        if(!m_bInitLocSuccess)
            m_nSetPosCounter++;

        // 正常行走一段距离后,允许再次保存黑匣子
        if(loc_good_dist >= m_Evaluate.good_keep_dist) {
            allow_save = true;
        }
    }
    if(!confident){
        loc_error_dist += cycle_dist;
    }
    else{
        loc_error_dist = 0.0;
    }

//    std::cout<<"By Yu Test : loc_fail_dist = "<<loc_fail_dist<<" loc_error_dist = "<<loc_error_dist<<std::endl;
    //error_tolerate_dist = 0.5,fail_tolerate_dist = 0.3

    if(match_fail && (loc_error_dist >= m_Evaluate.error_tolerate_dist || loc_fail_dist >= m_Evaluate.fail_tolerate_dist)) {
        error_code = RLP_POSITIONING_FAILED;
    }

    // 机器人路径跟踪失败后，自动产生日志
    bool path_follow_lost = false;
    CLocDataSet rx_data_set;
    if(pRoboClnt){
        pRoboClnt->GetRxLocDataSet(rx_data_set);
    }
    if(rx_data_set.m_VelocityData.Mask != RLP_PATH_FOLLOW_OK){
        path_follow_lost = true;
    }
    // 是否自动保存日志
    if(m_Diagnosis.auto_save_log &&
       allow_save &&
       ((match_fail && loc_fail_dist >= m_Evaluate.fail_tolerate_dist) || path_follow_lost))
    {
        if(fabs(cur_vel.fLinear) < 0.001 && fabs(cur_vel.fAngular) < 0.001){
            auto pBlackBox = AutoBlackBoxSingleton::GetInstance();
            pBlackBox->SetNeedOutPutFlag();
            allow_save = false;
        }
    }
    // 导航控制器和激光传感器通信中断
    if(pAFamily->IsBlocked()) {
        error_code = RLP_LASER_TIMEOUT;
    }


    if(results)
    {
        if(results->result_ == CMatchInfo::MATCH_LOADMAP_FAILED )
             error_code = RLP_LOADING_MAP_FAILED;
        if(results->result_ == CMatchInfo::MATCH_LOADING_MAP )
             error_code = RLP_LOADING_MAP;
    }
    if(!m_bLoadMapSuccess)
    {
         error_code = RLP_LOADING_MAP_FAILED;
    }
   /* if(results)
    {
        if(results->type_ == CMatchInfo::LOC_SLAM && results->result_== CMatchInfo::MATCH_FAIL)
            error_code =RLP_GRIDSLAM_FAILED;
    }*/
    // 发送位姿


    // By Sam: change for legMethod
    int legErr = 0;
    int legD_x = 0;
    int legD_y = 0;
    int legD_theta = 0;

    if (legOK==1)
    {
        legErr = 1;

        Eigen::Affine3d agvAffine = PostureToAffine(pose);
        Eigen::Affine3d legAffine = PostureToAffine(leg_pose);
        Eigen::Affine3d leg_agv_diff = agvAffine.inverse() * legAffine;
        CPosture legDiffagv = AffineToPosture(leg_agv_diff);

        legD_x = static_cast<int>(legDiffagv.x * 1000.0);
        legD_y = static_cast<int>(legDiffagv.y * 1000.0);
        legD_theta = static_cast<int>(legDiffagv.fThita * 1000.0);
    }
    if(legOK==2)
        legErr = 1;

   std::string strLocFucntion("Loc Fail");
   int iType = -1;
    if(results) //如果results为null
    {
        iType =  results->type_;
        switch(results->type_)
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
            case CMatchInfo::LOC_GRID:
                strLocFucntion = "Grid";
                break;

        }
        if(match_fail)
        {
            strLocFucntion+=" Loc Fail";
        }
        else
        {
            strLocFucntion+=" Loc OK";
        }

    }
     std::cout << "Types:"<< strLocFucntion <<" RoboPose: x_=" << x_ << ",y_=" << y_ << ",theta_=" << (theta_ / 3.14) * 180 << ",quality=" << uG << ",Number="
               << uN << ",confident=" << confident << ",pos_time=" << pos_time << ",error_code=" << error_code<< std::endl;

#ifdef USE_LEG_METHOD

    if(pRoboClnt){
        if(match_fail){
           pRoboClnt->SetRoboPose(x_, y_, theta_, leg_x, leg_y, leg_theta, legD_x, legD_y, legD_theta, 0, 0, pos_time, error_code, legErr, pos_mode);
        }
        else {
            pRoboClnt->SetRoboPose(x_, y_, theta_, leg_x, leg_y, leg_theta, legD_x, legD_y, legD_theta, uG, uN, pos_time, error_code, legErr, pos_mode);
        }
    }
    #ifdef USE_BLACK_BOX

        FILE_BlackBox(LocBox,"Types: ",iType, " RoboPose:",pose.x,", ",pose.y,", ",(pose.fThita/3.14) * 180,
                      ", GoodsCPose: (",leg_pose.x,", ", leg_pose.y,", ", (leg_pose.fThita / 3.14) * 180, "), diff_agv_leg: (",
                      (float)legD_x / 1000,", ", (float)legD_y / 1000,", ", ((float)legD_theta / 3140) * 180, "°), uG="
                      ,uG,", uN=",uN,", ",(int)confident,", ",(int)pos_time,", ",error_code,", ",legErr,", ",(int)pos_mode);
    #endif
#else
    if(pRoboClnt){
        if(match_fail){
           pRoboClnt->SetRoboPose(x_, y_, theta_, 0, 0, pos_time, error_code, pos_mode);
        }
        else {
            pRoboClnt->SetRoboPose(x_, y_, theta_, uG, uN, pos_time, error_code, pos_mode);
        }
    }
    #ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox,"Types: ",iType, " RoboPose:",pose.x,", ",pose.y,", ",(pose.fThita/3.14) * 180,", uG=",uG,", uN=",uN,", ",(int)confident,", ",(int)pos_time,", ",error_code,", ",(int)pos_mode);
    #endif
#endif



    return true;
}


//int fake_count = 0;
//int fake_i = -1;
bool CLocalizeFactory::DealWithLocResult_Cam(CStampedPos& pose, const CStampedPos& leg_pose,CMatchInfo* results, const CStampedPos& pose_cam, const short errorcode_cam)
{
    CLocDataSet Setpos_cam;
    Setpos_cam.Initialize();
    /*if(fake_count <= 0)
    {
        fake_count = rand()%200+1;
        fake_i = -fake_i;
    }
    else
        fake_count--;*/
    bool confident   = false;
    int  match_count = 0;
    float match_ratio = 0.0;
    if(results)
    {
        if(results->result_ > 0 )
            confident = true;
         match_count = results->matchNum_;
         match_ratio = results->matchRatio_;
    }

    // 本周期内的位移变化量
    double cycle_dist = 0.0;
    long long cycle_time = static_cast<long long>(m_LocCurTime.load() - m_LocPreTime.load());
    if(cycle_time <= 0) {
        cycle_time = 0;
    }

    auto pOdometry = BaseOdomSingleton::GetInstance();
    mapping::StVelocity base_vel = pOdometry->GetBaseVelocity();
    CVelocity cur_vel;
    cur_vel.fXLinear = static_cast<double>(base_vel.Vx) / 1000.0;
    cur_vel.fYLinear = static_cast<double>(base_vel.Vy) / 1000.0;
    cur_vel.fAngular = static_cast<double>(base_vel.Vtheta) / 1000.0;
    cur_vel.fLinear = sqrt(cur_vel.fXLinear * cur_vel.fXLinear + cur_vel.fYLinear * cur_vel.fYLinear);




    // 当有线速度时,计算距离;当只有角速度时,计算弧度;
    if(fabs(cur_vel.fLinear) > 0.0001){
        cycle_dist = fabs(cur_vel.fLinear) * cycle_time / 1000.0;
    }
    else if (fabs(cur_vel.fAngular) > 0.0001) {
        cycle_dist = fabs(cur_vel.fAngular) * cycle_time / 1000.0;
    }
    else {
    }


    // By Sam for legMethod
    int leg_x = static_cast<int>(leg_pose.x * 1000.0);
    int leg_y = static_cast<int>(leg_pose.y * 1000.0);
    int leg_theta = static_cast<int>(leg_pose.fThita * 1000.0);

    short uG = match_ratio;
    if(match_ratio >100)
        uG = 100;

    unsigned short uN = static_cast<unsigned short>(match_count);
    unsigned long long pos_time = pose.m_dwTimeStamp;  // 应该是当前定位结果的时间
    short error_code = RLP_NO_ERROR;
    unsigned char pos_mode = RLP_CONTINUOUS_POSITIONING;

    auto pRoboClnt = RoboClntSingleton::GetInstance();
    auto pAFamily = SensorFamilySingleton::GetInstance();
    // 当定位失败或者定位结果质量很差,超过一定距离且机器人的速度矢量为0,自动保存日志文件和数据集
    static double loc_fail_dist = 0.0;
    static double loc_good_dist = 0.0;
    static double loc_error_dist = 0.0;
    static bool match_fail = false;
    static bool allow_save = true;
    unsigned int min_match_num = m_Evaluate.min_match_num;


    // if(!m_bInitLocSuccess || errorcode_cam < 1)         // dq VISION 激光初始定位失败或视觉初始定位失败
    if(!m_bInitLocSuccess)
        error_code = RLP_POSITIONING_FAILED;

    if(results)
    {
        switch(results->type_)
        {
            case CMatchInfo::LOC_NDT:
               min_match_num = m_Evaluate.min_match_num;
            break;
            case CMatchInfo::LOC_FEATURE:
                min_match_num = m_Evaluate.min_feature_match_num;
                uN+=50;
            break;
            case CMatchInfo::LOC_TEMPLATE:
                break;
            case CMatchInfo::LOC_GRID:
                min_match_num = 0;
                m_Evaluate.min_quality_level = 0;
            break;

              default:
                break;
        }
    }

    if((!confident || uG < m_Evaluate.min_quality_level || uN < min_match_num) && errorcode_cam < 1){
        std::cout<<"match_fail = true"<<std::endl;
        match_fail = true;
        loc_fail_dist += cycle_dist;
        loc_good_dist = 0.0;
        pos_mode = RLP_VIRTUAL_POSITIONING;
    }
    else {
        loc_good_dist += cycle_dist;
        loc_fail_dist = 0.0;
        match_fail = false;
        pos_mode = RLP_CONTINUOUS_POSITIONING;

        if(m_bInitLocSuccess && errorcode_cam > 0)
             error_code = RLP_NO_ERROR;

        if(m_nSetPosCounter>5)
        {
            m_bInitLocSuccess = true;
            m_addPosTimes = 0;
            m_nSetPosCounter = 0;
            m_xVar = 0;
            m_yVar = 0;
            m_fThitaVar = 0;
        }

        if(!m_bInitLocSuccess || errorcode_cam < 1)
            m_nSetPosCounter++;

        // 正常行走一段距离后,允许再次保存黑匣子
        if(loc_good_dist >= m_Evaluate.good_keep_dist) {
            allow_save = true;
        }
    }
    if(!confident){
        loc_error_dist += cycle_dist;
    }
    else{
        loc_error_dist = 0.0;
    }

//    std::cout<<"By Yu Test : loc_fail_dist = "<<loc_fail_dist<<" loc_error_dist = "<<loc_error_dist<<std::endl;
    //error_tolerate_dist = 0.5,fail_tolerate_dist = 0.3

    if(match_fail && (loc_error_dist >= m_Evaluate.error_tolerate_dist || loc_fail_dist >= m_Evaluate.fail_tolerate_dist)) {
        error_code = RLP_POSITIONING_FAILED;
    }

    // 机器人路径跟踪失败后，自动产生日志
    bool path_follow_lost = false;
    CLocDataSet rx_data_set;
    if(pRoboClnt){
        pRoboClnt->GetRxLocDataSet(rx_data_set);
    }
    if(rx_data_set.m_VelocityData.Mask != RLP_PATH_FOLLOW_OK){
        path_follow_lost = true;
    }
    // 是否自动保存日志
    if(m_Diagnosis.auto_save_log &&
       allow_save &&
       ((match_fail && loc_fail_dist >= m_Evaluate.fail_tolerate_dist) || path_follow_lost))
    {
        if(fabs(cur_vel.fLinear) < 0.001 && fabs(cur_vel.fAngular) < 0.001){
            auto pBlackBox = AutoBlackBoxSingleton::GetInstance();
            pBlackBox->SetNeedOutPutFlag();
            allow_save = false;
        }
    }
    // 导航控制器和激光传感器通信中断
    if(pAFamily->IsBlocked()) {
        error_code = RLP_LASER_TIMEOUT;
    }





    if(results)
    {
        if(results->result_ == CMatchInfo::MATCH_LOADMAP_FAILED )
             error_code = RLP_LOADING_MAP_FAILED;
        if(results->result_ == CMatchInfo::MATCH_LOADING_MAP )
             error_code = RLP_LOADING_MAP;
    }
    if(!m_bLoadMapSuccess)
    {
         error_code = RLP_LOADING_MAP_FAILED;
    }
   /* if(results)
    {
        if(results->type_ == CMatchInfo::LOC_SLAM && results->result_== CMatchInfo::MATCH_FAIL)
            error_code =RLP_GRIDSLAM_FAILED;
    }*/

    // 融合位姿
    //if(fake_i == -1)
    //    confident = false;
    //std::cout<<"fake count: "<<fake_count<<", fake_i: "<<fake_i<<", confident: "<<confident<<std::endl;
    double r = 0;
    std::cout<<"\033[33;1muG: "<<uG<<", uN: "<<uN<<", errorcode_cam: "<<errorcode_cam<<"\033[0m"<<std::endl;
    // std::cout<<"confident: "<<confident<<std::endl;

    if((!confident || uG < m_Evaluate.min_quality_level || uN < min_match_num) && errorcode_cam > 0)
    {
        if(m_CamLocCount < m_MaxCamLocCount)
        {
            m_CamLocCount++;
            r = m_CamLocCount/m_MaxCamLocCount;
            //r = 0.5*(1+tanh(1/(1-m_CamLocCount/m_MaxCamLocCount))-m_MaxCamLocCount/m_CamLocCount);
        }
        else
            r = 1;
    }
    else if(m_CamLocCount > 0)
    {
            m_CamLocCount--;
            r = m_CamLocCount/m_MaxCamLocCount;
            //r = 0.5*(1+tanh(1/(1-m_CamLocCount/m_MaxCamLocCount))-m_MaxCamLocCount/m_CamLocCount);
    }


    if(m_topVisionMode == ONLY_TOPVISION_LOCALIZATION)
    {
        r = 1;
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox,"ONLY_TOPVISION_LOCALIZATION! ");
#endif
    }
    if(m_topVisionMode == ONLY_LASER_LOCALIZATION)
    {
        r = 0;
    }

    if(r > 1)
        r = 1;
    if(r < 0)
        r = 0;

    if(m_reloc)
        r = 0;
    // r = 0.5*(1+tanh(1/(1-m_CamLocCount/m_MaxCamLocCount))-m_MaxCamLocCount/m_CamLocCount);
    //std::cout<<"filter R = "<< r <<", m_CamLocCount: "<<m_CamLocCount<<std::endl;
    pose.x = (1-r)*pose.x +r*pose_cam.x;
    pose.y = (1-r)*pose.y +r*pose_cam.y;
    if(r>0.5)
        pose.fThita = pose_cam.fThita;
    std::cout<<"$$$$$$$ Report pose.x: "<<pose.x<<", pose.y: "<<pose.y<<", pose.fThita: "<<pose.fThita<<std::endl;
    // 发送位姿

    int x_ = static_cast<int>(pose.x * 1000.0);
    int y_ = static_cast<int>(pose.y * 1000.0);
    int theta_ = static_cast<int>(pose.fThita * 1000.0);
    if(m_reloc)
    {
        Setpos_cam.m_PositionData.nX = x_;
        Setpos_cam.m_PositionData.nY = y_;
        Setpos_cam.m_PositionData.nTheta = theta_;
        Setpos_cam.m_PositionData.lRawTime = pos_time;
        pRoboClnt->SetInitPosition_Cam(Setpos_cam.m_PositionData,1);
    }
    if(pRoboClnt){


        // By Sam: change for legMethod
        int legErr = 0;
        int legD_x = 0;
        int legD_y = 0;
        int legD_theta = 0;

        if (legOK==1)
        {
            legErr = 1;

            Eigen::Affine3d agvAffine = PostureToAffine(pose);
            Eigen::Affine3d legAffine = PostureToAffine(leg_pose);
            Eigen::Affine3d leg_agv_diff = agvAffine.inverse() * legAffine;
            CPosture legDiffagv = AffineToPosture(leg_agv_diff);

            legD_x = static_cast<int>(legDiffagv.x * 1000.0);
            legD_y = static_cast<int>(legDiffagv.y * 1000.0);
            legD_theta = static_cast<int>(legDiffagv.fThita * 1000.0);
        }
        if(legOK==2)
            legErr = 1;

        if(match_fail){
           std::cout<<"match failed!"<<std::endl;

#ifdef USE_LEG_METHOD
           pRoboClnt->SetRoboPose(x_, y_, theta_, leg_x, leg_y, leg_theta, legD_x, legD_y, legD_theta, 0, 0, pos_time, error_code, legErr, pos_mode);
#else
           pRoboClnt->SetRoboPose(x_, y_, theta_, 0, 0, pos_time, error_code, pos_mode);
#endif
        }
        else if((!confident || uG < m_Evaluate.min_quality_level || uN < min_match_num) && errorcode_cam > 0){
            std::cout<<"laser failed but vision successed!"<<std::endl;
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox,"laser failed but vision successed! RoboPose:",x_,", ",y_,", ",(theta_/3.14) * 180,", uG=",uG,", uN=",uN,", ",(int)confident,", ",(int)pos_time,", ",(int)errorcode_cam);
#endif

#ifdef USE_LEG_METHOD
            pRoboClnt->SetRoboPose(x_, y_, theta_, leg_x, leg_y, leg_theta, legD_x, legD_y, legD_theta,100, 100, pos_time, error_code, legErr, pos_mode);
#else

            pRoboClnt->SetRoboPose(x_, y_, theta_, 100, 100, pos_time, error_code, pos_mode);
#endif
        }
        else{
            std::cout<<"match successed! "<<std::endl;
            if((uG > 30 && uN > 30) && (m_topVisionMode != ONLY_TOPVISION_LOCALIZATION))
            {
                Setpos_cam.m_PositionData.nX = x_;
                Setpos_cam.m_PositionData.nY = y_;
                Setpos_cam.m_PositionData.nTheta = theta_;
                Setpos_cam.m_PositionData.lRawTime = pos_time;
                //dq VISION 激光定位正确后，发送位姿给顶视端（根据需要打开）
               // pRoboClnt->SetInitPosition_Cam(Setpos_cam.m_PositionData,1);
            }
#ifdef USE_LEG_METHOD
            pRoboClnt->SetRoboPose(x_, y_, theta_, leg_x, leg_y, leg_theta, legD_x, legD_y, legD_theta,uG, uN, pos_time, error_code, legErr, pos_mode);
#else
            pRoboClnt->SetRoboPose(x_, y_, theta_, uG, uN, pos_time, error_code, pos_mode);
#endif
        }
    }

   std::string strLocFucntion("Loc Fail");
   int iType = -1;
    if(results) //如果results为null
    {
        iType =  results->type_;
        switch(results->type_)
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
            case CMatchInfo::LOC_GRID:
                strLocFucntion = "Grid";
                break;

        }
        if(match_fail)
        {
            strLocFucntion+=" Loc Fail";
        }
        else if((!confident || uG < m_Evaluate.min_quality_level || uN < min_match_num) && errorcode_cam > 0)
        {
            strLocFucntion = "VISION";
            strLocFucntion+= " Loc OK";
            confident = errorcode_cam;
            pose = pose_cam;
        }
        else
        {
            strLocFucntion+=" Loc OK";
        }

    }
   // std::cout << "Types:"<< strLocFucntion <<" RoboPose: x_=" << x_ << ",y_=" << y_ << ",theta_=" << (theta_ / 3.14) * 180 << ",quality=" << uG << ",Number="
   //           << uN << ",confident=" << confident << ",pos_time=" << pos_time << ",error_code=" << error_code<< std::endl;
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox,"Types: ",iType, " RoboPose:",pose.x,", ",pose.y,", ",(pose.fThita/3.14) * 180,", uG=",uG,", uN=",uN,", ",(int)confident,", ",(int)pos_time,", ",error_code,", ",(int)pos_mode);
#endif
    return true;
}

//
//   对计算出的结果姿态进行平滑滤波。
//
bool CLocalizeFactory::FilterPose(const CStampedPos& pose)
{
    if(!m_FilterParm.bEnable){
        m_PosNowFiltered = pose;
        return true;
    }

    if(m_nSlideDataCount >= m_nSlideMeanSize) {
        m_RecentPoses.pop_front();
        m_RecentPoses.push_back(pose);
        m_nSlideDataCount = m_nSlideMeanSize;
    }
    else {
        m_RecentPoses.push_back(pose);
        m_nSlideDataCount += 1;
    }

    int run_step = 0;
    auto pOdometry = BaseOdomSingleton::GetInstance();
    CPosture odom_trans, pose_trans;
    Eigen::Affine3d TPose, TFromOdom;
    bool trans_ok = false;
    unsigned long long cur_pose_time = pose.m_dwTimeStamp;
    unsigned long pose_size = m_RecentPoses.size();
    unsigned long factor_size = m_FilterParm.vFactor.size();
    CStampedPos    pose_filtered;
    std::deque<CStampedPos> sync_poses;
    sync_poses.clear();
    pose_filtered.SetPosture(0.0, 0.0, 0.0);

    if(m_nSlideDataCount >= m_nSlideMeanSize && pose_size == factor_size){
        for (unsigned long i = 0 ; i < pose_size; i++){
            if(m_RecentPoses[i].GetTimeStamp() == cur_pose_time){
                sync_poses.push_back(m_RecentPoses[i]);
                continue;
            }

            trans_ok = pOdometry->GetLocalOdomTrans(m_RecentPoses[i].GetTimeStamp(), cur_pose_time, odom_trans);
            if(!trans_ok){
                break;
            }
            TFromOdom = PostureToAffine(odom_trans);
            TPose = PostureToAffine(m_RecentPoses[i].x, m_RecentPoses[i].y, m_RecentPoses[i].fThita);
            TPose = TPose * TFromOdom;
            pose_trans = AffineToPosture(TPose);
            sync_poses.push_back(CStampedPos(pose_trans, cur_pose_time));
        }

        double norm_angle = 0.0;
        unsigned long sync_pose_size = sync_poses.size();
        if(trans_ok && sync_pose_size == factor_size){
            for (unsigned long i = 0 ; i < pose_size; i++){
                pose_filtered.x += sync_poses[i].x * m_FilterParm.vFactor[i];
                pose_filtered.y += sync_poses[i].y * m_FilterParm.vFactor[i];

                // 角度标准化
                if(i == 0){
                    pose_filtered.fThita += sync_poses[i].fThita * m_FilterParm.vFactor[i];
                }
                else {
                    norm_angle = CAngle::NormalizeAngleDifference(sync_poses[i].fThita, sync_poses[0].fThita);
                    pose_filtered.fThita += norm_angle * m_FilterParm.vFactor[i];
                }
            }
            // [0, 2*PI]
            pose_filtered.fThita = CAngle::NormAngle(pose_filtered.fThita);
            pose_filtered.m_dwTimeStamp = cur_pose_time;
            m_PosNowFiltered = pose_filtered;
            run_step = 1;
        }
        else{
            m_PosNowFiltered = pose;
            run_step = 2;
        }
    }
    else {
        m_PosNowFiltered = pose;
        run_step = 3;
    }

#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "FilterPose: ",m_PosNowFiltered.x,", ",m_PosNowFiltered.y,", ",(m_PosNowFiltered.fThita/3.14) * 180,
                  ", ", static_cast<int>(m_PosNowFiltered.m_dwTimeStamp), ", ", run_step);
#endif

    return true;
}


bool CLocalizeFactory::FilterPose_Cam(const CStampedPos& pose)
{
    if(!m_FilterParm.bEnable){
        m_PosNowFiltered_Cam = pose;
        return true;
    }

    if(m_nSlideDataCount_Cam >= m_nSlideMeanSize) {
        m_RecentPoses_Cam.pop_front();
        m_RecentPoses_Cam.push_back(pose);
        m_nSlideDataCount_Cam = m_nSlideMeanSize;
    }
    else {
        m_RecentPoses_Cam.push_back(pose);
        m_nSlideDataCount_Cam += 1;
    }

    int run_step = 0;
    auto pOdometry = BaseOdomSingleton::GetInstance();
    CPosture odom_trans, pose_trans;
    Eigen::Affine3d TPose, TFromOdom;
    bool trans_ok = false;
    unsigned long long cur_pose_time = pose.m_dwTimeStamp;
    unsigned long pose_size = m_RecentPoses_Cam.size();
    unsigned long factor_size = m_FilterParm.vFactor.size();
    CStampedPos    pose_filtered;
    std::deque<CStampedPos> sync_poses;
    sync_poses.clear();
    pose_filtered.SetPosture(0.0, 0.0, 0.0);

    if(m_nSlideDataCount_Cam >= m_nSlideMeanSize && pose_size == factor_size){
        for (unsigned long i = 0 ; i < pose_size; i++){
            if(m_RecentPoses_Cam[i].GetTimeStamp() == cur_pose_time){
                sync_poses.push_back(m_RecentPoses_Cam[i]);
                continue;
            }

            trans_ok = pOdometry->GetLocalOdomTrans(m_RecentPoses_Cam[i].GetTimeStamp(), cur_pose_time, odom_trans);
            if(!trans_ok){
                break;
            }
            TFromOdom = PostureToAffine(odom_trans);
            TPose = PostureToAffine(m_RecentPoses_Cam[i].x, m_RecentPoses_Cam[i].y, m_RecentPoses_Cam[i].fThita);
            TPose = TPose * TFromOdom;
            pose_trans = AffineToPosture(TPose);
            sync_poses.push_back(CStampedPos(pose_trans, cur_pose_time));
        }

        double norm_angle = 0.0;
        unsigned long sync_pose_size = sync_poses.size();
        if(trans_ok && sync_pose_size == factor_size){
            for (unsigned long i = 0 ; i < pose_size; i++){
                pose_filtered.x += sync_poses[i].x * m_FilterParm.vFactor[i];
                pose_filtered.y += sync_poses[i].y * m_FilterParm.vFactor[i];

                // 角度标准化
                if(i == 0){
                    pose_filtered.fThita += sync_poses[i].fThita * m_FilterParm.vFactor[i];
                }
                else {
                    norm_angle = CAngle::NormalizeAngleDifference(sync_poses[i].fThita, sync_poses[0].fThita);
                    pose_filtered.fThita += norm_angle * m_FilterParm.vFactor[i];
                }
            }
            // [0, 2*PI]
            pose_filtered.fThita = CAngle::NormAngle(pose_filtered.fThita);
            pose_filtered.m_dwTimeStamp = cur_pose_time;
            m_PosNowFiltered_Cam = pose_filtered;
            run_step = 1;
        }
        else{
            m_PosNowFiltered_Cam = pose;
            run_step = 2;
        }
    }
    else {
        m_PosNowFiltered_Cam = pose;
        run_step = 3;
    }

#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "FilterPose_Cam: ",m_PosNowFiltered_Cam.x,", ",m_PosNowFiltered_Cam.y,", ",(m_PosNowFiltered_Cam.fThita/3.14) * 180,
                  ", ", static_cast<int>(m_PosNowFiltered_Cam.m_dwTimeStamp), ", ", run_step);
#endif

    return true;
}

bool CLocalizeFactory::InitializeFilterParms()
{
    m_FilterParm.bEnable = false;
    m_FilterParm.vFactor.clear();

    auto pParameterObject = ParameterObjectSingleton::GetInstance();
    pParameterObject->GetParameterValue("Filter_Enable", m_FilterParm.bEnable);
    pParameterObject->GetParameterValue("Filter_Factor", m_FilterParm.vFactor);

    // 初始化
    m_nSlideMeanSize = static_cast<unsigned int>(m_FilterParm.vFactor.size());
    m_nSlideDataCount = 0;
    m_nSlideDataCount_Cam = 0;
    m_RecentPoses.clear();
    m_RecentPoses_Cam.clear();

    // 核实滤波因子,所有因子相加必须等于１
    double sum_factor = 0.0;
    for (unsigned int i = 0 ;i < m_nSlideMeanSize; i++){
        sum_factor += m_FilterParm.vFactor[i];
    }
    if(sum_factor > 1.00001 || sum_factor < 0.99999){
        for (unsigned int i = 0 ;i < m_nSlideMeanSize; i++){
            m_FilterParm.vFactor[i] = 1.0 / static_cast<double>(m_nSlideMeanSize);
        }
    }
    return true;
}

bool CLocalizeFactory::GetFilteredPose(bool& filter_enable,CStampedPos& filtered_pose)
{
    filter_enable = m_FilterParm.bEnable;
    filtered_pose = m_PosNowFiltered;
    return true;
}

bool CLocalizeFactory::GetFilteredPose_Cam(bool& filter_enable,CStampedPos& filtered_pose)
{
    filter_enable = m_FilterParm.bEnable;
    filtered_pose = m_PosNowFiltered_Cam;
    return true;
}

// by DQ 在当前加载FeatureMap.map中获取模板信息及区域信息
void CLocalizeFactory::GetStaticObjects(int count, vector<vector<float> > points, vector<CPosture> &psts)
{
    CStaticObjects *static_objects = ((CTemplateMethod *)methods_->at(2))->map_;
    count = static_objects->size();
    int type = 0;
    psts.resize(count);
    points.resize(count);
    // 逐一读取模板
    for(int i = 0; i < count; i ++)
    {
        points[i].clear();
        CStaticObject *obj = static_objects->at(i);
        // 读取位姿
        psts.at(i) = obj->GetPosture();
        // 读取模板的组成（每个模板由多个基础形状构成）
        int Template_size = obj->GetTemplate().size();
        for(int j = 0; j < Template_size; j++)
        {
            CObjectElement* objectelement =static_cast<CObjectElement *> (obj->GetTemplate().at(j));
            type = objectelement->m_nType;
            std::cout<<"type: "<<type<<std::endl;
            // 线段--端点坐标（xs，ys，xe，ye）
            if (type == 1)
            {
                CLineElement* Line = dynamic_cast<CLineElement *> (objectelement);
                float Lxs = Line->m_line.m_ptStart.x;
                float Lys = Line->m_line.m_ptStart.y;
                float Lxe = Line->m_line.m_ptEnd.x;
                float Lye = Line->m_line.m_ptEnd.y;
                //std::cout<<"type: "<< type <<" Lxs: "<< Lxs <<" Lxe: "<< Lxe <<std::endl;
                points.at(i).push_back(Lxs);
                points.at(i).push_back(Lys);
                // 添加Last线段End点
                if(j == Template_size-1)
                {
                    points.at(i).push_back(Lxe);
                    points.at(i).push_back(Lye);
                }
            }
            // 圆--圆心坐标及半径（x,y.r）
            else if (type == 2)
            {
                CCircleElement* Circle = dynamic_cast<CCircleElement *> (objectelement);
                float cx = Circle->m_circle.m_ptCenter.x;
                float cy = Circle->m_circle.m_ptCenter.y;
                float cr = Circle->m_circle.m_fRadius;
                points.at(i).push_back(cx);
                points.at(i).push_back(cy);
                points.at(i).push_back(cr);
                //std::cout<<"type: "<< type <<" cx: "<< cx <<" cy: "<< cy <<" cr: "<< cr <<std::endl;
            }
            else
                std::cout<<"Unknown Object Type!!!!"<<std::endl;
        }
        // 最后添加形状（0直线,1圆）
        points.at(i).push_back(type-1);
    }
}
// by DQ 在当前加载FeatureMap.map中获取模板信息及区域信息
void CLocalizeFactory::GetPlans(int count, vector<vector<float> > plans, vector<int> type)
{
    CLocalizationPlan *all_plan_ = plan_;
    if(all_plan_ != NULL)
    {
        count = all_plan_->size();
        plans.resize(count);
        type.resize(count);
        // 逐一读取区域
        for(int i = 0; i < count; i ++)
        {
            plans[i].clear();
            CLocalizationRect *q =dynamic_cast<CLocalizationRect*>(all_plan_->at(i));
            if(q == NULL)
                continue;

            plans[i].push_back(all_plan_->at(i)->GetLeftTopPoint().x);
            plans[i].push_back(all_plan_->at(i)->GetLeftTopPoint().y);
            plans[i].push_back(all_plan_->at(i)->GetRightBottomPoint().x);
            plans[i].push_back(all_plan_->at(i)->GetRightBottomPoint().y);
            plans[i].push_back(0); //楼层(未启用)
            if(q->CheckMethodUsed(0)==0)
                type.push_back(0);
            else if(q->CheckMethodUsed(1)==0)
                type.push_back(1);
            else if(q->CheckMethodUsed(3)==0) // SLAM区域
                type.push_back(3);
            else
                std::cout<<"ERROR!!!UNKNOWN METHOD!!!"<<std::endl;
        }
    }
}
// by DQ 根据传入数据修改FeatureMap模板地图 （分为修改模板和修改模板区域）
bool CLocalizeFactory::ChangeObj(int count, vector<vector<float> > points, vector<CPosture> &psts)
{
    std::lock_guard<std::mutex> lock(loc_mtx);
    CStaticObjects *static_objects = ((CTemplateMethod *)methods_->at(2))->map_;
    CLocalizationPlan *templateplan_ = plan_;

    // 清空现有模板
    static_objects->Clear();
    // 清空现有模板区域
    for(int i=0; i < templateplan_->size(); i++)
    {
        CLocalizationRect *q =dynamic_cast<CLocalizationRect*>(templateplan_->at(i));
        if(q == NULL)
            continue;
        if(q->CheckMethodUsed(2)==0)
        {
           if(templateplan_->Delete(i))
           {
                std::cout<<"------------------------ Templateplan delete!!!!!!"<<std::endl;
                i--;
           }
        }
    }
    std::cout<<"---------------------static_objects size Before: "<<static_objects->size()<<"!!!!!!!!!!"<<std::endl;
    std::cout<<"---------------------templateplan_ size Before : "<<(int)templateplan_->size()<<"!!!!!!!!!!"<<std::endl;

    // 先从文件中读入模板库部分
    if(count!=0)
    {
        CStockedObjects *temp = new CStockedObjects;
        if (!temp->LoadPadYaml(count, points))
        {
            std::cout<<"load from pad failed!!!!!"<<std::endl;
            delete temp;
            return false;
        }
        static_objects->SetStockedObjects(temp);

        // 先读入模板数量
        static_objects->resize(count);
        // 依次读入并初始化各个静态物体
        for (int j = 0; j < count; j++)
        {
            // 为新物体分配空间
            CStaticObject *obj = static_objects->NewStaticObject();

            // 初始化静态物体的模板库指针
            obj->SetStockedObjects(temp);
            // 再读入该物体的参数，并对其进行初始化
            char buf[64] ;
            //buf[0]=(char)('0'+j);
            //buf[1] = '\0';
            // 设置物体名称
            if(j < 10)
            {
                buf[0]=(char)('0'+j);
                buf[1] = '\0';
            }
            else if(j >= 10 && j < 100)
            {
                buf[0]=(char)('0'+j/10);
                buf[1]=(char)('0'+j%10);
                buf[2] = '\0';
            }
            else
                return false;

            std::cout << "---------------------------By DQ: Load static object name = " << buf << std::endl;
            // 设置名称长度
            int nameLen;
            nameLen = sizeof(buf);
            std::cout << "---------------------------By DQ: Load static object nameLen = " << nameLen << std::endl;

            // 设置模板号和布署姿态
            int tempId;
            tempId = j;
            std::cout << "---------------------------By DQ: Load static object ID = " << tempId << std::endl;

            float f[3];
            f[0] = psts[j].x;
            f[1] = psts[j].y;
            f[2] = psts[j].fThita;
            std::cout << "---------------------------By DQ: Load static object pst.x = " <<  f[0] <<
                        ", m_pst.y = " <<  f[1] <<
                        ", m_pst.thita = " <<  f[2] << std::endl;

            obj->SetTemplate(temp->at(j));
            obj->SetPosture(psts[j]);
            obj->name = buf;
            obj->m_nTempId = tempId;
            static_objects->at(j) = obj;
        }
        std::cout<<"------------------------------static_objects size After: "<<static_objects->size()<<"!!!!!!!!!!"<<std::endl;
        std::cout<<"------------------------------static_objects have been set finished!!!"<<std::endl;

        // 添加模板区域
        for (int j = 0; j < count; j++)
        {
            CRectangle* p = templateplan_->CreateRect();
            float x = psts[j].x;
            float y = psts[j].y;
            float theta = psts[j].fThita;
            float ax[4] = { x+0.5*cos(theta)-0.1*sin(theta), x-0.5*cos(theta)-0.1*sin(theta), x+0.5*cos(theta)+2*sin(theta), x-0.5*cos(theta)+2*sin(theta) };
            float ay[4] = { y+0.5*sin(theta)+0.1*cos(theta), y-0.5*sin(theta)+0.1*cos(theta), y+0.5*sin(theta)-2*cos(theta), y-0.5*sin(theta)-2*cos(theta) };
            vector<float> bx(ax,ax+4);
            vector<float> by(ay,ay+4);
            vector<float>::iterator x_max = max_element(bx.begin(), bx.end());
            vector<float>::iterator x_min = min_element(bx.begin(), bx.end());
            vector<float>::iterator y_max = max_element(by.begin(), by.end());
            vector<float>::iterator y_min = min_element(by.begin(), by.end());
            if(fabs(tan(theta)) < 1.732 && fabs(tan(theta)) > 0.577)
            {
                if (!p->Create(*x_min+0.2, *y_max-0.2, *x_max-0.2, *y_min+0.2))
                    return false;
            }
            else
            {
                if (!p->Create(*x_min, *y_max, *x_max, *y_min))
                    return false;
            }
            CLocalizationRect::SetLocalizationMethods(methods_);
            CLocalizationRect *r = new CLocalizationRect(*p);
            // 该区域清空方法
            for (int i = 0; i < MAX_METHODS_PER_RECT; i++)
            {
                r->methodId_[i] = -1;
                r->param_[i] = NULL;
            }
            // 该区域启用模板法
            r->methodId_[0] = 2;
            r->param_[0] = methods_->CreateLocalizationParam(2);
            CTemplateLocalizationParam *param = dynamic_cast<CTemplateLocalizationParam*>(r->param_[0]);
            char buf[64];
            //buf[0]=(char)('0'+j);
            //buf[1] = '\0';
            if(j < 10)
            {
                buf[0]=(char)('0'+j);
                buf[1] = '\0';
            }
            else if(j >= 10 && j < 100)
            {
                buf[0]=(char)('0'+j/10);
                buf[1]=(char)('0'+j%10);
                buf[2] = '\0';
            }
            else
                return false;

            param->objNames[0] = buf;
            param->objId = j;
            templateplan_->push_back((CRectangle*)r);
            bx.clear();
            by.clear();


        }
        std::cout<<"------------------------------templateplan_ size() After: "<<(int)templateplan_->size()<<"!!!!!!!!!!"<<std::endl;
        std::cout<<"------------------------------static_objects have been set finished!!!"<<std::endl;
    }

    // 打开 FeatureMap.map 重新保存添加模板后的地图
    FILE *fp = fopen(WORK_PATH"FeatureMap.map", "wb");
    if (fp == NULL)
        return false;
    char version_[8] = "3.0.1.5";
    char date_[6] = {0, 0, 0, 0, 0, 0};

    // 先读取文件的版本和日期
    if (fwrite(version_, sizeof(char), 8, fp) != 8 || fwrite(date_, sizeof(char), 6, fp) != 6)
    {
        fclose(fp);
        return false;
    }
    if(SaveFeatureBinary(fp,WORK_PATH))
       fclose(fp);

    std::cout<<"New static objects have been saved!!!!"<<std::endl;
    return true;
}

bool CLocalizeFactory::ChangePlan(int plan_num, vector<vector<float> > plans, vector<int>type)
{
    std::lock_guard<std::mutex> lock1(loc_mtx);
    CLocalizationPlan *Pad_plan_ = plan_;
    // 清空现有模板区域
    for(int i=0; i < Pad_plan_->size(); i++)
    {
        CLocalizationRect *q =dynamic_cast<CLocalizationRect*>(Pad_plan_->at(i));
        if(q == NULL)
            continue;
        if(q->CheckMethodUsed(0)==0)
        {
           if(Pad_plan_->Delete(i))
           {
                std::cout<<"------------------------ NDTMD plan has been deleted!!!!!!"<<std::endl;
                i--;
           }
        }
        if(q->CheckMethodUsed(1)==0)
        {
           if(Pad_plan_->Delete(i))
           {
                std::cout<<"------------------------ FeatureMD plan has been deleted!!!!!!"<<std::endl;
                i--;
           }
        }
    }
    std::cout<<"--------------------- Plan_ size Before : "<<(int)Pad_plan_->size()<<"!!!!!!!!!!"<<std::endl;
    if(plan_num != 0)
    {
        for (int j = 0; j < plan_num; j++)
        {
            CRectangle* p = Pad_plan_->CreateRect();
            if (!p->Create( plans[j][1], plans[j][2], plans[j][3], plans[j][4]))
                return false;
            CLocalizationRect::SetLocalizationMethods(methods_);
            CLocalizationRect *r = new CLocalizationRect(*p);
            // 该区域清空方法
            for (int i = 0; i < MAX_METHODS_PER_RECT; i++)
            {
                r->methodId_[i] = -1;
                r->param_[i] = NULL;
            }
            // 该区域启用模板法
            r->methodId_[0] = type[j];
            r->param_[0] = methods_->CreateLocalizationParam(type[j]);
            // 方法参数可调（暂未启用）
            //CTemplateLocalizationParam *param = dynamic_cast<CTemplateLocalizationParam*>(r->param_[0]);
        }
        std::cout<<"------------------------------templateplan_ size() After: "<<(int)Pad_plan_->size()<<"!!!!!!!!!!"<<std::endl;
        std::cout<<"------------------------------create plan have been set finished!!!"<<std::endl;
    }
    // 打开 FeatureMap.map 重新保存添加模板后的地图
    FILE *fp = fopen(WORK_PATH"FeatureMap.map", "wb");
    if (fp == NULL)
        return false;
    char version_[8] = "3.0.1.5";
    char date_[6] = {0, 0, 0, 0, 0, 0};

    // 先读取文件的版本和日期
    if (fwrite(version_, sizeof(char), 8, fp) != 8 || fwrite(date_, sizeof(char), 6, fp) != 6)
    {
        fclose(fp);
        return false;
    }
    if(SaveBinary(fp,WORK_PATH))
       fclose(fp);

    std::cout<<"New plans have been saved!!!!"<<std::endl;
    return true;
}

//
// by lishen   把定位结果写到黑匣子里（将原来的分散写，统一到一个函数中）
// 定位结果 0 失败 1 成功  2 没有栅格地图
 void CLocalizeFactory::LocResultWriteBlackBox(int methodType, int locResult,CPosture &initPst,Eigen::Affine3d &resultPose)
 {
     std::string strLocFucntion("");
     switch(methodType)
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
         case CMatchInfo::LOC_GRID:
             strLocFucntion = "GRID";
             break;
         case CMatchInfo::LOC_SLAM:
             strLocFucntion = "SLAM";
             break;
     }
     if(locResult==0)
     {
         std::cout << "Loc Fail Types:"<< strLocFucntion <<" initPose: x_=" << initPst.x << ",y " << initPst.y << ",theta_=" << initPst.fThita<< std::endl;
         #ifdef USE_BLACK_BOX
         FILE_BlackBox(LocBox, "Loc Failed !! Types: " ,methodType ," InitPos : ", initPst.x, ", ", initPst.y, ", ", initPst.fThita);
         #endif
     }
     else if(locResult==1)
     {
         CPosture local_pst = AffineToPosture(resultPose);

         std::cout << "Loc Success Types:"<< strLocFucntion <<
                      " localPose: x_=" << local_pst.x << ",y_=" << local_pst.y <<
                      ",theta_=" << (local_pst.fThita / 3.14) * 180 << std::endl;

         #if defined USE_BLACK_BOX
             FILE_BlackBox(LocBox, "Loc Success!! Types : ", methodType , " LocalPose: x_=", local_pst.x  , ",y_=", local_pst.y, ",theta_=", (local_pst.fThita / 3.14) * 180,
                           " | ","initPose: x_=", initPst.x  , ",y_=", initPst.y, ",theta_=",(initPst.fThita / 3.14) * 180);
         #endif
     }

     else if(locResult ==2)
     {
         std::cout << "Loc faild Types:"<< strLocFucntion <<" GridMap is null  "<<std::endl;

         #ifdef USE_BLACK_BOX
             FILE_BlackBox(LocBox,"Loc Fail Types: ",methodType, " GridMap is Null !!!!");
         #endif
     }
     else if(locResult ==3)
     {
         std::cout << "Loc faild Types:"<< strLocFucntion <<" kkkkkk"<<std::endl;

         #ifdef USE_BLACK_BOX
             FILE_BlackBox(LocBox,"Loc Fail Types: ",methodType, "kkkkkkkkkkk");
         #endif
     }


 }


// By Sam
CMatchInfo *CLocalizeFactory::Localize(Eigen::Affine3d &estimatePose)
{
    // Init
    Eigen::Affine3d Tinit = estimatePose;

    // 汇总已收到的所有里程、激光数据
    CollectLaserCloud();

    // By Sam: For gridMethod
    CLocalizationMethod *method = NULL;

    //by lishen
    CSlamMethod  *pGridSlamMethod = (CSlamMethod *)methods_->at(4);
    CScanMatchMethod *pScanMatcher = (CScanMatchMethod *)methods_->at(3);
    pScanMatcher->SetFastMatch(&(((CNdtMethod *)methods_->at(0))->m_fastMatcher));

    // 取得对应于当前位姿的“定位指令”
    CPosture pst = AffineToPosture(Tinit);
    CLocalizationInst *locInst = plan_->GetInstructions(pst);

    int methodId = -1;
    int locModel = 1;
    int maxUseMethodNum = MAX_METHODS_PER_RECT;
    bool bWithSlam = false;
    bool bWithFeature = false;
    static CPosture pstFeatureFail;


    // 根据定位指令进行定位
    for (int i = 0; i < maxUseMethodNum; i++)
    {

        if (locInst != NULL)
        {

            CLocalizationInst &inst = locInst[i];

            m_topVisionMode = m_topVisionDefaultMode;

            // 方法编号应为非负数，且不应超出最大的定位方法编号
            if (inst.methodId_ < 0 || inst.methodId_ > NUM_LOCALIZATION_METHODS - 1)
                continue;           

            // 采用指定的定位方法
            method = methods_->at(inst.methodId_);
            if (method == NULL)
                continue;       

            // 应用指定的定位参数
            CLocalizationParam *param = static_cast<CLocalizationParam *>(inst.param_);
            if (param == NULL && (inst.methodId_ != 4 && inst.methodId_ != 3))
                continue;

            if(param != NULL && inst.methodId_ == 3)
            {
                CScanMatchParam *p = (CScanMatchParam*)param;
                m_topVisionMode = p-> m_topVisionMode;
            }

            if(!m_bInitLocSuccess && inst.methodId_==4)
            {
                continue;
            }

            CLocalizationInst &secondinst = locInst[1];
            if(secondinst.methodId_==4)
                bWithSlam = true;

            method->ApplyParam(param);

            CRectangle r = static_cast<CRectangle>(inst.localizationRect_);
            method->ApplyLocRect(&r);

            if(m_lastMethod == 4)
            {
                bool bflag = pGridSlamMethod->IsChangeToOtherMethod();
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
            m_topVisionMode = m_topVisionDefaultMode;

        }

        // 进行定位
        int ok = method->LocalizeProc(locModel, Tinit, cloudAdjusted, estimatePose);
        if (ok)
        {

            m_bFirstFeatureFail = true;
            // 评估定位质量
            float score;
            if (method->EvaluateQuality(score))  //??
            {
                LocResultWriteBlackBox(method->type_,ok,pst,estimatePose);
                if(methodId != 4)
                {
                    pGridSlamMethod->ResetOdom();
                    pGridSlamMethod->SetLastPos(cloudAdjusted.m_dwTimeStamp,estimatePose);
                }
                break;    // 成功，则不再尝试后续定位方法
            }
        }
        else
        {

            int matchresult = method->GetMatchInfo()->result_;

            if(matchresult != CMatchInfo::MATCH_TO_SINGLEFEATURE && ! bWithFeature)
                estimatePose = Tinit;

            if(matchresult == CMatchInfo::MATCH_LOADMAP_FAILED)
                 ok = 2;

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
                    #if defined USE_BLACK_BOX
                        FILE_BlackBox(LocBox, "In Long Corridor!!!----MATCH TO SINGLEFEATURE--- : ", dis , " / ", feature_param->m_MaxDisWithoutFeature);
                    #endif
                }
            }
            else if (!(bWithFeature&&i==0))
                m_bFirstFeatureFail = true;
            LocResultWriteBlackBox(method->type_,ok,pst,estimatePose);
        }

    }

    m_lastMethod = methodId;
    if(!bWithSlam)
         ((CSlamMethod *)methods_->at(4))->ResetOdom();

    return method->GetMatchInfo();

}

/*
bool CLocalizeFactory::LegLocalize(Eigen::Affine3d &estimatePose)
{
    // Init
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

        // 判断是否为模型方法或反光板方法
        if (inst.methodId_ == 2 || inst.methodId_ == 1)
        {
            legMethod = true;

            // 采用指定的定位方法
            method = methods_->at(inst.methodId_);
         //    method = methods_->at(1);
//            if (method == NULL)
//                return false;

            CLocalizationParam *param = static_cast<CLocalizationParam *>(inst.param_);

            if (param == NULL)
                return false;

            method->ApplyParam(param);

            CRectangle r = static_cast<CRectangle>(inst.localizationRect_);
            method->ApplyLocRect(&r);

           // bool legMethod = ((CFeatureMethod *)methods_->at(1))->GetLocParam();  // 程序内会进行货架腿定位方法的判断

            bool ok = method->LocalizeProc(locModel, Tinit, cloudAdjusted, estimatePose);

            if (ok)
            {
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "By Sam: Use LegMethod, Loc SUCCESS !!!");
#endif
                legPose = estimatePose;
                return true;
            }
            else
            {
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "By Sam: Use LegMethod, Loc FAILE !!!");
#endif
                return false;
            }
        }
    }

    if (!legMethod)
    {
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "By Sam: Not Use LegMethod, Reset LegMethod !!!");
#endif

        legPose = PostureToAffine(0, 0, 0);
        ((CFeatureMethod *)methods_->at(1))->ReSetMethod();
        ((CTemplateMethod *)methods_->at(2))->ReSetMethod();

        return false;
    }

}*/


int CLocalizeFactory::LegLocalize(Eigen::Affine3d &estimatePose)
{


    short bReflectCharge = 0;
    auto pRoboClnt = RoboClntSingleton::GetInstance();
    if(pRoboClnt)
        bReflectCharge = pRoboClnt->GetReflectChargeFlag();

   // std::cout<<"bReflectCharge     "<<bReflectCharge<<std::endl;




    if(bReflectCharge)
    {
        Eigen::Affine3d Tinit = estimatePose;
        CLocalizationMethod *method = NULL;

        // 取得对应于当前位姿的“定位指令”
        CPosture pst = AffineToPosture(Tinit);
        int locModel = 1;


        float x = pst.x;
        float y = pst.y;
        float theta = pst.fThita;


        float ax[4] = { x+0.7*sin(theta)-0.3*cos(theta), x-0.7*sin(theta)-0.3*cos(theta), x+0.7*sin(theta)+3*cos(theta), x-0.7*sin(theta)+3*cos(theta) };
        float ay[4] = { y+0.7*cos(theta)-0.3*sin(theta), y-0.7*cos(theta)-0.3*sin(theta),     y-0.7*cos(theta)+3*sin(theta), y+0.7*cos(theta)+3*sin(theta) };


        vector<float> bx(ax,ax+4);
        vector<float> by(ay,ay+4);
        vector<float>::iterator x_max = max_element(bx.begin(), bx.end());
        vector<float>::iterator x_min = min_element(bx.begin(), bx.end());
        vector<float>::iterator y_max = max_element(by.begin(), by.end());
        vector<float>::iterator y_min = min_element(by.begin(), by.end());

        CRectangle  rect(*x_min, *y_max, *x_max, *y_min);

       // std::cout<<" rect     "<<*x_min<< " "<<*y_max<<" "<<*x_max<<" "<<*y_min<<std::endl;

        CFeatureLocalizationParam legparam;
        legparam.ratio = 60;
        legparam.minRange = 0.1f;
        legparam.maxRange = 40.0f;
        legparam.maxRefLineRange  = legparam.maxRange / 3.0; //最大的直线参考集范围
        legparam.nLeastMatchCount_ = 3;
        legparam.nMostMatchCount_ = 5;
        legparam.fSamePointMaxDist_ = 0.1f;
        legparam.angleEqualLimit = CAngle::ToRadian(5);
        legparam.rangeEqualLimit = 0.3f;
        legparam.criteritonThreshold = 254; //反光板强度阈值
        legparam.refEfficientPointCount = 3;
        legparam.fLineEqualLimit_[0] = 0.01f;
        legparam.fLineEqualLimit_[1] = 0.02f;
        legparam.fLineEqualLimit_[2] = 0.05f;
        legparam.fLineEqualLimit_[3] = 0.08f;
        legparam.fLineEqualLimit_[4] = 0.1f;
        legparam.fMinLineLen_ = 0.30f; // 最短直线特征长度
        legparam.vecSpecialPntList_.clear();
        legparam.vecSpecialLineList_.clear();
        legparam.onlyUseInRectFeature = false;
        legparam.isUseUseMultiFrameLoc = false; //是否只应用多帧定位方法
        legparam.iMultiFrameLocDequeSize = 3; //多帧定位时应用的帧数
        legparam.fMaDisPowerfulValue = 0.13; //马氏距离阀值
        legparam.isUseSinglePointLoc = false; //是否应用单点定位方法
        legparam.m_MaxDisWithoutFeature = 0.5;

        CLocalizationParam *param = static_cast<CLocalizationParam *>(&legparam);

        method = methods_->at(1);
        method->ApplyParam(param);
        method->ApplyLocRect(&rect);

        // bool legMethod = ((CFeatureMethod *)methods_->at(1))->GetLocParam();  // 程序内会进行货架腿定位方法的判断

        bool ok = method->LocalizeProc(locModel, Tinit, cloudAdjusted, estimatePose);

        if (ok)
        {
    #ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "By Sam: Use LegMethod, Loc SUCCESS !!!");
    #endif
            legPose = estimatePose;
            return 1;
        }
        else
        {
    #ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "By Sam: Use LegMethod, Loc FAILE !!!");
    #endif
             return 2;
        }
    }
    else
    {
//#ifdef USE_BLACK_BOX
 //   FILE_BlackBox(LocBox, "By Sam: Not Use LegMethod, Reset LegMethod !!!");
//#endif

        legPose = PostureToAffine(0, 0, 0);
        ((CFeatureMethod *)methods_->at(1))->ReSetMethod();
        ((CTemplateMethod *)methods_->at(2))->ReSetMethod();

        return 0;
    }

}

} // namespace robo

#pragma GCC pop_options
