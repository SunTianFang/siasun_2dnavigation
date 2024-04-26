//
//   The interface of class "CLaserMapping".
//

#include"LaserAutoMapping.h"
#include <stdio.h>
#include <fstream>
#include "Tools.h"
#include "RawMap.h"
#include "json/json.h"
#include "BaseOdometry.h"
#include "RoboLocProto.h"
#include "RoboLocClnt.h"
#include "ParameterObject.h"
#include "blackboxhelper.hpp"

#include "ndt_pointcloud.h"
#include "NdtExtLocalization_oru.h"
#include "ndt_pointcloud.h"

#include "BuildMap.h"
#include "RoboManager.h"
#include "LocalizeFactory.h"
#include <malloc.h>
// dq VISION 20230707
#include "topcampub.h"
#include <iostream>
#include <iomanip>
#include <cmath>


#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

////////////////////////////////////////////////
//   实现自动建图功能
//   采用了cartographer中slam实现自动建图
//   Author: lishen
//   Date:   2022. 5.
///////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//   The support routine of laser mapping.
//
//  by  lishen
//

static int cccc = 0;
void* LaserAutoMappingSupportProc(LPVOID pParam)
{
    int time_diff  = 0;     // 每次循环执行时间
    int ctrl_cycle = 0;     // 每次循环休眠时间

    mapping::CLaserAutoMapping* pMapping = reinterpret_cast<mapping::CLaserAutoMapping*>(pParam);

     pMapping->m_bFirstScan = true;

     if(pMapping->m_mode == mapping::Mode_ExpandMap)
     {

             if(pMapping->LoadPbFile(NULL))
             {

                SubmapList submaplist;
                pMapping->m_pCartoSlam->GetSubmapList(submaplist);

                int size = submaplist.submap.size();

                vector<int> submapids;
                for(auto submap : submaplist.submap)
                {
                    submap_data pdata;
                    vector <double> blackcell_index;

                    int submap_index = submap.submap_index;

                    transform::Rigid3d global_pose = submaplist.submap.at(submap_index).pose;
                    pMapping->m_pCartoSlam->GetSubmapData_upload(submap_index, global_pose, &pdata, blackcell_index);

                    if(blackcell_index.size()>5)
                        submapids.push_back(submap_index);
                }

                int num=0;
                pMapping->SendSubMaps(num, nullptr);
                usleep(50000);

                int count = 0;
                pMapping->m_bPadAnswer = false;

                //pMapping->m_status =  mapping::Mapping_MAP_WAITING_EXPAND;

                while((count<5) && (!pMapping->m_bPadAnswer))
                {
                    std::cout<<"send lcm " << std::endl;
                    LCMTask::GetLcmInstance().SendLoadPbResult(1,submapids);
                    usleep(200000);
                    count++;
                }
             }
             else
             {
                 std::cout<<"LoadPbFile faild " << std::endl;
                vector<int> submapids;
                int count = 0;
                pMapping->m_bPadAnswer = false;
                while((count<5) && (!pMapping->m_bPadAnswer))
                {
                    std::cout<<"send lcm " << std::endl;
                    LCMTask::GetLcmInstance().SendLoadPbResult(0, submapids);
                    usleep(200000);
                    count++;
                }

                std::cout<<"LoadPbFile faild " << std::endl;
                // pMapping->m_status = mapping::Mapping_MAP_READPB_FAILED;
               // return false;
             }

           pMapping->m_bWaitingExpand = true;
           WaitForSingleObject ( pMapping->m_hExpandMap, INFINITY );
     }

    while (WaitForSingleObject(pMapping->m_hKillThread, 0) != WAIT_OBJECT_0)
    {
		// dq VISION
        //std::cout<<"*************************************************************************************************************"<<std::endl;
       pMapping->m_status =  mapping::Mapping_MAP_BUILDING;

        pMapping->MappingProc();
        pMapping->m_bWaitingExpand = false;

        // 根据执行时间,灵活设置休眠时间
        time_diff = static_cast<int>(pMapping->m_mappingEndTime.load() - pMapping->m_mappingCurTime.load());
        if ( time_diff < 0 ) {
            time_diff = 0;
        }
        // std::cout<<"time_diff: "<<time_diff<<std::endl;
        //ctrl_cycle = AUTO_MAPPING_MAX_CTRL_CYCLE - time_diff;
        ctrl_cycle = 200 - time_diff;
        if ( ctrl_cycle < 0 ) {
            ctrl_cycle = 0;
        }
        //ctrl_cycle = std::min ( ctrl_cycle, AUTO_MAPPING_MAX_CTRL_CYCLE );
        //ctrl_cycle = std::max ( ctrl_cycle, AUTO_MAPPING_MIN_CTRL_CYCLE );
        //std::cout<<"ctrl_cycle: "<<ctrl_cycle<<std::endl;
        //Sleep(ctrl_cycle);
    }

    SetEvent ( pMapping->m_hThreadDead );
    pthread_exit(NULL);

    return NULL;
}

namespace mapping {

CLaserAutoMapping *pLaserAutoMappingObj;
//
//  by  lishen  构造函数
//

CLaserAutoMapping::CLaserAutoMapping()
{
    m_hKillThread    = NULL;
    m_hThreadDead    = NULL;
    m_pMappingThread = 0;
    m_bStarted       = false;
    m_nStartTime     = 0;
    m_bFirstTime     = false;
    m_aWorkMode      = RLP_MODE_STANDBY;
    m_CurTimeStamp   = 0;
    m_PreTimeStamp   = 0;
    m_aFileSaving    = false;

    m_bMapping       = false;

    m_bInit          = false;
    m_mappingEndTime = 0;
    m_mappingCurTime = 0;
    pLaserAutoMappingObj = this;

    m_bSaving = false;

    m_status = Mapping_STAND_BY;

    m_bFirstScan = true;

    m_pCartoSlam = nullptr;

    m_bStopping = false;

    m_SetPose = true;

    m_bPadAnswer = false;

   // m_bExpandMap = false;

    m_mode = Mode_BuildMap;

    m_bFrozenNode = false;

    m_bWaitingExpand = false;

    m_bCameraFlag = false;

}

// 析构函数
CLaserAutoMapping::~CLaserAutoMapping()
{
    Stop();
}

//
// by lishen 初始化 读激光参数， 实例化carto类，注册carto类回调函数
//
bool CLaserAutoMapping::Initialize()
{
    if ( m_bInit )
    {
        if(m_pCartoSlam!=nullptr)
        {
            m_pCartoSlam -> RegisterSlamCallBack ( (SlamResultCbFunc)CartoSlamCallback );
            m_pCartoSlam -> RegisterBuildOverCallBack((BuildOverFunc)BuildMapOverCallback);
            m_pCartoSlam -> AddLaserID( (std::string)("scan0") );

            auto pCartoParm = CartoParmSingleton::GetInstance();
            pCartoParm->SetDefaultCartoParm();
            pCartoParm->LoadJsonForParam();

        }
        return true;
    }

    auto pAFamily = SensorFamilySingleton::GetInstance();
    sensor::CSensorData* sensor_data = NULL;

    m_scannerParam.clear();
    for ( unsigned int m = 0; m < pAFamily->GetCount(); m++ ) {
        sensor_data = pAFamily->GetSensorData(m);
        if ( sensor_data && sensor_data->parm  && (pAFamily->GetState(m))) {
            m_scannerParam.push_back(*(sensor_data->parm));
        }
    }


    LaserNaviConfig config;

    if ( m_pCartoSlam == nullptr )
    {
        m_pCartoSlam = CCartoSlam::GetInstance();
    }

    m_bInit = true;

    return true;
}





//
// lishen 自动建图周期函数
//
void CLaserAutoMapping::MappingProc()
{
    //timeval time1;
    //timeval time5;
    //gettimeofday(&time1,NULL);
	// dq VISION
    m_mappingCurTime =  GetTickCount();
    sensor::CRawScan rawScan;
    if ( UpdateScan(rawScan) )
    {
       // std::cout << "UpdateScan: " << GetTickCount() << std::endl;
        if(m_bCameraFlag)
        {
            time_record.push_back(GetTickCount());
            cv::Mat img;
            tp.GetFrame(img);
            img_record.push_back(img);
        }
       // std::cout << "$$$$$$ save over: " << GetTickCount() << std::endl;
        AddScan(rawScan);
    }

    // 发送位姿
    short         error_code = RLP_NO_ERROR;
    unsigned char pos_mode   = RLP_POSITIONING_STOPPED;    // ????

    auto pRoboClnt = RoboClntSingleton::GetInstance();
    if ( pRoboClnt ) {


#ifdef USE_LEG_METHOD
        pRoboClnt->SetRoboPose(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, error_code, error_code, pos_mode); // By Sam for LegMethod#
#else
        pRoboClnt->SetRoboPose(0, 0, 0, 0, 0, 0, error_code, pos_mode);
#endif

    }

    m_mappingEndTime =  GetTickCount();

#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "IN MAPPING MODE!!!!!!!!!!!!!!!!!!!!");
#endif
    //gettimeofday(&time5,NULL);

    //static int count = 0;
    //count++;
    //if(count%100 ==0)
    // printf(" timer=   %f\n", (time5.tv_sec - time1.tv_sec)*1000 + (double)(time5.tv_usec -time1.tv_usec)/1000  );

}

void CLaserAutoMapping::ExpandMap()
{
    SetEvent ( m_hExpandMap );
}
//
//  by  lishen  stop  create map
//

bool CLaserAutoMapping::Stop()
{

  //  if(m_status != Mapping_MAP_BUILDING)
    //    return false;

    if(m_status != Mapping_MAP_BUILDING )
    {
        if(m_status != mapping::Mapping_MAP_READPB_FAILED )
        {
            if(m_status != Mapping_MAP_WAITING_EXPAND)
                return false;
        }
    }

    if(m_mode == Mode_ExpandMap)
        SetEvent ( m_hExpandMap );

    SetEvent ( m_hKillThread );
    WaitForSingleObject ( m_hThreadDead, 5000 );
    PthreadJoin ( m_pMappingThread );

    if ( m_hKillThread != NULL )
    {
        CloseHandle ( m_hKillThread );
        m_hKillThread = NULL;
    }

    if ( m_hThreadDead != NULL )
    {
        CloseHandle ( m_hThreadDead );
        m_hThreadDead = NULL;
    }

    m_pMappingThread = 0;
    m_nStartTime     = 0;

    return true;
}

//
// lishen 更新原始数据帧
//
#ifdef ONE_LASER
bool CLaserAutoMapping::UpdateScan(sensor::CRawScan& rawScan)
{
    bool     result = false;
    bool     bRet   = false;


    auto                pOdometry   = BaseOdomSingleton::GetInstance();
    double              acc_odom    = static_cast<double>(pOdometry->GetAccumuOdom()) / 100000.0;
    // std::cout<<"acc_odom: "<<acc_odom<<std::endl;
    // dq VISION 0.2m 采集
    if(!m_bFirstTime && acc_odom < 0.2) {
        return false;
    }
    if(m_bFirstTime)
        m_bFirstTime = false;




    // 获取激光传感器点云数据
    bool bActiveCloud = true;
    auto pAFamily = SensorFamilySingleton::GetInstance();
    std::shared_ptr < sensor::CRawPointCloud > pRawCloud;

    for ( unsigned int i = 0; i < pAFamily->GetCount(); i++) {
        if(!pAFamily->GetState(i)|| (pAFamily->GetType(i) == LEIMOUF30)) {
            continue;
        }

        if ( !pAFamily->GetRawPointCloud(i, pRawCloud) ) {
            bActiveCloud = false;   // ????
            // continue;
        }
        uint64_t r = pRawCloud->timestamp_raw;

        while(pAFamily->GetRawPointCloud(i, pRawCloud))
        {
            if(pRawCloud->timestamp_raw != r)
                break;
            Sleep(3);
        }

        if ( i == 0 ) {
            rawScan.PushBackPointCloud(pRawCloud);
            result = true;
            break;
        }

        //pRawCloud.reset();
        //pRawCloud = nullptr;
    }

    CPosture odom_pst;
    unsigned long long  timeNow     = GetTickCount();
    /*
    auto                pOdometry   = BaseOdomSingleton::GetInstance();
    double              acc_odom    = static_cast<double>(pOdometry->GetAccumuOdom()) / 100000.0;
    std::cout<<"acc_odom: "<<acc_odom<<std::endl;
    if(!m_bFirstTime && acc_odom < 0.05) {
        return false;
    }
    if(m_bFirstTime)
        m_bFirstTime = false;
     */

    bRet = pOdometry->GetOdomPosture(timeNow, odom_pst);

    pOdometry->SetAccumuOdom(0);

    // 获取机器人的相对里程姿态,当前速度矢量,里程计坐标系的全局位姿
    sensor::COdometryData odom_data;

    CPosture odom_trans;
    StVelocity base_vel;

    odom_data.odom_flag = pOdometry->GetOdomFlag();
    odom_data.time_stamp = timeNow;
    base_vel = pOdometry->GetBaseVelocity();
    odom_data.velocity.fXLinear = base_vel.Vx / 1000.0;
    odom_data.velocity.fYLinear = base_vel.Vy / 1000.0;
    odom_data.velocity.fAngular = base_vel.Vtheta / 1000.0;

    // 采集建模数据时,根据时间获取位姿,计算两次的位姿变换量,因为车辆中途会长时间停滞,导致按照时间间隔获取位姿变化会出错.
    bRet = pOdometry->GetOdomPosture(timeNow, odom_pst);
    if(bRet){
        m_CurOdomPst = odom_pst;
    }


    odom_data.global_pst = m_CurOdomPst;

    rawScan.SetOdometry(odom_data);

    // 导航控制器和激光传感器通信中断,上报AGV    //??? send to pad
    short         error_code = RLP_NO_ERROR;
    unsigned char pos_mode   = RLP_POSITIONING_STOPPED;
    if ( pAFamily->IsBlocked() ) {
        error_code = RLP_LASER_TIMEOUT;
    }

    //m_PreOdomPst1 = m_CurOdomPst;
    m_PreTimeStamp = m_CurTimeStamp.load();
    
    return result;
}


#else
bool CLaserAutoMapping::UpdateScan(sensor::CRawScan& rawScan)
{
    bool     result = false;
    bool     bRet   = false;


    auto                pOdometry   = BaseOdomSingleton::GetInstance();
    double              acc_odom    = static_cast<double>(pOdometry->GetAccumuOdom()) / 100000.0;

    // dq VISION 0.2m 采集

    if(!m_bFirstTime && acc_odom < 0.2) {
      return false;
    }
    if(m_bFirstTime)
        m_bFirstTime = false;

    // 获取激光传感器点云数据
    bool bActiveCloud = true;
    auto pAFamily = SensorFamilySingleton::GetInstance();
    std::shared_ptr < sensor::CRawPointCloud > pRawCloud;

    for ( unsigned int i = 0; i < pAFamily->GetCount(); i++) {
        if(!pAFamily->GetState(i)|| (pAFamily->GetType(i) == LEIMOUF30)) {
            continue;
        }

        if ( !pAFamily->GetRawPointCloud(i, pRawCloud) ) {
            bActiveCloud = false;   // ????
            // continue;
        }
        uint64_t r = pRawCloud->timestamp_raw;

        while(pAFamily->GetRawPointCloud(i, pRawCloud) && i==0)
        {
            if(pRawCloud->timestamp_raw != r)
                break;
            Sleep(3);
        }

        rawScan.PushBackPointCloud(pRawCloud);
        result = true;

    }

    CPosture odom_pst;
    unsigned long long  timeNow     = GetTickCount();
    /*
    auto                pOdometry   = BaseOdomSingleton::GetInstance();
    double              acc_odom    = static_cast<double>(pOdometry->GetAccumuOdom()) / 100000.0;
    std::cout<<"acc_odom: "<<acc_odom<<std::endl;
    if(!m_bFirstTime && acc_odom < 0.05) {
        return false;
    }
    if(m_bFirstTime)
        m_bFirstTime = false;
     */

    bRet = pOdometry->GetOdomPosture(timeNow, odom_pst);

    pOdometry->SetAccumuOdom(0);

    // 获取机器人的相对里程姿态,当前速度矢量,里程计坐标系的全局位姿
    sensor::COdometryData odom_data;

    CPosture odom_trans;
    StVelocity base_vel;

    odom_data.odom_flag = pOdometry->GetOdomFlag();
    odom_data.time_stamp = timeNow;
    base_vel = pOdometry->GetBaseVelocity();
    odom_data.velocity.fXLinear = base_vel.Vx / 1000.0;
    odom_data.velocity.fYLinear = base_vel.Vy / 1000.0;
    odom_data.velocity.fAngular = base_vel.Vtheta / 1000.0;

    // 采集建模数据时,根据时间获取位姿,计算两次的位姿变换量,因为车辆中途会长时间停滞,导致按照时间间隔获取位姿变化会出错.
    bRet = pOdometry->GetOdomPosture(timeNow, odom_pst);
    if(bRet){
        m_CurOdomPst = odom_pst;
    }


    odom_data.global_pst = m_CurOdomPst;

    rawScan.SetOdometry(odom_data);

    // 导航控制器和激光传感器通信中断,上报AGV    //??? send to pad
    short         error_code = RLP_NO_ERROR;
    unsigned char pos_mode   = RLP_POSITIONING_STOPPED;
    if ( pAFamily->IsBlocked() ) {
        error_code = RLP_LASER_TIMEOUT;
    }

    //m_PreOdomPst1 = m_CurOdomPst;
    m_PreTimeStamp = m_CurTimeStamp.load();

    return result;
}

#endif

//
// lishen carto类 处理激光和码盘数据
//
bool CLaserAutoMapping::AddScan(sensor::CRawScan& rawScan)
{
    laserscan_msg lscan;

    if(m_scannerParam.size()<=0)
        return false;

    m_dequeRawScan.push_back(rawScan);

    TransformRawCloudToLaserMsg(rawScan,lscan);

    // std::cout << "angle_min"<< lscan.angle_min<<" "<<lscan.angle_max<<" "<<lscan.angle_increment<<std::endl;
    // std::cout << "laser tf "<<lasertf.x<<" "<<lasertf.y<<" "<<lasertf.theta<<std::endl;
   //  std::cout << "lscan.stamp "<<lscan.stamp<<std::endl;


    odometry_msg odom;

    odom.pos.x      = rawScan.odom_data.global_pst.x;
    odom.pos.y      = rawScan.odom_data.global_pst.y;
    odom.pos.theta  = rawScan.odom_data.global_pst.fThita;


    // std::cout << "lscan.stamp "<<lscan.stamp<<std::endl;
    // std::cout << "odom 1 "<< odom.pos.x<<" "<<odom.pos.y<<" "<<180.0*odom.pos.theta/3.1415<<std::endl;
    // odom.stamp = (double)(rawScan.odom_data.time_stamp - 1) / 1000.0;     // sec

    odom.stamp = (double)(lscan.stamp - 2) / 1000.0;     // sec

    // std::cout << "rawScan.odom_data.time_stamp "<<rawScan.odom_data.time_stamp<<std::endl;

    // 第一帧创建地图 add 2022.06.09
    if ( m_bFirstScan ) {

        if ( m_pCartoSlam != nullptr )
        {
            auto pCartoParm = CartoParmSingleton::GetInstance();
            pCartoParm->SetDefaultCartoParm();


            pCartoParm->LoadJsonForParam();
            m_pCartoSlam -> RegisterSlamCallBack ( (SlamResultCbFunc)CartoSlamCallback );
            m_pCartoSlam -> RegisterBuildOverCallBack((BuildOverFunc)BuildMapOverCallback);
            if(m_mode == Mode_BuildMap)
            {
                 std::cout<<"m_pCartoSlam->createMap"<<std::endl;
                m_pCartoSlam->createMap();
            }
            if(m_mode == Mode_ExpandMap || m_mode == Mode_UpdateMap)
            {
                //????
                double stamp = (double)(lscan.stamp) / 1000.0;     // sec
                  std::cout<<"m_pCartoSlam-> begin StartExpandMapnew"<<std::endl;
                m_pCartoSlam->StartExpandMapnew(m_expandInitPos,nullptr, stamp);

            }
        }

        m_bFirstScan = false;
    }

    if ( m_pCartoSlam != nullptr && lscan.ranges.size() > 0 )
    {
        //std::cout<<"m_pCartoSlam->HandleLaserData"<<std::endl;
        m_pCartoSlam->HandleEncoderData ( &odom );
        m_pCartoSlam->HandleLaserData ( &lscan );

    }
}

//
// by lishen 转换点云 为laserMsg结构体形式
//
bool CLaserAutoMapping::TransformRawCloudToLaserMsg(const sensor::CRawScan& rawScan, laserscan_msg &lscan)
{
    lscan.sensor_id = (std::string)("scan0");
    lscan.angle_increment = m_scannerParam.at(0).m_fReso;
    lscan.angle_max = m_scannerParam.at(0).m_fEndAngle;
    lscan.angle_min = m_scannerParam.at(0).m_fStartAngle;
    lscan.range_max = m_scannerParam.at(0).m_fMaxRange;
    lscan.range_min = m_scannerParam.at(0).m_fMinRange;
    lscan.scan_time = 0.02;   //50hz
    lscan.time_increment = 0.000005;

    Pose lasertf;
    lasertf.x       = m_scannerParam.at(0).m_pst.x;
    lasertf.y       = m_scannerParam.at(0).m_pst.y;
    lasertf.theta   = m_scannerParam.at(0).m_pst.fThita ;

    lscan.laser_tf = lasertf;

    for(auto r: rawScan.point_cloud[0]->distance)
    {
        lscan.ranges.push_back(float(r)/1000.0);  //???
    }

    lscan.stamp = rawScan.point_cloud[0]->timestamp_raw;

}

//
// by lishen 转换点云 为LaserScan结构体形式
//
bool CLaserAutoMapping::TransformRawCloudLaserScan(const sensor::CRawScan& rawScan, sensor_msgs::LaserScan &lscan)
{


   transform::Rigid2d  loc;
    sensor_msgs::LaserScan msg;

    Pose lasertf;
    lasertf.x       = m_scannerParam.at(0).m_pst.x;
    lasertf.y       = m_scannerParam.at(0).m_pst.y;
    lasertf.theta   = m_scannerParam.at(0).m_pst.fThita ;


   // long int sec = (long int)(pmsg->stamp/1000.0);  //ms   to   sec
   // long int usec = (long int)(((double)pmsg->stamp/1000.0-sec)*1000000ll);

    long int sec = (long int)(rawScan.point_cloud[0]->timestamp_raw/1000.0);  //ms   to   sec
    long int usec = (rawScan.point_cloud[0]->timestamp_raw%1000)*1000;

    msg.header.stamp.SetTime(sec , usec);

    msg.angle_min = m_scannerParam.at(0).m_fStartAngle;
    msg.angle_max = m_scannerParam.at(0).m_fEndAngle;
    msg.angle_increment = m_scannerParam.at(0).m_fReso;

    msg.time_increment = 0.000005;

    msg.scan_time = 0.02;

    msg.range_min = m_scannerParam.at(0).m_fMinRange;
    msg.range_max = m_scannerParam.at(0).m_fMaxRange;


    for(auto r: rawScan.point_cloud[0]->distance)
        msg.ranges.push_back(float(r)/1000.0);  //???


    //msg.intensities= pmsg->intensities;
    msg.sensor_id =  (std::string)("scan0");

    Eigen::Matrix<double, 3, 1> tf(lasertf.x, lasertf.y,lasertf.theta) ;
    msg.laser_tf = tf;

}

//
//by lishen 转换点云 为Scan结构体形式
//


bool CLaserAutoMapping::TransformRawCloudToScan(const CPosture &pstRobot,  const CScannerGroupParam &Params,
                                             const sensor::CRawScan& rawScan, vector<CScan> &scans)
{
    // 根据激光器在机器人上的相对安装位置计算其在世界坐标系下的姿态

    if ( rawScan.point_cloud.size() <= 0 )
        return false;

    scans.clear();
    scans.resize(Params.size());


    for(int m=0;m<Params.size();m++)
    {

        std::shared_ptr<sensor::CRawPointCloud> pRawCloud = rawScan.point_cloud.at(m);

       // CScan scan;

        CFrame frmRobot ( pstRobot );
        CPosture pstScanner ( Params.at(m).m_pst );            // 激光器相对机器人的安装姿态
        pstScanner.InvTransform ( frmRobot );


        scans[m].m_pstScanner.SetPosture( pstScanner );     // ??????????????

        scans[m].m_poseRelative = scans[m].m_pstScanner;
        scans[m].m_fStartAng    = Params.at(m).m_fStartAngle;
        scans[m].m_fEndAng      = Params.at(m).m_fEndAngle;

        // 计算扫描的角分辨率
        float fAngReso = (Params.at(m).m_fEndAngle - Params.at(m).m_fStartAngle) / Params.at(m).m_nLineCount;


        scans[m].Clear();
        if ( !scans[m].Create(Params.at(m).m_nLineCount) )
            return false;

        scans[m].Stamp(pRawCloud->timestamp_raw);

        // 依次读入所有点的数据
        for (int i = 0; i < pRawCloud->num_points; i++)
        {
            float r = pRawCloud->distance[i] / 1000.0;

           // if (r < Param.m_fMinRange || r > Param.m_fMaxRange )    // r > 65500为临时措施，解决扫描线出现一圈半径为65534的弧的问题
           //     r = 0;



            if (r < Params.at(m).m_fMinRange )
                r = 0;
             if(r > Params.at(m).m_fMaxRange)
                 r = Params.at(m).m_fMaxRange+2;

            // 过虑掉不满足可视角度的点

            CScanPoint &sp = scans[m].m_pPoints[i];

            sp.r = r;
            sp.a = scans[m].m_fStartAng + fAngReso * i;
            sp.m_nIntensity = pRawCloud->intensity.size();

            if ( !Params.at(m).m_AppAngleRange.Contain(sp.a) ) {   // ?????
                r = 0; sp.r = 0.0;
            }

            // 极径大于0，表示数据有效
            if ( r >= 0 )
            {
                // sp.r /= 1000;          // 目前文件格式以mm为单位，需要转换为m
                // 计算迪卡尔坐标
                sp.UpdateCartisian();

            }

    #if 0
            // 处理无效点
            else
                sp.x = sp.y = 0;
    #endif

            sp.id = i;
        }

        std::cout<<" scans[m] m= "<< m << "  count=  " <<  scans[m].m_nCount <<std::endl;

        //scans.push_back(scan);
    }
}


bool CLaserAutoMapping::TransformRawCloudToScan(const CPosture &pstRobot,  const CLaserScannerParam &Param,
                                             const sensor::CRawScan& rawScan, CScan &scan)
{
    // 根据激光器在机器人上的相对安装位置计算其在世界坐标系下的姿态

    if ( rawScan.point_cloud.size() <= 0 )
        return false;

    std::shared_ptr<sensor::CRawPointCloud> pRawCloud = rawScan.point_cloud.at(0);


    CFrame frmRobot ( pstRobot );
    CPosture pstScanner ( Param.m_pst );            // 激光器相对机器人的安装姿态
    pstScanner.InvTransform ( frmRobot );


    scan.m_pstScanner.SetPosture( pstScanner );     // ??????????????

    scan.m_poseRelative = scan.m_pstScanner;
    scan.m_fStartAng    = Param.m_fStartAngle;
    scan.m_fEndAng      = Param.m_fEndAngle;

    // 计算扫描的角分辨率
    float fAngReso = (Param.m_fEndAngle - Param.m_fStartAngle) / Param.m_nLineCount;


    scan.Clear();
    if ( !scan.Create(Param.m_nLineCount) )
        return false;

    scan.Stamp(pRawCloud->timestamp_raw);

    // 依次读入所有点的数据
    for (int i = 0; i < pRawCloud->num_points; i++)
    {
        float r = pRawCloud->distance[i] / 1000.0;

       // if (r < Param.m_fMinRange || r > Param.m_fMaxRange )    // r > 65500为临时措施，解决扫描线出现一圈半径为65534的弧的问题
       //     r = 0;



        if (r < Param.m_fMinRange )
            r = 0;
         if(r > Param.m_fMaxRange)
             r = Param.m_fMaxRange+2;

        // 过虑掉不满足可视角度的点

        CScanPoint &sp = scan.m_pPoints[i];

        sp.r = r;
        sp.a = scan.m_fStartAng + fAngReso * i;
        sp.m_nIntensity = pRawCloud->intensity.size();

        if ( !m_scannerParam.at(0).m_AppAngleRange.Contain(sp.a) ) {   // ?????
            r = 0; sp.r = 0.0;         
        }

        // 极径大于0，表示数据有效
        if ( r >= 0 )
        {
            // sp.r /= 1000;          // 目前文件格式以mm为单位，需要转换为m
            // 计算迪卡尔坐标
            sp.UpdateCartisian();

        }

#if 0
        // 处理无效点
        else
            sp.x = sp.y = 0;
#endif

        sp.id = i;
    }
}



//
// by lishen 开始建图
//
bool CLaserAutoMapping::StartMapping()
{
    std::lock_guard<std::mutex> lock(build_mtx);
	
    if(m_status != Mapping_STAND_BY)
        return true;

    auto pAFamily = SensorFamilySingleton::GetInstance();

    // 导航控制器和激光传感器通信中断,上报AGV PAD   // ??? send to pad
    short         error_code = RLP_NO_ERROR;
    unsigned char pos_mode   = RLP_POSITIONING_STOPPED;
    if ( pAFamily->IsBlocked() ) {
        error_code = RLP_LASER_TIMEOUT;
        std::cout << "laser error*******************" << std::endl;
   //     m_bStarted   = false;
        m_status = Mapping_STAND_BY;
        return false;
    }



    //read param
    m_nStartTime = static_cast<unsigned int>(GetTickCount() - 200);

    // clear the odometry
    auto pOdometry = BaseOdomSingleton::GetInstance();
    CPosture odom_trans;
    pOdometry->SetAccumuOdom(0);    // 单位为毫米
    // pOdometry->GetLocalOdomTrans(odom_trans);

    // get the mapping dataset size   ///???????
    int dataset_size = RAW_MAP_CAPACITY_MAPPING;
    auto pParameterObject = ParameterObjectSingleton::GetInstance();
    pParameterObject->GetParameterValue("Mapping_MappingDataSetSize", dataset_size);

    // clear the raw map space
    auto pRawMap = RawMapSingleton::GetInstance();
    if(m_mode == Mode_BuildMap)
    {
        pRawMap->Clear();
    }
    pRawMap->SetMaxCount(dataset_size);
    pRawMap->SetStartTime(m_nStartTime);
    m_bWaitingExpand = false;

    m_rawScans.clear();
    m_dequeRawScan.clear();

    // Init signal events
    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hKillThread == NULL)
        return false;

    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hThreadDead == NULL)
        return false;

    m_hExpandMap = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (m_hExpandMap == NULL)
        return false;

     m_bFirstTime = true;



      auto pLocalize = LocalizeFactorySingleton::GetInstance();

      m_bCameraFlag = pLocalize->m_bFlagCameraLoc;

      if(m_bCameraFlag)
      {
         // dq VISION
         // 发送消息通知进程暂停定位模式
         auto pRoboClnt = RoboClntSingleton::GetInstance();
         pRoboClnt->SetMappingMode_Cam();
         usleep(2000*1000);
         /*while(1)
         {
            int i = pRoboClnt->GetCamMode();
            std::cout<<"pRoboClnt->GetCamMode(): "<<i<<std::endl;
            if(i == 0)
                break;
            usleep(200000);
         }*/

         // 相机初始化
         rtn = tp.Init();
         if (rtn)
         {
             cout << "Open camera success!!!" << endl;
            #ifdef USE_BLACK_BOX
                FILE_BlackBox(LocBox, "Open camera success!!!");
            #endif
             m_bCameraFlag = true;
         }
         else
         {
             cout << "Open camera failed!!!" << endl;
            #ifdef USE_BLACK_BOX
                FILE_BlackBox(LocBox, "Open camera failed!!!");
            #endif
             m_bCameraFlag = false;
             return false;
         }

      }
     if(m_bCameraFlag)
     {
         cout << "CAM_ID: " << tp.getCamId() <<" has been opened!!! sleep 30ms" <<endl;
         usleep(30000);
         for(int i = 1; i < 11; i ++)
         {
             cv::Mat img;
             tp.GetFrame(img);
             std::cout<<"Test!!! "<<i<<" / 10"<<std::endl;
         }
     }
     cccc = 0;
    // Start the support procedure
    pthread_attr_t attr;
    SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attr);
    if(pthread_create(&m_pMappingThread, &attr, LaserAutoMappingSupportProc, reinterpret_cast<LPVOID>(this)) != 0)
    {
        std::cout<<"Creat LaserAutoMappingSupportProc Pthread Failed"<<std::endl;
        return false;
    }
    else
    {
        std::cout<<"Creat LaserAutoMappingSupportProc Pthread OK"<<std::endl;
    }

    pthread_attr_destroy(&attr);

    m_status = Mapping_MAP_BUILDING;



    return true;
}
//
// by lishen 创建地图过程回调函数
//
void CLaserAutoMapping::CartoSlamCallback(slam_result *slam, std::vector<opt_result> *opt)
{
    std::cout<<"CartoSlamCallback"<<std::endl;
    pLaserAutoMappingObj->HandleOptPose ( slam, opt );
}

//
// by lishen 创建地图过程回调函数 得到机器人当前位姿，优化结果等，发送pad显示
//
void CLaserAutoMapping::HandleOptPose(slam_result *slam, std::vector<opt_result> *opt)
{
    // 建模数据集
    timeval stNode = slam->stamp;
    auto pRawMap = RawMapSingleton::GetInstance();

    while (!m_dequeRawScan.empty())
    {
        sensor::CRawScan scan;
        scan = m_dequeRawScan.front();

        if ( slam->nodeId != -1 ) // ?????
        {
            long int sec  = (long int)(scan.point_cloud[0]->timestamp_raw / 1000.0);  // ms   to   sec
            long int usec = (scan.point_cloud[0]->timestamp_raw % 1000) * 1000;


            if((long int)stNode.tv_sec == sec && (long int)stNode.tv_usec == usec)
            {

                Pose xyt = slam->local_pose;

                std::cout<<"node:  "<<slam->nodeId<<"   "<<xyt.x<<"   "<<xyt.y<< "   "<<xyt.theta<<std::endl;

                //   save dx               //?????????? if there are two lasers
                 if ( slam->nodeId == 0 ) {  //first scan
                     m_PreOdomPst = scan.odom_data.global_pst;

                 }
                 CTransform transOdom;
                 CPosture odom_trans ;
                 transOdom.Init(m_PreOdomPst);

                 odom_trans = transOdom.GetLocalPosture(scan.odom_data.global_pst);
                    // [-PI, PI]
                 odom_trans.fThita = CAngle::NormAngle2(odom_trans.fThita);
				 // dq VISION
                 transform_odom.push_back(odom_trans);

                 scan.odom_data.local_pst = odom_trans;

                 m_PreOdomPst = scan.odom_data.global_pst;

                //std::cout<<"odom  local "<<slam->nodeId<<" "<<scan.odom_data.local_pst.x<<" "<<scan.odom_data.local_pst.y<< " "<<scan.odom_data.local_pst.fThita<<std::endl;
                //std::cout<<"odom global "<<slam->nodeId<<" "<<scan.odom_data.global_pst.x<<" "<<scan.odom_data.global_pst.y<< " "<<scan.odom_data.global_pst.fThita<<std::endl;

                pRawMap->AddRawScan(scan);

                m_rawScans[slam->nodeId] = scan;
                m_dequeRawScan.pop_front();


                break;
            }
        }
        m_dequeRawScan.pop_front();
    }

    std::cout<<"slam->nodeId:  "<<slam->nodeId<<std::endl;




    //发送pad显示
     //   SendMap ( slam, opt , m_RecordPose);

    std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< START SEND SUBMAPSSS!!!! >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;

    submap_data pdata;
    SubmapList submaplist;
    vector <double> blackcell_index;
    m_pCartoSlam->GetSubmapList(submaplist);
    common::Time stamp = submaplist.header.stamp;
    int size = submaplist.submap.size();
    int submap_index = 0;
    bool sendopt = false;

    if(size > 2)
        submap_index =size-2;
    int submap_version = submaplist.submap.at(submap_index).submap_version;

    //std::cout<<"@@@ SubMap Size: "<<size<<std::endl;
    //std::cout<<"@@@ SubMap Index: "<<submap_index<<std::endl;
    //std::cout<<"@@@ Submap Version: "<<submap_version<<std::endl;
    //std::cout<<"@@@ Submap Version Record: "<<submap_version_record<<std::endl;

    if(m_mode == Mode_ExpandMap && m_pCartoSlam->GetFrozenSubmapNum()==(size-1) && size>=1)
    {
        submap_index = size-1;
        submap_index_record = submap_index_record;
        submap_version = submaplist.submap.at(submap_index).submap_version;
    }



    std::cout<<"@@@@@@@ SubMap Index: "<<submap_index<<std::endl;
    std::cout<<"@@@@@@@ submap_index_record: "<<submap_index_record<<std::endl;
    std::cout<<"@@@@@@@ Submap Version: "<<submap_version<<std::endl;
    std::cout<<"@@@@@@@ Submap Version Record: "<<submap_version_record<<std::endl;



   /* static int pppp = 1;
    if(pppp == 1)
    {
        std::cout<<"submap_index == submap_index_record &&submap_version-submap_version_record >= 1"<<std::endl;
        transform::Rigid3d global_pose = submaplist.submap.at(0).pose;
        m_pCartoSlam->GetSubmapData_upload(0, global_pose, &pdata, blackcell_index);
        LCMTask::GetLcmInstance().SendSubMaps(0, pdata, blackcell_index);
        SendMap ( slam, opt , m_RecordPose);
        submap_version_record = submap_version;
        submap_index_record =submap_index;
        stamp_record = stamp;
        pppp =2;
    }
    else*/



    if(submap_index == submap_index_record && submap_version-submap_version_record >= 1)
    {
        std::cout<<"submap_index == submap_index_record &&submap_version-submap_version_record >= 1"<<std::endl;
        transform::Rigid3d global_pose = submaplist.submap.at(submap_index).pose;
        m_pCartoSlam->GetSubmapData_upload(submap_index, global_pose, &pdata, blackcell_index);
        LCMTask::GetLcmInstance().SendSubMaps(submap_index, pdata, blackcell_index,false);
       // SendMap ( slam, opt , m_RecordPose);
        submap_version_record = submap_version;
        submap_index_record =submap_index;
        stamp_record = stamp;
    }
    else if(submap_index > submap_index_record)
    {
        std::cout<<"submap_index > submap_index_record"<<std::endl;
        int temversion = submaplist.submap.at(submap_index_record).submap_version;
        std::cout<<"submap_index_record: "<<submap_index_record<<", temversion: "<<temversion<<std::endl;
        transform::Rigid3d global_pose = submaplist.submap.at(submap_index_record).pose;
        m_pCartoSlam->GetSubmapData_upload(submap_index_record, global_pose, &pdata, blackcell_index);
        LCMTask::GetLcmInstance().SendSubMaps(submap_index_record, pdata, blackcell_index,false);
      //  SendMap ( slam, opt , m_RecordPose);
        submap_version_record = 0;
        submap_index_record =submap_index;
        stamp_record = stamp;
    }
    else if(stamp - stamp_record > 100)
    {
        std::cout<<"Time OUT!!! stamp - stamp_record = "<< stamp - stamp_record <<std::endl;
        transform::Rigid3d global_pose = submaplist.submap.at(submap_index).pose;
        m_pCartoSlam->GetSubmapData_upload(submap_index, global_pose, &pdata, blackcell_index);
        LCMTask::GetLcmInstance().SendSubMaps(submap_index, pdata, blackcell_index,false );

        submap_version_record = submap_version;
        submap_index_record =submap_index;
        stamp_record = stamp;
    }


    //std::cout << "laser tf "<<lasertf.x<<" "<<lasertf.y<<" "<<lasertf.theta<<std::endl;
    SendMap ( slam, opt , m_RecordPose,m_pCartoSlam->GetFrozenNodeNum());
    vector <Pose> optRes;
    optRes.resize(size);
    for(int i = 0; i < size; i++)
    {
        transform::Rigid3d global_pose = submaplist.submap.at(i).pose;
        double theta = transform::GetYaw(global_pose.rotation());
        Eigen::Matrix<double, 3, 1> xyz = global_pose.translation();
        optRes[i].x = xyz(0);
        optRes[i].y = xyz(1);
        optRes[i].theta = theta;
        submap_pose.push_back(xyz(0));
        submap_pose.push_back(xyz(1));
        submap_pose.push_back(theta);
        //std::cout<<"### SubMap: "<<i<< " Pose: "<<xyz(0)<<", "<<xyz(1)<<", "<<theta<<std::endl;
    }

    if(submap_pose != submap_pose_record)
    {
        sendopt = true;
    }

    std::cout<<"submap_pose.size(): "<<submap_pose.size()<<", submap_pose_record.size(): "<<submap_pose_record.size()<<", sendopt: "<<sendopt<<std::endl;

    if(sendopt && submap_version_record >= 1)
    {
        std::cout<<" ------------------------------------------------ SEND OPT TRUE!!! ---------------------------------------------------------"<<std::endl;
        if(size-2>0)
            LCMTask::GetLcmInstance().SendSubMapsPoses(size-2, optRes);
        submap_pose_record.assign(submap_pose.begin(), submap_pose.end());
    }
    submap_pose.clear();
    blackcell_index.clear();
}

//
//by lishen 创建地图结束回调函数
//
void CLaserAutoMapping::BuildMapOverCallback()
{
    // send  to pad
    pLaserAutoMappingObj->SetMappingStatus(Mapping_MAP_WAITING);   // 2：结束建图, 优化完成, 询问pad是否保存地图

}

//
// by lishen 建图结束后 reset
//
void *ResetCartoSlam(void *arg)
{
    CLaserAutoMapping *p = (CLaserAutoMapping *)arg;
    if (p== NULL)
        return NULL;

    std::cout<<"ResetCartoSlam"<<std::endl;
    p->SlamMappingReset();

    return NULL;
}

//
// by lishen建图结束后 reset
//
void CLaserAutoMapping::SlamMappingReset(void)
{
    std::cout<<"SlamMappingReset"<<std::endl;
    std::lock_guard<std::mutex> lock(build_mtx);
    usleep(1000);

     std::cout<<"SlamMappingReset 1"<<std::endl;

    m_pCartoSlam->Reset();

    m_status = Mapping_STAND_BY;

     std::cout<<"SlamMappingReset 2"<<std::endl;
    //切换到定位模式
#ifdef NAV_APP
    auto pLocalize = LocalizeFactorySingleton::GetInstance();
    auto pRoboMan = RoboManagerSingleton::GetInstance();
    pLocalize->LoadMap();

    if(m_mode == Mode_ExpandMap && m_bWaitingExpand == true)
    {
          pLocalize->SetPose(m_expandInitPos.x, m_expandInitPos.y, m_expandInitPos.theta, GetTickCount());
    }
    else
    {
        if (m_SetPose)
            pLocalize->SetPose(m_RecordPose.x, m_RecordPose.y, m_RecordPose.theta, GetTickCount());
    }

    pRoboMan->HandlePadMsg( RLP_MODE_LOCALIZATION );

#endif
}
//
//by lishen 取消保存地图, 不保存地图了  但保存优化后的数据集，资源回收，内存释放等
//
bool CLaserAutoMapping::CancelSaveMap()
{
    std::lock_guard<std::mutex> lock(build_mtx);

    std::cout << "enter CancelSaveMap()" << std::endl;

    if ( m_pCartoSlam == nullptr )
        return false;


    if(m_status != Mapping_MAP_WAITING)
        return false;

    m_status = Mapping_MAP_CANCELING;

   // m_stepDatas.clear();
    map<int,CStepData>().swap(m_stepDatas);
    malloc_trim(0);

    std::vector<node_data>  nodeDatas;

    m_pCartoSlam->GetNodeData(nodeDatas);


    if ( nodeDatas.size() > 0 )
    {
        map<int, sensor::CRawScan>::iterator iter_rawscan;
        map<int, CStepData>::iterator iter_step;

        auto pRawMap = RawMapSingleton::GetInstance();

        std::vector<sensor::CRawScan> opt_scans;
        std::deque<sensor::CRawScan> pRawScans;


        CPosture preOdomPst;

        if(pRawMap->GetRawScans(pRawScans))
        {

            for ( int i=0; i < nodeDatas.size(); i++)
            {

                node_data   node    = nodeDatas.at(i);
                timeval   nodestamp = node.stamp;

                if(pRawScans.size()>i)
                {

                    sensor::CRawScan rawscan = pRawScans.at(i);

                    long int sec  = (long int)(rawscan.point_cloud[0]->timestamp_raw / 1000.0);  // ms   to   sec
                    long int usec = (rawscan.point_cloud[0]->timestamp_raw % 1000) * 1000;


                    if((long int)nodestamp.tv_sec == sec && (long int)nodestamp.tv_usec == usec)
                    {

                        CScan scan;
                        CPosture pstRobot{node.pos.x,node.pos.y,node.pos.theta};
                        CLaserScannerParam Param = m_scannerParam.at(0);

                        /////////////???????????

                        //   save opt dx               //?????????? if there are two lasers
                         if ( i == 0 ) {  //first scan
                             preOdomPst = pstRobot;

                         }
                         CTransform transOdom;
                         CPosture odom_trans ;
                         transOdom.Init(preOdomPst);

                         odom_trans = transOdom.GetLocalPosture(pstRobot);
                            // [-PI, PI]
                         odom_trans.fThita = CAngle::NormAngle2(odom_trans.fThita);
                         rawscan.odom_data.local_pst = odom_trans;

                         preOdomPst = pstRobot;

                         rawscan.odom_data.global_pst = pstRobot;
                         opt_scans.push_back(rawscan);
                    }
                }

            }
        }
        pRawMap->Clear();
        m_nStartTime = static_cast<unsigned int>(GetTickCount() - 200);
        // get the mapping dataset size   ///???????
        int dataset_size = RAW_MAP_CAPACITY_MAPPING;
        auto pParameterObject = ParameterObjectSingleton::GetInstance();
        pParameterObject->GetParameterValue("Mapping_MappingDataSetSize", dataset_size);
        pRawMap->SetMaxCount(dataset_size);
        pRawMap->SetStartTime(m_nStartTime);

        for (unsigned int i = 0; i < opt_scans.size(); i++) {

            pRawMap->AddRawScan(opt_scans[i]);
        }

        SaveDxFile(WORK_PATH"CancelReflectorPoints.dx");

        pRawMap->Clear();
    }





  // m_rawScans.clear();
   m_dequeRawScan.clear();

   map<int,sensor::CRawScan>().swap(m_rawScans);
   malloc_trim(0);

    // reset carto
    pthread_t tid;
    m_SetPose = false;
    pthread_create(&tid, NULL, ResetCartoSlam, this);


    std::cout<<"cancel save over"<<std::endl;

   // 结束建图, 第一帧创建地图变量复位 add 2022.06.09
   m_bFirstScan = true;
   submap_version_record = 0;
   submap_index_record = 0;
   stamp_record = 0;

   transform_odom.clear();
   submap_pose_record.clear();

   if(m_bCameraFlag)
   {
       img_record.clear();
       tp.Stop();
       // 发送消息通知进程启动定位模式
       auto pRoboClnt = RoboClntSingleton::GetInstance();
       pRoboClnt->SetLocMode_Cam();
       usleep(2000*1000);

   }
   /*while(1)
   {
      std::cout<<"pRoboClnt->GetCamMode(): "<<pRoboClnt->GetCamMode()<<std::endl;
      if(pRoboClnt->GetCamMode() == 1)
          break;
      usleep(200000);
   }*/
}
//

//
// by lishen 保存地图之前旋转
//
bool CLaserAutoMapping::RotateMap(double angle)
{
    if(m_pCartoSlam!=nullptr)
    {
        Eigen::AngleAxisd yaw_angle(angle, Eigen::Vector3d::UnitZ());
        const transform::Rigid3d    rot = transform::Rigid3d::Rotation(yaw_angle);
        Eigen::Matrix<double, 3, 1> tr(m_RecordPose.x, m_RecordPose.y ,0.0);
        Eigen::Quaterniond qd = transform::RollPitchYaw(0, 0,m_RecordPose.theta) ;
        transform::Rigid3d pos(tr, qd);
        const transform::Rigid3d afterrot = rot*pos;
        const transform::Rigid2d ddpos = transform::Project2D(afterrot);

        std::cout<<"before "<<m_RecordPose.x<<" "<<m_RecordPose.y<<" "<<m_RecordPose.theta<<std::endl;

        m_RecordPose.x = ddpos.translation().x();
        m_RecordPose.y = ddpos.translation().y();
        m_RecordPose.theta = ddpos.rotation().angle();


        std::cout<<"after "<<m_RecordPose.x<<" "<<m_RecordPose.y<<" "<<m_RecordPose.theta<<std::endl;


        return m_pCartoSlam->RotateMap(angle);
    }
    else
        return false;
}
//
// by lishen 根据数据集得到创建地图需要左上角和右下角坐标
//
bool CLaserAutoMapping::GetMapRange(CPosture &ptLeftBottom,CPosture &ptRightTop)
{
    if ( m_stepDatas.size() <= 0 )
        return false;

    double  laserRange = 15;
    for ( int i = 0; i < m_scannerParam.size(); i++ )
    {
        if ( m_scannerParam.at(i).m_fMaxRange > laserRange )
            laserRange = m_scannerParam.at(i).m_fMaxRange;
    }

    laserRange = laserRange;

    map<int, CStepData>::iterator iter = m_stepDatas.begin();

    double xmin = iter->second.m_pstRobot.x;
    double ymin = iter->second.m_pstRobot.y;
    double xmax = iter->second.m_pstRobot.x;
    double ymax = iter->second.m_pstRobot.y;

    for ( iter = m_stepDatas.begin(); iter != m_stepDatas.end(); iter++ )
    {
        CPosture pst = iter->second.m_pstRobot;

        if (pst.x<xmin)	xmin = pst.x;   // x0
        if (pst.x>xmax)	xmax = pst.x;   // x1
        if (pst.y<ymin)	ymin = pst.y;   // y0
        if (pst.y>ymax)	ymax = pst.y;   // y1
    }

    ptLeftBottom.x  = xmin - laserRange;
    ptLeftBottom.y  = ymin - laserRange;
    ptRightTop.x    = xmax + laserRange;
    ptRightTop.y    = ymax + laserRange;

    return true;
}

//
// by lishen  停止创建
//
bool CLaserAutoMapping::StopMapping()
{
    std::cout<<"enter StopMapping"<<std::endl;
    std::lock_guard<std::mutex> lock(build_mtx);

    if(m_status != Mapping_MAP_BUILDING )
    {
        if(m_status != mapping::Mapping_MAP_READPB_FAILED )
        {
            if(m_status != Mapping_MAP_WAITING_EXPAND)
                return false;
        }
    }

    if ( !Stop() ) {
       return false;
    }

    if(m_mode == Mode_ExpandMap && m_bWaitingExpand == true)
    {
        std::cout<<"m_mode == Mode_ExpandMap && m_bWaitingExpand == true"<<std::endl;
        m_pCartoSlam->Reset();

        m_status = Mapping_STAND_BY;


        //切换到定位模式
    #ifdef NAV_APP
        auto pLocalize = LocalizeFactorySingleton::GetInstance();
        auto pRoboMan = RoboManagerSingleton::GetInstance();
        pLocalize->LoadMap();

        if(m_mode == Mode_ExpandMap && m_bWaitingExpand == true)
        {
              pLocalize->SetPose(m_expandInitPos.x, m_expandInitPos.y, m_expandInitPos.theta, GetTickCount());
        }
        else
        {
            if (m_SetPose)
                pLocalize->SetPose(m_RecordPose.x, m_RecordPose.y, m_RecordPose.theta, GetTickCount());
        }

        pRoboMan->HandlePadMsg( RLP_MODE_LOCALIZATION );

    #endif
    }
    else
    {
        m_status = Mapping_MAP_OPTING;

        SaveDxFile(WORK_PATH"ReflectorPoints.dx");

        m_pCartoSlam->StopMapping();
    }

    return true;
}
//
// by lishen  保存dx 写激光参数
//
bool CLaserAutoMapping::WriteLaserParm(const char* filename)
{
    FILE* fp  = NULL;
    bool bRet = false;

    fp = fopen(filename, "w");
    if ( !fp ) {
        return false;
    }
    fclose(fp);

    fp = fopen(filename, "ab");
    if ( !fp ) {
        return false;
    }

    auto pRawMap = RawMapSingleton::GetInstance();
    bRet = pRawMap->WriteLaserParm(fp,false);
    fclose(fp);

    return bRet;
}
//
// by lishen  保存dx 写scan data
//
bool CLaserAutoMapping::WriteScanData(const char* filename)
{
    FILE* fp = NULL;
    bool bRet = false;

    // if(m_aWorkMode.load() == RLP_MODE_MAPPING)   //?????
    {
        fp = fopen(filename, "ab");
        if(!fp) {
            return false;
        }
    }
    // else {
    //     return false;
    // }

    auto pRawMap = RawMapSingleton::GetInstance();
    bRet = pRawMap->WriteScanData(fp);
    fclose(fp);

    return bRet;
}
//
// by lishen  保存dx 写file
//
bool CLaserAutoMapping::SaveDxFile(const char* filename)
{
    // if(m_aFileSaving.load()){
    //     return false;
    // }

    m_aFileSaving = true;
    if ( !WriteLaserParm(filename) ) {
        m_aFileSaving = false;
        return false;
    }

    if ( !WriteScanData(filename) ) {
        m_aFileSaving = false;
        return false;
    }
    m_aFileSaving = false;

    return true;
}

bool CLaserAutoMapping::SavePbFile(const char* filename)
{
    if ( m_pCartoSlam == nullptr )
    {
       return false;
    }

    m_aFileSaving = true;
    FILE* fp  = NULL;
    bool bRet = false;


        fp = fopen(filename, "w");
        if ( !fp ) {
            return false;
        }
        fclose(fp);

        fp = fopen(filename, "ab");
        if ( !fp ) {
            return false;
        }


    auto pRawMap = RawMapSingleton::GetInstance();
    bRet = pRawMap->WriteLaserParm(fp,true);  //save two laser
    if(!bRet){
        fclose(fp);
        return bRet;
    }

    bRet = pRawMap->WriteScanData(fp);
    if(!bRet){
        fclose(fp);
        return bRet;
    }

    bRet = m_pCartoSlam->SaveBinary(fp);

    fclose(fp);

    return bRet;
}

// 
void CLaserAutoMapping::SetMappingMode(MappingMode uMode)
{
    m_mode = uMode;
}
void CLaserAutoMapping::SetInitPos(Pose &pos)
{
   std::cout<<"CLaserAutoMappingm_expandInitPos "<<m_expandInitPos.x<< " "<<m_expandInitPos.y<< " "<<m_expandInitPos.theta<<std::endl;

    m_expandInitPos = pos;

    std::cout<<"CLaserAutoMappingm_expandInitPos "<<m_expandInitPos.x<< " "<<m_expandInitPos.y<< " "<<m_expandInitPos.theta<<std::endl;

}
void CLaserAutoMapping::SetExpandFrozen(bool frozen)
{
    m_bFrozenNode = frozen;
}


bool CLaserAutoMapping::LoadPbFile(char* filename)
{

    const char* pstr;
    if(filename==NULL)
    {
        string str = WORK_PATH"data.pb";
        pstr = str.c_str();
    }
    else
    {
        string str = filename;
        str =  WORK_PATH + str;
        pstr = str.c_str();
    }


    std::cout<<pstr<<std::endl;


    FILE *fp = fopen(WORK_PATH"data.pb", "rb");
    if (fp == NULL)
    {
        std::cout<<"read failed\n";
        return false;

    }
    int nVersion = 210;

    if (fread(&nVersion, sizeof(int), 1, fp) != 1)
        return false;

      std::cout<<"nVersion"<<nVersion<<std::endl;

    unsigned int uTimeStamp;
    if (nVersion >= 210)
    {
        if (fread(&uTimeStamp, sizeof(unsigned int), 1, fp) != 1)
            return false;

        m_nStartTime = uTimeStamp;
    }


      // 激光参数

    m_scannerParamExpandDx.clear();

    if (!m_scannerParamExpandDx.LoadBinary(fp, nVersion))
        return false;

    if(m_scannerParamExpandDx.size()<1)
        return false;

     std::cout<<"m_originscannerParam.size() = "<<m_scannerParamExpandDx.size()<<std::endl;

    vector<int> linecounts;

    for(int i=0; i<m_scannerParamExpandDx.size();i++)
        linecounts.push_back(m_scannerParamExpandDx.at(i).m_nLineCount);


    auto pRawMap = RawMapSingleton::GetInstance();

    // get the mapping dataset size   ///???????
    int dataset_size = RAW_MAP_CAPACITY_MAPPING;
    pRawMap->SetMaxCount(dataset_size);

    if(!pRawMap->ReadScanData(fp,nVersion,m_scannerParamExpandDx.size(),linecounts))
        return false;

    std::deque<sensor::CRawScan> pRawScans;

    pRawMap->GetRawScans(pRawScans);

    std::cout << "read  pRawScans.size() "<<pRawScans.size() << std::endl;


    if ( m_pCartoSlam == nullptr )
    {
         m_pCartoSlam = CCartoSlam::GetInstance();
    }

    std::cout<<"m_pCartoSlam->OpenExpandDx"<<std::endl;

    if(!m_pCartoSlam->OpenExpandDx(fp))
        return false;

    std::cout<<"read pb  over"<<std::endl;

    return true;

}

void CLaserAutoMapping::SendSubMaps(int num, int8_t *pSubmapIds)
{
    SubmapList submaplist;

    m_pCartoSlam->GetSubmapList(submaplist);

    int size = submaplist.submap.size();

    if(num==0)
    {
        for(auto submap : submaplist.submap)
        {
            submap_data pdata;
            vector <double> blackcell_index;

            transform::Rigid3d global_pose = submap.pose;
            int submap_index = submap.submap_index;

            // std::cout<<"submap_index ="<<submap_index<<std::endl;

            m_pCartoSlam->GetSubmapData_upload(submap_index, global_pose, &pdata, blackcell_index);

            std::cout<<"blackcell_index"<<blackcell_index.size()<<std::endl;

            LCMTask::GetLcmInstance().SendSubMaps(submap_index, pdata, blackcell_index,true);

            usleep(200000);

        }
    }
    else
    {
        for(int i=0;i<num;i++)
        {
            submap_data pdata;
            vector <double> blackcell_index;


            int submap_index = pSubmapIds[i];
            transform::Rigid3d global_pose = submaplist.submap[submap_index].pose;

            // std::cout<<"submap_index ="<<submap_index<<std::endl;

            m_pCartoSlam->GetSubmapData_upload(submap_index, global_pose, &pdata, blackcell_index);

            //std::cout<<"blackcell_index"<<blackcell_index.size()<<std::endl;

            LCMTask::GetLcmInstance().SendSubMaps(submap_index, pdata, blackcell_index,true);

            usleep(200000);
        }
    }
}

//
//by lishen 保存地图
//
#ifdef ONE_LASER
bool CLaserAutoMapping::SaveMap()
{
    std::cout << "enter SaveMap()" << std::endl;
    std::lock_guard<std::mutex> lock(build_mtx);

    if(m_pCartoSlam == nullptr)
        return false;

    if(m_status != Mapping_MAP_WAITING)
        return false;

    m_status = Mapping_MAP_SAVING;

    std::vector<node_data>  nodeDatas;
    int frozenNodeNum = 0;

    m_pCartoSlam->GetNodeData(nodeDatas);

    std::cout << "SaveMap()   nodeDatas "<<nodeDatas.size() << std::endl;

    if(m_mode == Mode_ExpandMap && m_bFrozenNode)
    {
        frozenNodeNum = m_pCartoSlam->GetFrozenNodeNum();

    }
    std::cout << "frozenNodeNum "<<frozenNodeNum << std::endl;


    if ( nodeDatas.size() > 0 )
    {

        m_stepDatas.clear();


        auto pRawMap = RawMapSingleton::GetInstance();
        std::deque<sensor::CRawScan> pRawScans;

        std::deque<sensor::CRawScan> opt_scans;
        CPosture preOdomPst;

        std::cout << "pRawScans.size() "<<pRawScans.size() << std::endl;
        // dq VISION

        ofstream os;
        os.open(WORK_PATH"ImgRecord/LaserMsg.json", ios::out);
        if(!os.is_open())
            std::cout<<"Error: can not find create the file which named \"LaserMsg.json\"."<<std::endl;
        //outfile1.open(WORK_PATH"ImgRecord/Lasstamp_record.txt", ios::out);
        if(pRawMap->GetRawScans(pRawScans))
        {

            //std::cout << "SaveMap()   pRawScans.size() "<<pRawScans.size() << std::endl;

            for ( int i=0; i < nodeDatas.size(); i++)
            {

                node_data   node    = nodeDatas.at(i);
                timeval   nodestamp = node.stamp;

                if(pRawScans.size()>i)
                {
                    //sensor::CRawScan rawscan = pRawScans.at(i);   //;;;;;;;

                   // long int sec  = (long int)(rawscan.point_cloud[0]->timestamp_raw / 1000.0);  // ms   to   sec
                   // long int usec = (rawscan.point_cloud[0]->timestamp_raw % 1000) * 1000;

                     long int sec  = (long int)(pRawScans.at(i).point_cloud[0]->timestamp_raw / 1000.0);  // ms   to   sec
                     long int usec = (pRawScans.at(i).point_cloud[0]->timestamp_raw % 1000) * 1000;

                     std::cout<<"node i =  "<<i<<"  "<<nodestamp.tv_sec<<"  "<<nodestamp.tv_usec<< std::endl;
                    // std::cout<<"praw i =  "<<i<<"  "<<sec<<"  "<<usec<< std::endl;


                    if((long int)nodestamp.tv_sec == sec && (long int)nodestamp.tv_usec == usec)
                    {

                        CScan scan;
                        CPosture pstRobot{node.pos.x,node.pos.y,node.pos.theta};
                        CLaserScannerParam Param ;

                        /////////////???????????? expand

                        if(i<frozenNodeNum)
                        {
                            if(m_scannerParamExpandDx.size()>0)
                               Param = m_scannerParamExpandDx.at(0);
                        }
                        else
                            Param = m_scannerParam.at(0);

                        //TransformRawCloudToScan ( pstRobot, Param, rawscan, scan );
                        TransformRawCloudToScan ( pstRobot, Param, pRawScans.at(i), scan);
                        //CStepData step;
                        m_stepDatas[node.node_id].id         = node.node_id;
                        m_stepDatas[node.node_id].m_scan     = scan;
                        m_stepDatas[node.node_id].m_pstRobot = pstRobot;
                        m_stepDatas[node.node_id].m_pstOdom  = pRawScans.at(i).odom_data.global_pst;    //????

                        m_stepDatas[node.node_id].m_pstMoveEst.Stamp ( pRawScans.at(i).odom_data.time_stamp );
                        m_stepDatas[node.node_id].m_pstMoveEst.SetPosture( 0, 0, 0 );       // 机器人的估测姿态变化量

                        m_stepDatas[node.node_id].m_pst.Stamp ( pRawScans.at(i).odom_data.time_stamp );
                        m_stepDatas[node.node_id].m_pst.SetPosture ( pstRobot );            // 由数据集记录的机器人的绝对姿态(仅供参考)
                        //outfile1<<m_stepDatas[node.node_id].id<<": "<<pstRobot.x<<", "<<pstRobot.y<<", "<<pstRobot.fThita<<endl;
                       // outfile1<<m_stepDatas[node.node_id].id<<": "<<pRawScans.at(i).point_cloud[0]->timestamp_raw<<endl;

                        m_stepDatas[node.node_id].m_vel.x      = pRawScans.at(i).odom_data.velocity.fXLinear;       // 速度向量，借用CPosture结构来表示
                        m_stepDatas[node.node_id].m_vel.y      = pRawScans.at(i).odom_data.velocity.fYLinear;
                        m_stepDatas[node.node_id].m_vel.fThita = pRawScans.at(i).odom_data.velocity.fAngular;
                        //m_stepDatas[node.node_id] = step;

                        //dq
                        scan.Clear();

                        //   save opt dx               //?????????? if there are two lasers
                         if ( i == 0 ) {  //first scan
                             preOdomPst = pstRobot;

                         }
                         CTransform transOdom;
                         CPosture odom_trans ;
                         transOdom.Init(preOdomPst);

                         odom_trans = transOdom.GetLocalPosture(pstRobot);
                            // [-PI, PI]
                         odom_trans.fThita = CAngle::NormAngle2(odom_trans.fThita);
                         pRawScans.at(i).odom_data.local_pst = odom_trans;

                         preOdomPst = pstRobot;

                         pRawScans.at(i).odom_data.global_pst = pstRobot;
                         opt_scans.push_back(pRawScans.at(i));


                         if(m_bCameraFlag)
                         {
                             // dq VISION record Json
                             Json::Value root;
                             Json::Value laser;
                             Json::Value img;
                             Json::Value pose;
                             Json::Value odom;
                            for(auto r: pRawScans.at(i).point_cloud[0]->distance)
                            {
                                int dis = float(r);
                                if(dis != 0)
                                    laser["ranges"].append(dis);
                                else
                                    laser["ranges"].append("NAN");
                            }

                            laser["angle_step"] = Json::Value(0.1);
                            uint64_t time = pRawScans.at(i).point_cloud[0]->timestamp_raw;
                            laser["time"] = Json::Value(time);
                            laser["min_angle"] = Json::Value(-180.0);
                            laser["max_angle"] = Json::Value(180.0);
                            img["idx"] = Json::Value(i);
                            img["time"] = Json::Value(time_record[i]);
                            pose["y"] = Json::Value(pstRobot.y);
                            pose["x"] = Json::Value(pstRobot.x);
                            pose["yaw"] = Json::Value(pstRobot.fThita);
                            pose["time"] = Json::Value(time);
                            odom["y"] = Json::Value(transform_odom[i].y);
                            odom["x"] = Json::Value(transform_odom[i].x);
                            odom["yaw"] = Json::Value(transform_odom[i].fThita);
                            odom["time"] = Json::Value(time);
                            root["laser"] = Json::Value(laser);
                            root["img"] = Json::Value(img);
                            root["pose"] = Json::Value(pose);
                            root["odom"] = Json::Value(odom);
                            Json::FastWriter fw;
                            if(os)
                                os << fw.write(root);
                         }
                    }
                }
            }
        }

   if(os)
        os.close();

    std::cout<<"m_stepDatas.size = "<<m_stepDatas.size()<<std::endl;
    std::cout<<"nodeDatas.size() = "<<nodeDatas.size()<<std::endl;


    //保存ndt地图
    CPosture ptLeftBottom;
    CPosture ptRightTop;
    if ( GetMapRange(ptLeftBottom, ptRightTop) )
    {

        double mapSizeX = (max(fabs(ptLeftBottom.x), fabs(ptRightTop.x)));    // 地图X方向长度的一半
        double mapSizeY = (max(fabs(ptLeftBottom.y), fabs(ptRightTop.y)));    // 地图Y方向长度的一半

        int sizeX = mapSizeX/0.2;
        mapSizeX = sizeX*0.2*2;

        int sizeY = mapSizeY/0.2;
        mapSizeY = sizeY*0.2*2;

        ndt_oru::CSubmapParam submapParam{0.2,mapSizeX,mapSizeY};

        CBuildMap buildmap;
        buildmap.Create();

        buildmap.SetScansParam(&m_scannerParam); //???
        buildmap.SetNdtSubmapParam(&submapParam);
        buildmap.BuildNdtMap(&m_stepDatas);

        buildmap.SaveNdtMap();

        std::cout<<"Save  ndt Map over "<<std::endl;
        //保存 grid 地图
        buildmap.SaveProbGridMap(WORK_PATH"ProbMap.txt",&m_stepDatas,ptLeftBottom, ptRightTop,frozenNodeNum);

        std::cout<<"Save  grid Map over "<<std::endl;

    }
    // dq malloc 释放内存到系统
    //m_stepDatas.clear();
    map<int,CStepData>().swap(m_stepDatas);
    malloc_trim(0);

    //保存优化后的数据集
   pRawMap->Clear();
   m_nStartTime = static_cast<unsigned int>(GetTickCount() - 200);
   // get the mapping dataset size   ///???????
   int dataset_size = RAW_MAP_CAPACITY_MAPPING;
   auto pParameterObject = ParameterObjectSingleton::GetInstance();
   pParameterObject->GetParameterValue("Mapping_MappingDataSetSize", dataset_size);
   pRawMap->SetMaxCount(dataset_size);
   pRawMap->SetStartTime(m_nStartTime);

   for (unsigned int i = 0; i < opt_scans.size(); i++) {

       pRawMap->AddRawScan(opt_scans[i]);
   }

   SaveDxFile(WORK_PATH"OptReflectorPoints.dx");
   SavePbFile(WORK_PATH"data.pb");

   pRawMap->Clear();
   opt_scans.clear();
   pRawScans.clear();

    //////////////////////////////////
    system("cp ProbMap.txt ProbMap_bak.txt");
    system("cp Gridmap.map Gridmap_bak.map");


    }
    //dq VISION save IMG
    if(m_bCameraFlag)
    {
        for(int q = m_pCartoSlam->GetFrozenNodeNum(); q < img_record.size(); q++)
        {
            imgname = WORK_PATH"ImgRecord/"+to_string(q) + ".jpg";
            cv::imwrite(imgname,img_record[q]);
        }
        img_record.clear();
        time_record.clear();
    }

   // m_rawScans.clear();
    map<int,sensor::CRawScan>().swap(m_rawScans);
    malloc_trim(0);
    m_dequeRawScan.clear();

    // reset carto
    pthread_t tid;
    m_SetPose = true;
    pthread_create(&tid, NULL, ResetCartoSlam, this);


   // 结束建图, 第一帧创建地图变量复位 add 2022.06.09
   m_bFirstScan = true;
   submap_version_record = 0;
   submap_index_record = 0;
   stamp_record = 0;
   submap_pose_record.clear();
   transform_odom.clear();



   tp.Stop();
   // 发送消息通知进程启动定位模式
   auto pRoboClnt = RoboClntSingleton::GetInstance();
   pRoboClnt->SetLocMode_Cam();
   usleep(2000*1000);
}
#else
bool CLaserAutoMapping::SaveMap()
{
    std::cout << "enter SaveMap()" << std::endl;
    std::lock_guard<std::mutex> lock(build_mtx);

    if(m_pCartoSlam == nullptr)
        return false;

    if(m_status != Mapping_MAP_WAITING)
        return false;

    m_status = Mapping_MAP_SAVING;

    std::vector<node_data>  nodeDatas;
    int frozenNodeNum = 0;

    m_pCartoSlam->GetNodeData(nodeDatas);

    std::cout << "SaveMap()   nodeDatas "<<nodeDatas.size() << std::endl;

    if(m_mode == Mode_ExpandMap && m_bFrozenNode)
    {
        frozenNodeNum = m_pCartoSlam->GetFrozenNodeNum();

    }
    std::cout << "frozenNodeNum "<<frozenNodeNum << std::endl;


    if ( nodeDatas.size() > 0 )
    {

        m_stepDatas.clear();


        auto pRawMap = RawMapSingleton::GetInstance();
        std::deque<sensor::CRawScan> pRawScans;

        std::deque<sensor::CRawScan> opt_scans;
        CPosture preOdomPst;

        std::cout << "pRawScans.size() "<<pRawScans.size() << std::endl;
        // dq VISION

        ofstream os;
        os.open(WORK_PATH"ImgRecord/LaserMsg.json", ios::out);
        if(!os.is_open())
            std::cout<<"Error: can not find create the file which named \"LaserMsg.json\"."<<std::endl;
        //outfile1.open(WORK_PATH"ImgRecord/Lasstamp_record.txt", ios::out);
        if(pRawMap->GetRawScans(pRawScans))
        {

            //std::cout << "SaveMap()   pRawScans.size() "<<pRawScans.size() << std::endl;

            for ( int i=0; i < nodeDatas.size(); i++)
            {

                node_data   node    = nodeDatas.at(i);
                timeval   nodestamp = node.stamp;

                if(pRawScans.size()>i)
                {

                     long int sec  = (long int)(pRawScans.at(i).point_cloud[0]->timestamp_raw / 1000.0);  // ms   to   sec
                     long int usec = (pRawScans.at(i).point_cloud[0]->timestamp_raw % 1000) * 1000;

                     std::cout<<"node i =  "<<i<<"  "<<nodestamp.tv_sec<<"  "<<nodestamp.tv_usec<< std::endl;
                    // std::cout<<"praw i =  "<<i<<"  "<<sec<<"  "<<usec<< std::endl;


                    if((long int)nodestamp.tv_sec == sec && (long int)nodestamp.tv_usec == usec)
                    {

                        vector<CScan> scans;
                        CScan scan;
                        CPosture pstRobot{node.pos.x,node.pos.y,node.pos.theta};
                        CLaserScannerParam Param ;

                        /////////////???????????? expand

                        if(i<frozenNodeNum)
                        {
                            TransformRawCloudToScan ( pstRobot, m_scannerParamExpandDx, pRawScans.at(i),  m_stepDatas[node.node_id].m_scans);
                        }
                        else
                              TransformRawCloudToScan ( pstRobot, m_scannerParam, pRawScans.at(i),  m_stepDatas[node.node_id].m_scans);


                        //TransformRawCloudToScan ( pstRobot, Param, rawscan, scan );
                       // TransformRawCloudToScan ( pstRobot, Param, pRawScans.at(i), scan);

                        m_stepDatas[node.node_id].id         = node.node_id;

                        m_stepDatas[node.node_id].m_pstRobot = pstRobot;
                        m_stepDatas[node.node_id].m_pstOdom  = pRawScans.at(i).odom_data.global_pst;    //????

                        m_stepDatas[node.node_id].m_pstMoveEst.Stamp ( pRawScans.at(i).odom_data.time_stamp );
                        m_stepDatas[node.node_id].m_pstMoveEst.SetPosture( 0, 0, 0 );       // 机器人的估测姿态变化量

                        m_stepDatas[node.node_id].m_pst.Stamp ( pRawScans.at(i).odom_data.time_stamp );
                        m_stepDatas[node.node_id].m_pst.SetPosture ( pstRobot );            // 由数据集记录的机器人的绝对姿态(仅供参考)
                        //outfile1<<m_stepDatas[node.node_id].id<<": "<<pstRobot.x<<", "<<pstRobot.y<<", "<<pstRobot.fThita<<endl;
                       // outfile1<<m_stepDatas[node.node_id].id<<": "<<pRawScans.at(i).point_cloud[0]->timestamp_raw<<endl;

                        m_stepDatas[node.node_id].m_vel.x      = pRawScans.at(i).odom_data.velocity.fXLinear;       // 速度向量，借用CPosture结构来表示
                        m_stepDatas[node.node_id].m_vel.y      = pRawScans.at(i).odom_data.velocity.fYLinear;
                        m_stepDatas[node.node_id].m_vel.fThita = pRawScans.at(i).odom_data.velocity.fAngular;
                        //m_stepDatas[node.node_id] = step;

                        //dq
                        for(auto scan:scans)
                        {
                            scan.Clear();
                        }
                        scans.clear();

                        //   save opt dx               //?????????? if there are two lasers
                         if ( i == 0 ) {  //first scan
                             preOdomPst = pstRobot;

                         }
                         CTransform transOdom;
                         CPosture odom_trans ;
                         transOdom.Init(preOdomPst);

                         odom_trans = transOdom.GetLocalPosture(pstRobot);
                            // [-PI, PI]
                         odom_trans.fThita = CAngle::NormAngle2(odom_trans.fThita);
                         pRawScans.at(i).odom_data.local_pst = odom_trans;

                         preOdomPst = pstRobot;

                         pRawScans.at(i).odom_data.global_pst = pstRobot;
                         opt_scans.push_back(pRawScans.at(i));


                         if(m_bCameraFlag)
                         {
                             // dq VISION record Json
                             Json::Value root;
                             Json::Value laser;
                             Json::Value img;
                             Json::Value pose;
                             Json::Value odom;
                            for(auto r: pRawScans.at(i).point_cloud[0]->distance)
                            {
                                int dis = float(r);
                                if(dis != 0)
                                    laser["ranges"].append(dis);
                                else
                                    laser["ranges"].append("NAN");
                            }



                            laser["angle_step"] = m_scannerParam.at(0).m_fReso*180.0/PI;
                            //laser["angle_step"] = Json::Value(0.1);
                            uint64_t time = pRawScans.at(i).point_cloud[0]->timestamp_raw;
                            laser["time"] = Json::Value(time);
                           // laser["min_angle"] = Json::Value(-180.0);
                           // laser["max_angle"] = Json::Value(180.0);

                            laser["min_angle"] = m_scannerParam.at(0).m_fStartAngle*180.0/PI;
                            laser["max_angle"] =  m_scannerParam.at(0).m_fEndAngle*180.0/PI;

                            img["idx"] = Json::Value(i);
                            img["time"] = Json::Value(time_record[i]);
                            pose["y"] = Json::Value(pstRobot.y);
                            pose["x"] = Json::Value(pstRobot.x);
                            pose["yaw"] = Json::Value(pstRobot.fThita);
                            pose["time"] = Json::Value(time);
                            odom["y"] = Json::Value(transform_odom[i].y);
                            odom["x"] = Json::Value(transform_odom[i].x);
                            odom["yaw"] = Json::Value(transform_odom[i].fThita);
                            odom["time"] = Json::Value(time);
                            root["laser"] = Json::Value(laser);
                            root["img"] = Json::Value(img);
                            root["pose"] = Json::Value(pose);
                            root["odom"] = Json::Value(odom);
                            Json::FastWriter fw;
                            if(os)
                                os << fw.write(root);
                         }
                    }
                }
            }
        }

   if(os)
        os.close();

    std::cout<<"m_stepDatas.size = "<<m_stepDatas.size()<<std::endl;
    std::cout<<"nodeDatas.size() = "<<nodeDatas.size()<<std::endl;


    //保存ndt地图
    CPosture ptLeftBottom;
    CPosture ptRightTop;
    if ( GetMapRange(ptLeftBottom, ptRightTop) )
    {

        double mapSizeX = (max(fabs(ptLeftBottom.x), fabs(ptRightTop.x)));    // 地图X方向长度的一半
        double mapSizeY = (max(fabs(ptLeftBottom.y), fabs(ptRightTop.y)));    // 地图Y方向长度的一半

        int sizeX = mapSizeX/0.2;
        mapSizeX = sizeX*0.2*2;

        int sizeY = mapSizeY/0.2;
        mapSizeY = sizeY*0.2*2;

        ndt_oru::CSubmapParam submapParam{0.2,mapSizeX,mapSizeY};

        CBuildMap buildmap;
        buildmap.Create();

        buildmap.SetScansParam(&m_scannerParam); //???
        buildmap.SetNdtSubmapParam(&submapParam);
        buildmap.BuildNdtMap(&m_stepDatas);

        buildmap.SaveNdtMap();

        std::cout<<"Save  ndt Map over "<<std::endl;
        //保存 grid 地图
        buildmap.SaveProbGridMap(WORK_PATH"ProbMap.txt",&m_stepDatas,ptLeftBottom, ptRightTop,frozenNodeNum);

        std::cout<<"Save  grid Map over "<<std::endl;

    }
    // dq malloc 释放内存到系统
    //m_stepDatas.clear();
    map<int,CStepData>().swap(m_stepDatas);
    malloc_trim(0);

    //保存优化后的数据集
   pRawMap->Clear();
   m_nStartTime = static_cast<unsigned int>(GetTickCount() - 200);
   // get the mapping dataset size   ///???????
   int dataset_size = RAW_MAP_CAPACITY_MAPPING;
   auto pParameterObject = ParameterObjectSingleton::GetInstance();
   pParameterObject->GetParameterValue("Mapping_MappingDataSetSize", dataset_size);
   pRawMap->SetMaxCount(dataset_size);
   pRawMap->SetStartTime(m_nStartTime);

   for (unsigned int i = 0; i < opt_scans.size(); i++) {

       pRawMap->AddRawScan(opt_scans[i]);
   }

   SaveDxFile(WORK_PATH"OptReflectorPoints.dx");
   SavePbFile(WORK_PATH"data.pb");

   pRawMap->Clear();
   opt_scans.clear();
   pRawScans.clear();

    //////////////////////////////////
    system("cp ProbMap.txt ProbMap_bak.txt");
    system("cp Gridmap.map Gridmap_bak.map");


    }
    //dq VISION save IMG
    if(m_bCameraFlag)
    {
        for(int q = m_pCartoSlam->GetFrozenNodeNum(); q < img_record.size(); q++)
        {
            imgname = WORK_PATH"ImgRecord/"+to_string(q) + ".jpg";
            cv::imwrite(imgname,img_record[q]);
        }
        img_record.clear();
        time_record.clear();
    }

   // m_rawScans.clear();
    map<int,sensor::CRawScan>().swap(m_rawScans);
    malloc_trim(0);
    m_dequeRawScan.clear();

    // reset carto
    pthread_t tid;
    m_SetPose = true;
    pthread_create(&tid, NULL, ResetCartoSlam, this);


   // 结束建图, 第一帧创建地图变量复位 add 2022.06.09
   m_bFirstScan = true;
   submap_version_record = 0;
   submap_index_record = 0;
   stamp_record = 0;
   submap_pose_record.clear();
   transform_odom.clear();


   if(m_bCameraFlag)
   {
       tp.Stop();
       // 发送消息通知进程启动定位模式
       auto pRoboClnt = RoboClntSingleton::GetInstance();
       pRoboClnt->SetLocMode_Cam();
       usleep(2000*1000);
   }
}

#endif

} // namespace mapping

