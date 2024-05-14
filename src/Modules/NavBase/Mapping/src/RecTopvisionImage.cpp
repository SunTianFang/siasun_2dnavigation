#include <stdafx.h>
#include "Project.h"

#include "RecTopvisionImage.h"

#include "blackboxhelper.hpp"
#include "RoboLocProto.h"
#include "RoboLocClnt.h"

#include "BaseOdometry.h"

#include "LCMTask.h"

#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

namespace mapping {


/*
void* RecordImageSupportProc(LPVOID pParam)
{
    int time_diff  = 0;     // 每次循环执行时间
    int ctrl_cycle = 0;     // 每次循环休眠时间

    mapping::CRecTopvisionImage* pRecord = reinterpret_cast<mapping::CRecTopvisionImage*>(pParam);


    while (WaitForSingleObject(pRecord->m_hKillThread, 0) != WAIT_OBJECT_0)
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

    SetEvent ( pRecord->m_hThreadDead );
    pthread_exit(NULL);

    return NULL;
}

*/



CRecTopvisionImage::CRecTopvisionImage()
{
    m_bFirstTime = true;
    m_bLaserParm = false;
    m_bCameraInit = false;

    m_hThreadDead    = NULL;
    m_hKillThread = NULL;

}

int CRecTopvisionImage::StartRecord()
{

    std::lock_guard<std::mutex> lock(m_mtxRecord);


    if(m_bCameraInit)
        return RECORDIMG_START_SUCCESS;

    Reset();

    // 发送消息通知进程暂停定位模式
    auto pRoboClnt = RoboClntSingleton::GetInstance();
    pRoboClnt->SetMappingMode_Cam();
    usleep(2000*1000);

    m_bCameraInit = tp.Init();
    if (m_bCameraInit)
    {

        cout << "&&&&&&&&&&&&&&&&   Open camera success!!!" << endl;
       #ifdef USE_BLACK_BOX
           FILE_BlackBox(LocBox, "Open camera success!!!");
       #endif

    }
    else
    {
        cout << "&&&&&&&&&&&&&&&&    Open camera failed!!!" << endl;
       #ifdef USE_BLACK_BOX
           FILE_BlackBox(LocBox, "Open camera failed!!!");
       #endif

        return RECORDIMG_INIT_FALURE;
    }

    auto pOdometry = BaseOdomSingleton::GetInstance();
    pOdometry->SetAccumuOdom(0);    // 单位为毫米


    auto pAFamily = SensorFamilySingleton::GetInstance();
    sensor::CSensorData* sensor_data = NULL;

   // m_scannerParam.clear();
    for ( unsigned int m = 0; m < pAFamily->GetCount(); m++ )
    {
        sensor_data = pAFamily->GetSensorData(m);
        if ( sensor_data && sensor_data->parm  && (pAFamily->GetState(m)))
        {
            //m_scannerParam.push_back(*(sensor_data->parm));

            if(m==0)
            {
                angle_step = sensor_data->parm->m_fReso*180.0/PI;
                min_angle = sensor_data->parm->m_fStartAngle*180.0/PI;
                max_angle=  sensor_data->parm->m_fEndAngle*180.0/PI;
                m_bLaserParm = true;
            }
        }
    }
    if(!m_bLaserParm)
        return RECORDIMG_LASER_ERROR;


 //   m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
  //  if (m_hKillThread == NULL)
  //      return false;

  //  m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
  //  if (m_hThreadDead == NULL)
   //     return false;


/*    pthread_attr_t attr;
    SetPthreadPriority(SCHED_RR ,THREAD_PRIORITY_NORMAL, attr);
    if(pthread_create(&m_pRecordThread, &attr, RecordImageSupportProc, reinterpret_cast<LPVOID>(this)) != 0)
    {
        std::cout<<"Creat RecordImageSupportProc Pthread Failed"<<std::endl;
        return false;
    }
    else
    {
        std::cout<<"Creat RecordImageSupportProc Pthread OK"<<std::endl;
    }

    pthread_attr_destroy(&attr);*/



    return RECORDIMG_START_SUCCESS;
}

bool CRecTopvisionImage::RecordImage(CPosture &robotPos)
{

    if(m_bCameraInit)
    {
            //time_record.push_back(GetTickCount());
            cv::Mat img;
            tp.GetFrame(img);
            img_record.push_back(img);
    }

}
bool CRecTopvisionImage::RecordLaserAndPos(const sensor::CRawScan& rawScan,CPosture &robotPose)
{
     std::lock_guard<std::mutex> lock(m_mtxRecord);

    if(!m_bCameraInit)
        return false;
    if(!m_bLaserParm)
        return false;

   // auto                pOdometry   = BaseOdomSingleton::GetInstance();
   // double              acc_odom    = static_cast<double>(pOdometry->GetAccumuOdom()) / 100000.0;

    // dq VISION 0.2m 采集


    std::cout<<robotPose.x <<" "<< robotPose.y<<" "<<m_PreRobotPst.x <<" "<<m_PreRobotPst.y<<std::endl;



    float acc_dis = robotPose.DistanceTo(m_PreRobotPst);

        std::cout<<" %%%% RecordLaserAndPos %%%"  << acc_dis<<  std::endl;


    if(!m_bFirstTime && acc_dis < 0.2) {
      return false;
    }
    if(m_bFirstTime)
    {
        m_bFirstTime = false;

    }

    m_PreRobotPst = robotPose;

    Json::Value laser;
    Json::Value img;
    Json::Value pose;
    Json::Value odom;


    for(auto r: rawScan.point_cloud[0]->distance)
    {
       int dis = float(r);
       if(dis != 0)
           laser["ranges"].append(dis);
       else
           laser["ranges"].append("NAN");
    }



   uint64_t time = rawScan.point_cloud[0]->timestamp_raw;

   laser["time"] = Json::Value(time);
   laser["angle_step"] = angle_step;
   laser["min_angle"] = min_angle;
   laser["max_angle"] = max_angle;


   pose["y"] = Json::Value(robotPose.y);
   pose["x"] = Json::Value(robotPose.x);
   pose["yaw"] = Json::Value(robotPose.fThita);
   pose["time"] = Json::Value(time);


   if ( m_vtOdoms.size() <= 0 ) {  //first scan
       m_PreOdomPst = rawScan.odom_data.global_pst;

   }
   CTransform transOdom;
   CPosture odom_trans ;
   transOdom.Init(m_PreOdomPst);

   odom_trans = transOdom.GetLocalPosture(rawScan.odom_data.global_pst);
      // [-PI, PI]
   odom_trans.fThita = CAngle::NormAngle2(odom_trans.fThita);
   m_PreOdomPst = rawScan.odom_data.global_pst;


   odom["x"] = Json::Value(odom_trans.x);
   odom["y"] = Json::Value(odom_trans.y);
   odom["yaw"] = Json::Value(odom_trans.fThita);
   odom["time"] = Json::Value(time);



   int index = 0;

   if(m_vtImgs.size()>=1)
       index = m_vtImgs.size();

   img["idx"] = Json::Value(index);
   img["time"] = Json::Value((uint64_t)GetTickCount());

   cv::Mat image;
   tp.GetFrame(image);
   img_record.push_back(image);

   m_vtLasers.push_back(laser);
   m_vtImgs.push_back(img);
   m_vtPoses.push_back(pose);
   m_vtOdoms.push_back(odom);

   LCMTask::GetLcmInstance().SendImageIndex(index);




}
int CRecTopvisionImage::StopRecord()
{
    std::lock_guard<std::mutex> lock(m_mtxRecord);

   /* SetEvent ( m_hKillThread );
    WaitForSingleObject ( m_hThreadDead, 5000 );
    PthreadJoin ( m_pMappingThread );*/

   /* if ( m_hKillThread != NULL )
    {
        CloseHandle ( m_hKillThread );
        m_hKillThread = NULL;
    }

    if ( m_hThreadDead != NULL )
    {
        CloseHandle ( m_hThreadDead );
        m_hThreadDead = NULL;
    }*/



    if(!m_bCameraInit)
        return RECORDIMG_INIT_FALURE;


    ofstream os;
    os.open(WORK_PATH"ImgRecord/LaserMsg.json", ios::out);
    if(!os.is_open())
    {
        std::cout<<"Error: can not find create the file which named \"LaserMsg.json\"."<<std::endl;
         Reset();
        return RECORDIMG_OPENFILE_ERROR;

    }


    for(int i = 0 ;i<m_vtLasers.size();i++)
    {

        Json::Value root;

       root["laser"] = Json::Value(m_vtLasers[i]);
       root["img"] = Json::Value(m_vtImgs[i]);
       root["pose"] = Json::Value(m_vtPoses[i]);
       root["odom"] = Json::Value(m_vtOdoms[i]);
       Json::FastWriter fw;
       if(os)
           os << fw.write(root);
    }


    if(os)
       os.close();


    std::cout<<"img_record.size()"<<img_record.size()<<std::endl;


        for(int q = 0; q < img_record.size(); q++)
        {
            std::string imgname = WORK_PATH"ImgRecord/"+to_string(q) + ".jpg";
            cv::imwrite(imgname,img_record[q]);
        }
        img_record.clear();

        tp.Stop();

        // 发送消息通知进程启动定位模式
      auto pRoboClnt = RoboClntSingleton::GetInstance();
      pRoboClnt->SetLocMode_Cam();
      usleep(2000*1000);

       Reset();

    return RECORDIMG_STOP_SUCCESS;

}
void CRecTopvisionImage::Reset()
{

         img_record.clear();
         m_vtLasers.clear();
         m_vtImgs.clear();
         m_vtPoses.clear();
         m_vtOdoms.clear();


         m_bFirstTime = true;
         m_bLaserParm = false;
         m_bCameraInit = false;
}

}

