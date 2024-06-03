#include "stdafx.h"

#include "AffinePosture.h"

#include "voxel_filter.h"
#include "gridmap.h"
#include "SlamMethod.h"
#include "BasObject.h"

#include "LinuxSetting.h"

#include <iostream>

#include <fstream>
#include <iostream>

#include "LinuxSetting.h"
#include "MagicSingleton.h"
#include "json/json.h"
#include "AutoOutPutBlackBox.h"
#include "Project.h"

#include "blackboxhelper.hpp"

#include "Tools.h"
#include "RoboLocClnt.h"
#include "AutoOutPutBlackBox.h"
#include "blackboxhelper.hpp"
#include "ParameterObject.h"
#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif


////////////////////////////////////////////////
//   实现栅格slam的定位方法
//   Author: lishen
//   Date:   2022. 10.30
///////////////////////////////////////////////


using namespace mapping;
using namespace proto;
using namespace common;

CSlamMethod *pCSlamMethod;

#if 0
CSlamParam::CSlamParam()
{
    m_ratioMatchWithMap = 0.2;      //
    m_rationMatchWithLaser = 0.2;  //
    m_linearSearchWindow = 0.5;
    m_angularSearchWindow =25.0*PI/180.0;
    m_scoreFastMatch = 0.3;

}

CSlamParam::CSlamParam(const CSlamParam &other)
{
    *this = other;
}

CSlamParam *CSlamParam::Duplicate()
{
    CSlamParam *copy = new CSlamParam;
    *copy = *this;
    return copy;
}

bool CSlamParam::LoadBinary(FILE *fp)
{
    return true;
}

bool CSlamParam::SaveBinary(FILE *fp)
{
    return true;
}
#endif

//
//   By lishen
//
CSlamMethod::CSlamMethod()
{
    type_ = 4;
    m_score = 0;
   // map_= nullptr;


    m_locOK = false;
    matchInfo_.result_ = CMatchInfo::MATCH_FAIL;
    m_SlamCount = 1;
    m_SlamMaxDis = 0;
    m_SlamTotalDis = 0;
    m_realtimescore = 0;

    pCSlamMethod = this;
     m_pCartoSlam = nullptr;
    if ( m_pCartoSlam == nullptr )
    {
        m_pCartoSlam = CCartoSlam::GetInstance();
        m_pCartoSlam -> RegisterSlamCallBack ( (SlamResultCbFunc)CartoSlamCallback );
        m_pCartoSlam -> RegisterBuildOverCallBack((BuildOverFunc)SlamOverCallback);
        m_pCartoSlam -> AddLaserID( (std::string)("scan0") );
    }
    m_bSlaming = false;
    m_res = true;

}
CSlamMethod::~CSlamMethod()
{

}

bool CSlamMethod::Initialize()
{
    return true;
}


bool CSlamMethod::UnloadMap()
{
    return true;
}

//
//   生成一个适用于本方法的定位参数块。
//
CLocalizationParam *CSlamMethod::CreateLocParam()
{
    //std::cout<<"CSlamMethod::CreateLocParam"<<std::endl;
    return new CSlamLocalizationParam;
}

//
//   应用指定的定位参数。
//
bool CSlamMethod::ApplyParam(const CLocalizationParam *param)
{
    CSlamLocalizationParam *p = (CSlamLocalizationParam*)param;
    if (p == NULL)
        return false;

    m_param = *p;

    return true;
}

//
//   By lishen : slam call back function
//
void CSlamMethod::CartoSlamCallback(slam_result *pSlamResult, std::vector<opt_result> *pOptResult)
{
    pCSlamMethod->SlamCallbackFunc ( pSlamResult, pOptResult );
}

void CSlamMethod::SlamOverCallback()
{

}

//
//   By lishen : 得到定位结果
//

void CSlamMethod::SlamCallbackFunc(slam_result *pSlamResult, std::vector<opt_result> *pOptResult)
{

    Pose xyt = pSlamResult->local_pose;
    Pose global_xyt = xyt;
    bool bfind = false;

    m_realtimescore = pSlamResult->realtimescore;

   //   printf("SlamCallbackFunc***********xyt = %f %f %f\n", xyt.x, xyt.y,xyt.theta);


   // mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose>  trajectory_nodes_= m_pCartoSlam->GetTrajectoryNodePoses();

    //auto begin_it = trajectory_nodes_.begin();
    //auto end_it = trajectory_nodes_.end();



    mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> trajectory_nodes_ = m_pCartoSlam->GetTrajectoryNodePoses();



    for (const auto& nodedata : trajectory_nodes_)
    {

    //for (const auto& node_id_data : trajectory_nodes_)
    //for (auto it= end_it; it!=begin_it;--it)

        mapping::TrajectoryNodePose::ConstantPoseData constant_pose_data = nodedata.data.constant_pose_data.value();
        Pose originpos;

       // data.node_id = node_id_data.id.node_index;
       // data.stamp.tv_sec = constant_pose_data.time.Sec();
       // data.stamp.tv_usec = constant_pose_data.time.Usec();

        double theta = transform::GetYaw(constant_pose_data.local_pose.rotation());
        Eigen::Matrix<double, 3, 1> xyz = constant_pose_data.local_pose.translation();

        originpos.x = xyz(0);
        originpos.y = xyz(1);
        originpos.theta = theta;

        for (const auto& node_id_data : *pOptResult)
        {
            Pose lastoptpos = node_id_data.global_pose;

             if(node_id_data.stamp.tv_sec == constant_pose_data.time.Sec() && node_id_data.stamp.tv_usec == constant_pose_data.time.Usec())
            {
                bfind = true;

                //printf("lastoptpos = %f %f %f\n", lastoptpos.x, lastoptpos.y,lastoptpos.theta);
                //printf("originpos = %f %f %f\n", originpos.x, originpos.y,originpos.theta);
                Pose delta;
                delta = common::xytInvMul31(originpos,xyt);

                //printf("delta = %f %f %f\n", delta.x, delta.y,delta.theta);
                global_xyt = common::xytMultiply(lastoptpos, delta);


                //printf("***********xyt = %f %f %f\n", xyt.x, xyt.y,xyt.theta);
                //printf("send_local_xyt = %f %f %f\n", send_local_xyt.x, send_local_xyt.y,send_local_xyt.theta);
                break;
            }

        }
        if(bfind)
            break;
    }
    m_posSlamResult.x = global_xyt.x;
    m_posSlamResult.y = global_xyt.y;
    m_posSlamResult.fThita = global_xyt.theta;


    printf("SlamCallbackFunc***********global_xyt = %f %f %f\n", global_xyt.x, global_xyt.y,global_xyt.theta);


}

//
//   By lishen : 设置初始节点位姿
//
void CSlamMethod::SetInitNodePose(const Eigen::Affine3d &initPose,unsigned long long timestamp)
{
    CPosture initposture = AffineToPosture(initPose);
    //const transform::Rigid2d init_pose({initposture.x, initposture.y},initposture.fThita);
    Pose initpos;
    initpos.x = initposture.x;
    initpos.y = initposture.y;
    initpos.theta = initposture.fThita;


    if ( m_pCartoSlam != nullptr )
    {
        m_pCartoSlam->Clear();
        m_pCartoSlam->SetInitNodePose(timestamp,initpos);
    }
}

//
//   By lishen : 释放资源
//
 void CSlamMethod::Clear()
 {
     if ( m_pCartoSlam != nullptr )
     {
         m_pCartoSlam->Clear();
     }
 }

 //
 //   By lishen : 初始化slam
 //

void CSlamMethod::InitLocate()
{
    //std::cout<<"InitLocate"<<std::endl;
   /* tmpmap_ = common::make_unique<ProbabilityGrid>(map_->limits_);

    tmpmap_->known_cells_box_ = map_->known_cells_box_;
    tmpmap_->correspondence_cost_cells_ = map_->correspondence_cost_cells_;
    tmpmap_->min_correspondence_cost_ = map_->min_correspondence_cost_;
    tmpmap_->max_correspondence_cost_ = map_->max_correspondence_cost_;
    tmpmap_->update_indices_ = map_->update_indices_;*/

    auto pCartoParm = CartoParmSingleton::GetInstance();


   // pCartoParm->SetFastScanMatchParam(1.2, PI/6);
   // pCartoParm->SetConstraintBuilderParam(15,0.3);
   // pCartoParm->SetMotionFilterParam(0.2,PI,300);
   // pCartoParm->SetPoseGraphParam(20);
   // pCartoParm->SetSubmapsParam(20);


    Pose initpos;
    initpos.x = 0;
    initpos.y = 0;
    initpos.theta = 0;

    double timestamp = 0;

    if ( m_pCartoSlam != nullptr )
    {
        std::cout<<"CSlamMethod::InitLocate"<<std::endl;
        m_pCartoSlam -> RegisterSlamCallBack ( (SlamResultCbFunc)CartoSlamCallback );
        m_pCartoSlam -> RegisterBuildOverCallBack((BuildOverFunc)SlamOverCallback);
        m_pCartoSlam -> StartLocate(initpos,nullptr,timestamp);
    }



}
//
//   By lishen :  Stop slam
//


void CSlamMethod::StopSlamLocate()
{
    if ( m_pCartoSlam != nullptr )
    {
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "Stop  Slam");
#endif
       m_pCartoSlam->StopLocate();

    }
}

//
//   By dq : 是否由slam  切到 scanmatch
//

bool CSlamMethod::ChangeToScanMatchMethod(int methodId, const bool bInitLocSuccess)
{
    // dq 11.25

    bool UnuseScanMatchMethod = true;

    if(methodId == type_ && bInitLocSuccess)
    {
        if(m_SlamCount%5 ==0)
            UnuseScanMatchMethod = false;
    }
    else if(methodId == type_ && !bInitLocSuccess)
    {
        UnuseScanMatchMethod = false;
       // continue;
    }
    else
    {
        ResetOdom();
        UnuseScanMatchMethod = true;
    }
    return UnuseScanMatchMethod;
}

bool CSlamMethod::IsChangeToOtherMethod()
{
    // dq 11.25

    bool res = false;

    if(m_SlamCount%5 ==0)
          res = true;

    return res;
}



//
//   By lishen
//
void CSlamMethod::SetLastPos(unsigned long long laststamp, Eigen::Affine3d lpos)
{

    m_lastPos = lpos;
    m_lastScanMsg = m_scanMsg;
    m_lastOdomMsg = m_odomMsg;



}

//
//   By lishen
//
bool CSlamMethod::LocalizeProc(int localMode, const Eigen::Affine3d &initPose,
                                   const ndt_oru::CStampedPointCloud cloudIn,
                                   Eigen::Affine3d &estimatePose)
{

    timeval time1;
    timeval time3;

    gettimeofday(&time1,NULL);
    #ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "******************USE TYPE 4 METHOD THIS FRAME*****************");
    #endif



    if(m_SlamCount == 1)
    {
        #ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "########## SetInitNodePose ##########");
        #endif

         std::cout<< " m_laststamp ="<<m_scanMsg.stamp <<" "<<m_lastScanMsg.stamp<<std::endl;

        if((m_scanMsg.stamp -m_lastScanMsg.stamp)<500)
        {
            std::cout<<"*********************  SetInitNodePose  last ********** \n";
            SetInitNodePose(m_lastPos,m_lastScanMsg.stamp);
            m_pCartoSlam->HandleEncoderData ( &m_lastOdomMsg );
            m_pCartoSlam->HandleLaserData ( &m_lastScanMsg );
        }
        else
           SetInitNodePose(initPose,m_odomMsg.stamp);

    }

    if ( m_pCartoSlam != nullptr )
    {
        m_pCartoSlam->HandleEncoderData ( &m_odomMsg );
        m_pCartoSlam->HandleLaserData ( &m_scanMsg );
        m_SlamCount++;
    }


    bool result = true;
    double grid_quality = 0.5;
    double point_quality = 0.5;

    CPosture initposture = AffineToPosture(initPose);
    const transform::Rigid2d init_pose({initposture.x, initposture.y},initposture.fThita);

    m_locOK = false;


    std::cout<<" **** initposture      = "<<initposture.x<<"  "<<initposture.y<<"  "<<initposture.fThita<<std::endl;
    std::cout<<" **** m_posSlamResult      = "<<m_posSlamResult.x<<"  "<<m_posSlamResult.y<<"  "<<m_posSlamResult.fThita<<std::endl;
    std::cout<<" ****m_realtimescore     = "<<m_realtimescore<< "m_SlamCount"<<m_SlamCount<< std::endl;



   // if(m_realtimescore < 0.25 && m_SlamCount > 5)
   //     m_res = false;

  //  if(UseSlam(m_posSlamResult) && m_res)
    if(UseSlam(m_posSlamResult))
    {
        matchInfo_.pst_ = m_posSlamResult;
        matchInfo_.result_ = CMatchInfo::MATCH_OK;
        estimatePose = PostureToAffine(m_posSlamResult);
        m_locOK = true;
        result = true;

        std::cout<<"!!!!!!!!!SLAMMETHOD RETURN TRUE!!!!!"<<"  Slam dis: "<< m_SlamMaxDis <<"  slam total dis: "<< m_SlamTotalDis <<" Res: "<< m_res << " score "<<m_realtimescore<<std::endl;
        #ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox,"TYPE 4 METHOD RETURN TRUE!!!!!  Farthest Dis: ", m_SlamMaxDis ,"   Total Dis: ", m_SlamTotalDis ,"   Res: ", m_res);
        #endif
    }
    else
    {
        matchInfo_.pst_ = initposture;
        matchInfo_.result_ = CMatchInfo::MATCH_FAIL;
        m_locOK = false;
        result = false;
        std::cout<<"!!!!!!!!!SLAMMETHOD RETURN FALSE!!!!!!!"<<"  Slam Dis: "<< m_SlamMaxDis <<"  Slam Total Dis: "<< m_SlamTotalDis <<" Res: "<< m_res <<" score "<<m_realtimescore<<std::endl;
        #ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox,"TYPE 4 METHOD RETURN FALSE!!!!!  Farthest Dis: ", m_SlamMaxDis ,"   Total Dis: ", m_SlamTotalDis ,"   Res: ", m_res);
        #endif
    }

    matchInfo_.type_= CMatchInfo::LOC_SLAM;
    matchInfo_.matchNum_ = point_quality*100;
    matchInfo_.matchRatio_ = grid_quality*100;
    matchInfo_.cloudIn = cloudIn;
    matchInfo_.initposture = initposture;

    m_lastRobotPos = matchInfo_.pst_;

    gettimeofday(&time3,NULL);

  //  double time = (time3.tv_sec - time1.tv_sec)*1000 + (double)(time3.tv_usec -time1.tv_usec)/1000  ;

    return result;

}


//
//   取得匹配数据。
//
CMatchInfo *CSlamMethod::GetMatchInfo()
{
    return &matchInfo_;
}

//
//   对定位质量进行评估。
//
bool CSlamMethod::EvaluateQuality(float &score)
{
    return m_locOK;
}

//
//   By dq 采用slam超出设定范围  返回false
//
bool CSlamMethod::UseSlam(CPosture &posSlamResult)
{
    if(m_SlamCount == 2)
    {
        lastlocal_pst = posSlamResult;
        orilocal_pst = posSlamResult;
        local_pst = posSlamResult;
        m_SlamTotalDis = 0;
        m_SlamMaxDis = 0;
        m_res = true;
    }
    if(m_SlamCount > 2)
    {
        local_pst = posSlamResult;
        m_SlamTotalDis += sqrt(pow(lastlocal_pst.x-local_pst.x,2)+pow(lastlocal_pst.y-local_pst.y,2));
        m_SlamMaxDis = sqrt(pow(orilocal_pst.x-local_pst.x,2)+pow(orilocal_pst.y-local_pst.y,2));
        lastlocal_pst = local_pst;
    }
    if(m_SlamMaxDis > m_param.SlamMaxDis || m_SlamTotalDis > m_param.SlamTotalDis )
        return false;
    else
        return true;
}

//
//   By dq  reset
//

void CSlamMethod::ResetOdom()
{
    m_SlamCount = 1;
    m_SlamTotalDis = 0;
    m_SlamMaxDis = 0;
    m_res = true;
}

//
//   从txt文件装入地图。
//
bool CSlamMethod::LoadBinary(FILE *fp, string filename,int floor, bool bChangeFloor)
{
    return true;
}




bool CSlamMethod::SaveBinary(FILE *fp, string filename)
{
    return true;

}

// By Sam
bool CSlamMethod::ReSetMethod()
{
    return true;
}

