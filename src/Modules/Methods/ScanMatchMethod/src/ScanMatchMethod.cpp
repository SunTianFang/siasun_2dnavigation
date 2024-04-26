#include "stdafx.h"
#include "FastMatchMethod.h"
#include "AffinePosture.h"
#include "ceres_scan_matcher_2d.h"
#include "voxel_filter.h"
#include "gridmap.h"
#include "ScanMatchMethod.h"
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
//   实现基于栅格的定位方法
//   采用了cartographer中realtime 和cere定位方法实现基于栅格地图的定位
//   Author: lishen
//   Date:   2022. 6.
//   modify:  dq  加黑匣子   由 pad 设置初始定位时把分支限界范围变大
///////////////////////////////////////////////


using namespace mapping;
using namespace proto;
using namespace common;



//
//   By lishen
//

CScanMatchMethod::CScanMatchMethod()
{
    type_ = 3;
    m_score = 0;
    map_= nullptr;
    m_countFailed = 0;
    m_pFastMatcher = nullptr;
    m_locOK = false;
    matchInfo_.result_ = CMatchInfo::MATCH_FAIL;

    m_param.m_ratioMatchWithMap = 0.25;      //
    m_param.m_rationMatchWithLaser = 0.20;  //
    m_param.m_linearSearchWindow = 0.5;
    m_param.m_angularSearchWindow =25.0*PI/180.0;
    m_param.m_scoreFastMatch = 0.3;
    m_param.m_reloc_linearSearchWindow = 1;
    m_param.m_reloc_angularSearchWindow = 45*PI/180.0;

    // load GridLocParm.json
    std::ifstream   FileGridLocParm(WORK_PATH"GridLocParm.json");
    Json::Reader    Jreader;
    Json::Value     GridLocParmRoot;
    if ( !FileGridLocParm) {
        printf ( "Load GridLocParm.json failed\n" );
        return;
    }
    if ( !Jreader.parse ( FileGridLocParm, GridLocParmRoot ) ) {
        printf ( "Load GridLocParm.json  Jreader.parse failed\n" );
        FileGridLocParm.close();
        return;
    }
    GridLocParmRoot.toStyledString();

    // MotionFilterOptions
    if ( !GridLocParmRoot["LocQuality"]["MatchRatioWithMap"].isNull() ) {
        m_param.m_ratioMatchWithMap = GridLocParmRoot["LocQuality"]["MatchRatioWithMap"].asDouble();
    }
    if ( !GridLocParmRoot["LocQuality"]["MatchRatioWithLaserPt"].isNull() ) {
        m_param.m_rationMatchWithLaser = GridLocParmRoot["LocQuality"]["MatchRatioWithLaserPt"].asDouble();
    }
    if ( !GridLocParmRoot["FastMatchRange"]["LinearSearchWindow"].isNull() ) {
        m_param.m_linearSearchWindow = GridLocParmRoot["FastMatchRange"]["LinearSearchWindow"].asDouble();
    }
    if ( !GridLocParmRoot["FastMatchRange"]["AngleSearchWindow"].isNull() ) {
        m_param.m_angularSearchWindow = (GridLocParmRoot["FastMatchRange"]["AngleSearchWindow"].asDouble())*PI/180.0;
    }
    if ( !GridLocParmRoot["FastMatchRange"]["FastMatchMinScore"].isNull() ) {
        m_param.m_scoreFastMatch= (GridLocParmRoot["FastMatchRange"]["FastMatchMinScore"].asDouble());
    }
    if ( !GridLocParmRoot["FastMatchRange"]["Reloc_LinearSearchWindow"].isNull() ) {
        m_param.m_reloc_linearSearchWindow = GridLocParmRoot["FastMatchRange"]["Reloc_LinearSearchWindow"].asDouble();
    }
    if ( !GridLocParmRoot["FastMatchRange"]["Reloc_AngleSearchWindow"].isNull() ) {
        m_param.m_reloc_angularSearchWindow = (GridLocParmRoot["FastMatchRange"]["Reloc_LinearSearchWindow"].asDouble())*PI/180.0;
    }

    m_param.m_localMode = 0;
    m_param.m_bUseSingleFeature = false;
}
CScanMatchMethod::~CScanMatchMethod()
{

}

bool CScanMatchMethod::Initialize()
{
    return true;
}
//
//   By lishen  set fast matcher handler
//

void CScanMatchMethod::SetFastMatch(CFastMatchMethod *p)
{
    if(p->matchers_.size()>0)
        m_pFastMatcher = p;

}
//
//   By lishen
//

bool CScanMatchMethod::UnloadMap()
{
    if(map_!= nullptr)
    {
        map_.reset();
        map_=nullptr;
    }

    return true;
}

//
//   生成一个适用于本方法的定位参数块。
//
CLocalizationParam *CScanMatchMethod::CreateLocParam()
{
    return new CScanMatchParam;
}

//
//   应用指定的定位参数。
//
bool CScanMatchMethod::ApplyParam(const CLocalizationParam *param)
{

    CScanMatchParam *p = (CScanMatchParam*)param;
    if (p == NULL)
    {
        m_param.m_bUseSingleFeature = false;
        m_param.m_localMode = 0;
        return true;
    }

    m_param.m_bUseSingleFeature = p->m_bUseSingleFeature;
    m_param.m_localMode = p->m_localMode;

    return true;
}

//
//    by lishen 栅格定位
//
bool CScanMatchMethod::LocalizeProc(int localMode, const Eigen::Affine3d &initPose,
                                   const ndt_oru::CStampedPointCloud cloudIn,
                                   Eigen::Affine3d &estimatePose)
{

    timeval time1;
    timeval time3;
    timeval time5;
    timeval time6;
    timeval time7;

    gettimeofday(&time1,NULL);

    CPosture initposture = AffineToPosture(initPose);

    matchInfo_.result_ = CMatchInfo::MATCH_FAIL;
    matchInfo_.type_= CMatchInfo::LOC_GRID;

    if(map_==NULL)
    {
        matchInfo_.matchNum_ = 0;
        matchInfo_.cloudIn = cloudIn;
        matchInfo_.pst_ = initposture;
        matchInfo_.initposture = initposture;
        matchInfo_.matchRatio_ = 0;
        m_locOK = false;
        m_lastRobotPos = initposture;
        matchInfo_.result_ = CMatchInfo::MATCH_LOADMAP_FAILED;
        return false;
    }

    bool result = true;
    sensor::PointCloud point_cloud;
    sensor::PointCloud filter_pt_for_quality;


    const transform::Rigid2d init_pose({initposture.x, initposture.y},initposture.fThita);
    transform::Rigid2d result_pose = init_pose;
    m_locOK = false;

    //dq
    #ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "Scan Match Method InitPosture: ", initposture.x,", ",initposture.y,", ",(initposture.fThita/ 3.14) * 180);
    #endif
    std::cout<<" **** initposture      = "<<initposture.x<<"  "<<initposture.y<<"  "<<(initposture.fThita/ 3.14) * 180<<std::endl;

    double deltaTheta = initposture.fThita-m_lastRobotPos.fThita;

    if(deltaTheta>PI)
        deltaTheta = deltaTheta - 2*PI;
    else if(deltaTheta<-PI)
        deltaTheta = deltaTheta + 2*PI;
		
	   //激光参数   计算评价指标用	
    int highscore_num = 0;
    int N = 1;
    scan_matching::LaserParam laserparam;

    laserparam.angleStep = asin(0.1 / 30);
    laserparam.beginAngle = -135.0*PI/180.0;
    laserparam.endAngle = 135.0*PI/180.0;
    laserparam.minRange = 0.3;
    laserparam.maxRange = 25.0;
    laserparam.xInRobot = 0;
    laserparam.yInRobot = 0;
    laserparam.thetaInRobot = 0;
	
	
    vector<Eigen::Vector3f> xySensors;
    for (size_t i = 0; i < m_pScannerGroupParam->size(); i++)
    {
        if(i==0)
        {
            CLaserScannerParam &ScannerParam = m_pScannerGroupParam->at(i);

            if(ScannerParam.m_nLineCount>3000)
                N = 3;
            else if(ScannerParam.m_nLineCount>1200)
                N = 2;
            laserparam.angleStep = ScannerParam.m_fReso*N;
            laserparam.beginAngle = ScannerParam.m_fStartAngle;
            laserparam.endAngle = ScannerParam.m_fEndAngle;
            laserparam.minRange = ScannerParam.m_fMinRange;
            laserparam.maxRange = ScannerParam.m_fMaxRange;

            laserparam.xInRobot = ScannerParam.m_pst.x;
            laserparam.yInRobot = ScannerParam.m_pst.y;
            laserparam.thetaInRobot = ScannerParam.m_pst.fThita;

            if( laserparam.maxRange >25.0)
                 laserparam.maxRange = 25;

            Eigen::Affine3d sensor_pose = PostureToAffine(ScannerParam.m_pst);
            //std::cout<< " x "<< ScannerParam.m_pst.x<< " y "<< ScannerParam.m_pst.y<< " fThita "<< ScannerParam.m_pst.fThita<<std::endl;

            Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> T = sensor_pose.cast<float>();

            Eigen::Vector3f scpt;
            scpt << 0.0, 0.0, 0.0 ;
            scpt = T * scpt;
            xySensors.push_back(scpt);

        }
    }

	  //激光点云 间隔取值  计算评价指标用

    vector<int> vtOmit;
    int kk = 0;            //??????  only one sensor
    for (int i = 0; i < cloudIn.size(); i++)
    {

       CPnt pt;
       bool isObject = true;
       pt.x = cloudIn[i](0);
       pt.y = cloudIn[i](1);
       point_cloud.push_back({Eigen::Vector3f{pt.x, pt.y, 0.f}});

       if(i%N==0)
       {
            if(xySensors.size()>0)
            {
                Eigen::Vector3f sensorpos = xySensors.at(0);
                if (fabs(pt.x-sensorpos(0))<0.1 && fabs(pt.y-sensorpos(1))<0.1)
                {
                      isObject = false;
                      vtOmit.push_back(kk);
                }
            }
            if(isObject)
            {
                //std::cout<<"i="<<i<<"x="<<pt.x<<"y="<<pt.y<<std::endl;
                 filter_pt_for_quality.push_back({Eigen::Vector3f{pt.x, pt.y, 0.f}});
            }
            kk++;
        }

    }

    // RealTime match 匹配方法  体素滤波参数
    proto::AdaptiveVoxelFilterOptions adaptive_voxel_filter_options;
    adaptive_voxel_filter_options.max_length = 0.15;
    adaptive_voxel_filter_options.min_num_points = 300;
    adaptive_voxel_filter_options.max_range = 50.0;

    sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
        adaptive_voxel_filter_options);
    const sensor::PointCloud filtered_point_cloud_for_realtime =
        adaptive_voxel_filter.Filter(point_cloud);

    // realtime定位 参数
    proto::RealTimeCorrelativeScanMatcherOptions options;
    options.angular_search_window = 15.0*PI/180.0;
    options.linear_search_window = 0.15;
    options.translation_delta_cost_weight =0.1;
    options.rotation_delta_cost_weight =0.05;
    options.angular_resolution = 0.5*PI/180;

    transform::Rigid2d initial_ceres_pose = init_pose;
    scan_matching::RealTimeCorrelativeScanMatcher2D real_time_correlative_scan_matcher_(options);

    // 如果滤波后点云size为0，返回false
    if (filtered_point_cloud_for_realtime.empty()) {
            m_score = 0;
            matchInfo_.type_= CMatchInfo::LOC_GRID;
            matchInfo_.result_ = CMatchInfo::MATCH_FAIL;
            matchInfo_.initposture = initposture;
            matchInfo_.matchRatio_ = 0;
            estimatePose = initPose;
            result = false;
            return result;
    }

    if((GetPose_byPad() || m_countFailed >= 3) && m_pFastMatcher!=nullptr)
    {
        #ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "----- FM Method START -----");
        #endif
        // 如果前几周期定位出错，直接调用分支限界
        float score = 0.0;
        Eigen::Affine3d fastResultPose;
        CPosture resultPosture;
        const transform::Rigid2d init_pose({initposture.x, initposture.y},initposture.fThita);


        m_pFastMatcher->SetParam(m_param.m_linearSearchWindow,m_param.m_angularSearchWindow);

        // 手动上线单独设置搜索框
        if(GetPose_byPad())
        {
            m_pFastMatcher->SetParam(m_param.m_reloc_linearSearchWindow,m_param.m_reloc_angularSearchWindow);
            // std::cout<<"2m 0.3*PI Search Window!!!!!!!!!!!!!!!"<<std::endl;
        }
        m_pFastMatcher->SetMapID(0);
        bool res = m_pFastMatcher->LocalizeProc(FAST_MATCH, initPose, cloudIn,fastResultPose);
        m_pFastMatcher->EvaluateQuality(score);

        // 手动上线提高定位指标
        if(GetPose_byPad())
            score -= 0.1;

        if(res && score>m_param.m_scoreFastMatch)
        {
            resultPosture = AffineToPosture(fastResultPose);
            transform::Rigid2d init_pose({resultPosture.x, resultPosture.y},resultPosture.fThita);
            initial_ceres_pose = init_pose;
            // by DQ
            #ifdef USE_BLACK_BOX
                FILE_BlackBox(LocBox, "!!!!!!!!FM Loc Success!!!!!!!! with Score: ", score);
            #endif
        }
        else
        {
            // by DQ
            #ifdef USE_BLACK_BOX
                FILE_BlackBox(LocBox, "!!!!!!!!FM Loc Failed!!!!!!!! with Score: ", score);
            #endif
        }
    }
    else
    {           
        //否则先用realtime定位
        gettimeofday(&time6,NULL);
        real_time_correlative_scan_matcher_.Match(init_pose, filtered_point_cloud_for_realtime,
                *map_,&initial_ceres_pose);
        gettimeofday(&time5,NULL);
        float realtime = (time5.tv_sec - time6.tv_sec)*1000 + (double)(time5.tv_usec -time6.tv_usec)/1000 ;
       // printf(" real_time_correlative_scan_matcher_time = %f\n", realtime );

        // by DQ
        #ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "Real-time Result: ",initial_ceres_pose.translation().x(),", ",initial_ceres_pose.translation().y(),", ",(initial_ceres_pose.rotation().angle()/3.14) * 180,", time: ",realtime);
        #endif

        //    std::cout<<" realtime**** resultPosture      = "<<initial_ceres_pose.translation().x()<<"  "<<initial_ceres_pose.translation().y()<<std::endl;

    }


 
     //质量评估，主要是为了判断是否为长廊
    //real_time_correlative_scan_matcher_.Match_1(initial_ceres_pose, filter_pt_for_quality,*map_,highscore_num);

    // cere scan match 匹配方法 体素滤波参数
    proto::AdaptiveVoxelFilterOptions adaptive_voxel_filter_options_forcere;
    adaptive_voxel_filter_options_forcere.max_length = 0.1;
    adaptive_voxel_filter_options_forcere.min_num_points = 0;
    adaptive_voxel_filter_options_forcere.max_range = 50.0;

    sensor::AdaptiveVoxelFilter adaptive_voxel_filter_for_cere(
          adaptive_voxel_filter_options_forcere);
    const sensor::PointCloud filtered_gravity_aligned_point_cloud_for_cere =
          adaptive_voxel_filter_for_cere.Filter(point_cloud);


    T_L_SUMMARY summary;
    proto::CeresScanMatcherOptions2D cereOptions;

    cereOptions.occupied_space_weight = 10;
    cereOptions.translation_weight = 40;//4
    cereOptions.rotation_weight = 1;
    cerematcher.SetParam(cereOptions);


     /* if(highscore_num>10 && m_countFailed==0)   //corridor
      {

          transform::Rigid2d tmp (init_pose.translation(),initial_ceres_pose.rotation());
          initial_ceres_pose = tmp;
          std::cout<<" >>>>>>>>>>> in corridor  init pose"<<init_pose.translation().x()<<"  "<<init_pose.translation().y()<<"  "<<initial_ceres_pose.rotation().angle()<<std::endl;

          cereOptions.occupied_space_weight = 4;
          cereOptions.translation_weight =8 ;
          cereOptions.rotation_weight =1;
          cerematcher.SetParam(cereOptions);


          // by DQ
          #ifdef USE_BLACK_BOX
              FILE_BlackBox(LocBox, ">>>>>>>>>> In Corridor !!!!!!>>>>>>>>>>", " InitPose: ",init_pose.translation().x(),", ",init_pose.translation().y(),", ",initial_ceres_pose.rotation().angle());
          #endif

     }*/

     //cere 定位
     cerematcher.Match(initial_ceres_pose.translation(),
                                       initial_ceres_pose,
                                       filtered_gravity_aligned_point_cloud_for_cere,
                                       *map_,
                                       &result_pose,summary);


    double grid_quality = 0;
    double point_quality = 0;

     //质量评估
    real_time_correlative_scan_matcher_.MatchWithQuality(result_pose, filter_pt_for_quality,*map_,grid_quality,point_quality,laserparam,vtOmit);


    const CPosture resultPosture(result_pose.translation().x(),result_pose.translation().y(),result_pose.rotation().angle());

    m_score = grid_quality;

    std::cout<<" cere**** resultPosture      = "<<resultPosture.x<<"  "<<resultPosture.y<<"  "<<(resultPosture.fThita/ 3.14) * 180<<std::endl;

    if(fabs(deltaTheta)>0.08)
    {
        point_quality = point_quality+0.05;
        grid_quality = grid_quality +0.05;
    }


    if(point_quality>m_param.m_rationMatchWithLaser && grid_quality>m_param.m_ratioMatchWithMap)
    {
        matchInfo_.matchNum_ = point_quality*100;
        matchInfo_.cloudIn = cloudIn;
        matchInfo_.pst_ = resultPosture;
        matchInfo_.initposture = initposture;
        matchInfo_.result_ = CMatchInfo::MATCH_OK;
        matchInfo_.matchRatio_ = grid_quality*100;
        estimatePose = PostureToAffine(resultPosture);
        m_countFailed = 0;
        m_locOK = true;
        result = true;
        m_lastRobotPos = resultPosture;
    }
    else
    {
        matchInfo_.matchNum_ = point_quality*100;
        matchInfo_.cloudIn = cloudIn;
        matchInfo_.pst_ = initposture;
        matchInfo_.initposture = initposture;
        matchInfo_.result_ = CMatchInfo::MATCH_FAIL;
        matchInfo_.matchRatio_ = grid_quality*100;
        m_countFailed++;
        m_locOK = false;
        result = false;
        m_lastRobotPos = initposture;
    }
    CPosture  pos;
    switch(m_param.m_localMode)
    {


            case 1:

                        pos.x = initposture.x;

                        if(m_locOK)
                            pos.x += 20000;
                        else
                            pos.x += 40000;

                        pos.y = matchInfo_.pst_.y;
                        pos.fThita = matchInfo_.pst_.fThita;
                        estimatePose = PostureToAffine(pos);
                        matchInfo_.result_ = CMatchInfo::MATCH_TO_SINGLEFEATURE;
                        return false;
            case 2:

                        pos.y = initposture.y;

                        if(m_locOK)
                            pos.y += 20000;
                        else
                            pos.y += 40000;

                        pos.x = matchInfo_.pst_.x;
                        pos.fThita = matchInfo_.pst_.fThita;
                        estimatePose = PostureToAffine(pos);
                        matchInfo_.result_ = CMatchInfo::MATCH_TO_SINGLEFEATURE;
                        return false;
            case 3:

                        pos.x = initposture.x;
                        pos.y = matchInfo_.pst_.y;
                        pos.fThita = matchInfo_.pst_.fThita;
                        estimatePose = PostureToAffine(pos);
                        #if defined USE_BLACK_BOX
                        FILE_BlackBox(LocBox, "In Long Corridor!!!----SCANMATCH_METHOD-X--- ");
                        #endif
                        break;
            case 4:

                        pos.y = initposture.y;
                        pos.x = matchInfo_.pst_.x;
                        pos.fThita = matchInfo_.pst_.fThita;
                        estimatePose = PostureToAffine(pos);
                        #if defined USE_BLACK_BOX
                        FILE_BlackBox(LocBox, "In Long Corridor!!!----SCANMATCH_METHOD-Y--- ");
                        #endif
                        break;


    }





    gettimeofday(&time3,NULL);

    double time = (time3.tv_sec - time1.tv_sec)*1000 + (double)(time3.tv_usec -time1.tv_usec)/1000  ;



    if(!m_locOK)
        std::cout<<" **** match time    =   "<<time<<"ms    ?????????????????????????????????????????????????????????????????????????????????????"<<grid_quality<<"point_quality= "<<point_quality<<std::endl;
    else
        std::cout<<" **** match time    =   "<<time<<"ms    ********************************************grid_quality= "<<grid_quality<<"point_quality= "<<point_quality<<std::endl;

        std::cout<<" **** match result = "<<result_pose.translation().x()<<"  "<<result_pose.translation().y()<<"  "<<result_pose.rotation().angle()<<std::endl;

    return result;

}

//
//   取得匹配数据。
//
CMatchInfo *CScanMatchMethod::GetMatchInfo()
{
    return &matchInfo_;
}

//
//   对定位质量进行评估。
//
bool CScanMatchMethod::EvaluateQuality(float &score)
{
    return m_locOK;
}

//
//    by lishen 从txt文件装入地图。
//
bool CScanMatchMethod::LoadBinary(FILE *fp, string filename,int floor, bool bChangeFloor)
{

    ifstream infile;
    string str;

    if(bChangeFloor)
    {
        str =filename + "ProbMap"+"_"+to_string(floor)+".txt";
    }
    else
    {
        str= filename+"ProbMap.txt";
    }

     infile.open(str,ios::in);

    if(!infile)
        return true;   // By Sam: temporary test for read map not return false

    double _range;
    double _resolution;
    double meterPer;
    int nwidth;
    int nheight;
    double origin_position_x;
    double origin_position_y;
    bool is_map_data_valid= true;


    infile>>_range;
    infile>>_resolution;
    infile>>nheight;
    infile>>nwidth;
    infile>>meterPer;
    infile>>origin_position_x;
    infile>>origin_position_y;


    if (nheight<=0 || nwidth<=0 || meterPer<0.01)
    {
        printf("ERR: Wrong Map File Format");
        return false;
    }

    printf("Received a %d X %d probmap  %.3f m/pix  %f,  %f \n", nwidth,nheight,_resolution,origin_position_x,origin_position_y);


    int num_x = nheight;
    int num_y = nwidth;

    double max_x = meterPer*num_y + origin_position_x;
    double max_y = meterPer*num_x + origin_position_y;


    map_ = nullptr;
    map_ = common::make_unique<mapping::ProbabilityGrid>( MapLimits(meterPer, max_x, max_y, CellLimits(num_x, num_y)));


    int i=0;

    while(!infile.eof())
    {

        int data  ;

        infile>>data;

        float prob = 1-((((double)data) /255.0)/1.25+0.1);


        //occupid  0.9
        if(prob>0.9)
             prob = 0.9;
        if(prob<0.15)
            prob = 0.1;
        // by dq 10-18
        //if(data == 127)
        //    prob = 0.1;

        // get index in Grid2D corresponding to the k-th element in OccupancyGrid
        Eigen::Vector2i msgIdx((int)i / nwidth,i % nwidth);  // row major order

        const float padding =-0.01 * _resolution;  // needed to make sure we get the interior
                                           // of the cell, instead of the boundary
        Eigen::Vector2f msg_cell_position(
            (msgIdx(1) + 1) * _resolution + origin_position_x +padding,
            (msgIdx(0) + 1) * _resolution + origin_position_y +padding);  // msgIdx is (horizontal, vertical)
        Eigen::Vector2i grid_index =  map_->limits().GetCellIndex(
            msg_cell_position);  // cell index in carto::mapping::Grid2D

        if(map_->limits_.Contains(grid_index))
        {
            map_->SetProbability(grid_index, prob);
        }
        i++;

    }


    infile.close();

    return true;
}

//
//  by lishen 将地图写入txt文件,共同保存分支限界地图
//
bool CScanMatchMethod::SaveBinary(FILE *fp, string filename)
{
    if(map_== nullptr)
        return true;


    ifstream infile;
    string str = filename+"ProbMap.txt";

    map_->saveMap(std::string(str));

    infile.open(str,ios::in);

    if(!infile)
        return true;

    double _range;
    double _resolution;
    double meterPer;
    int nwidth;
    int nheight;
    double origin_position_x;
    double origin_position_y;
    bool is_map_data_valid= true;

    infile>>_range;
    infile>>_resolution;
    infile>>nheight;
    infile>>nwidth;
    infile>>meterPer;
    infile>>origin_position_x;
    infile>>origin_position_y;

    if (nheight<=0 || nwidth<=0 || meterPer<0.01)
    {
        printf("ERR: Wrong Map File Format");
        return false;
    }

    printf("Received a %d X %d map  %.3f m/pix  %f,  %f \n", nwidth,nheight,_resolution,origin_position_x,origin_position_y);

    int i=0;

   /* int num_x = nheight + 20;
    int num_y = nwidth + 20;

    double max_x = meterPer*num_y + origin_position_x-0.5;
    double max_y = meterPer*num_x + origin_position_y-0.5;
*/

    int num_x = nheight;
    int num_y = nwidth;
    double max_x = meterPer*num_y + origin_position_x;
    double max_y = meterPer*num_x + origin_position_y;

    MapLimits limits(meterPer, max_x, max_y, CellLimits(num_x, num_y));


    map_ = nullptr;
    map_ = common::make_unique<mapping::ProbabilityGrid>( MapLimits(meterPer, max_x, max_y, CellLimits(num_x, num_y)));

    double reso =  map_->limits_.resolution();
    std::shared_ptr<ProbabilityGrid> fastmap_;

    fastmap_ = common::make_unique<mapping::ProbabilityGrid>( MapLimits(2*reso, max_x, max_y, CellLimits(num_x/2, num_y/2)));

    while(!infile.eof())
    {

        int data  ;

        infile>>data;

        float prob = 1-((((double)data) /255.0)/1.25+0.1);

        //occupid  0.9
        if(prob>0.9)
            prob = 0.9;
        if(prob<0.15)
            prob = 0.1;

        // by dq 10-18
        //if(data == 127)
        //    prob = 0.1;

        // get index in Grid2D corresponding to the k-th element in OccupancyGrid
        Eigen::Vector2i msgIdx((int)i / nwidth,i % nwidth);  // row major order


        const float padding =-0.01 * _resolution;  // needed to make sure we get the interior
                                       // of the cell, instead of the boundary
        Eigen::Vector2f msg_cell_position(
            (msgIdx(1) + 1) * _resolution + origin_position_x +padding,
            (msgIdx(0) + 1) * _resolution + origin_position_y +padding);  // msgIdx is (horizontal, vertical)

        Eigen::Vector2i grid_index =  map_->limits().GetCellIndex(
            msg_cell_position);  // cell index in carto::mapping::Grid2D


        if(map_->limits_.Contains(grid_index))
        {
                map_->SetProbabilityAnyWay(grid_index, prob);
                //////////////////fast match map

                MapLimits fstlimits(2*reso, max_x, max_y, CellLimits(num_x/2, num_y/2));

                Eigen::Vector2i grid_index_fastmatch =  fastmap_->limits().GetCellIndex(
                      msg_cell_position);  // cell index in carto::mapping::Grid2D


                if(fastmap_->limits_.Contains(grid_index_fastmatch))
                {
                    float curProb = fastmap_->GetProbability(grid_index_fastmatch);
                    if(prob>curProb)
                        fastmap_->SetProbabilityAnyWay(grid_index_fastmatch,prob);
                }

        }

        i++;

    }
    infile.close();



    int branch_and_bound_depth = 3;
    proto::FastCorrelativeScanMatcherOptions2D fastmatchoption={0.5, 3.1415/9, branch_and_bound_depth};
    std::shared_ptr<scan_matching::FastCorrelativeScanMatcher2D> matcher;

    matcher= std::make_shared<scan_matching::FastCorrelativeScanMatcher2D>(*fastmap_, fastmatchoption);

    string str2 = filename+"Gridmap.map";
    int submapnum = 1;

    FILE *fastfp = fopen(str2.c_str(), "wb");
       if (fastfp == NULL)
           false;

    //子图个数
    if (fwrite(&submapnum, sizeof(int), 1, fastfp) != 1)
        return false;

     //子图ID
    int submapID = 0;
    if (fwrite(&submapID, sizeof(int), 1, fastfp) != 1)
        return false;

    matcher->SaveBinary(fastfp);
    fclose(fastfp);


    std::cout<<"read grid map time =           "<<std::endl;


    return true;



}

// By Sam 
bool CScanMatchMethod::ReSetMethod()
{
    return true;
}

