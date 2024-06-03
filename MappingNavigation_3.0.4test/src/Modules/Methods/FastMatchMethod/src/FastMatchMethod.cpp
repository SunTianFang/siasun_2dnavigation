#include "stdafx.h"
#include "FastMatchMethod.h"
#include "AffinePosture.h"
#include "ceres_scan_matcher_2d.h"
#include "voxel_filter.h"

#include "gridmap.h"


#include "BasObject.h"
#include <iostream>

#include "Tools.h"
#include "blackboxhelper.hpp"
#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

////////////////////////////////////////////////
//   分支限界 实现大范围定位
//   Author: lishen
//   Date:   2022. 1.
///////////////////////////////////////////////


using namespace mapping;
using namespace proto;
using namespace common;

//
//   By lishen  没用CFastMatchParam   在method中设的参数
//
CFastMatchParam::CFastMatchParam()
{
  // linear_search_window = 0.5 ;
 //  angular_search_window = 3.14159/9;


}

CFastMatchParam::CFastMatchParam(const CFastMatchParam &other)
{
    *this = other;
}

//
//   生成本数据的一个副本。
//
CFastMatchParam *CFastMatchParam::Duplicate()
{
    CFastMatchParam *copy = new CFastMatchParam;
    *copy = *this;
    return copy;
}

bool CFastMatchParam::LoadBinary(FILE *fp)
{


    return true;
}

bool CFastMatchParam::SaveBinary(FILE *fp)
{

    return true;
}

CFastMatchMethod::CFastMatchMethod()
{
    type_ = 3;
    m_score = 0;
    m_curSubmapId = 0;
}

CFastMatchMethod::~CFastMatchMethod()
{

}

//
//   By lishen  由ndt地图生成分支限界用的地图
//

bool CFastMatchMethod::CreateGridMap(ndt_oru::NDTMaps *ndtMaps,FILE *fp)
{
    int submapnum = 0;

    for (int i=0;i<ndtMaps->size();i++)
    {

        CRectangle  rect_ = ndtMaps->at(i)->GetCoveringRect();
        // 取得左上角点的位置
        CPnt ptLeftTop = rect_.GetLeftTopPoint();
        // 取得右下角点的位置
        CPnt ptRightBottom = rect_.GetRightBottomPoint();

        if(fabs(ptRightBottom.x-ptLeftTop.x)<0.5 && fabs(ptLeftTop.y -ptRightBottom.y)<0.5)
            continue;
        submapnum++;
    }
    if(submapnum == 0)
        return false;

    //子图个数
    if (fwrite(&submapnum, sizeof(int), 1, fp) != 1)
     return false;


    for (int i=0;i<ndtMaps->size();i++)
    {
        double cx = -1;
        double cy = -1;
        double cz = -1;
        ndtMaps->at(i)->getCellSize(cx, cy, cz);

        if(cx<=0||cx>0.5||cy<=0||cy>0.5)
            continue;

        CRectangle  rect_ = ndtMaps->at(i)->GetCoveringRect();

        // 取得左上角点的位置
        CPnt ptLeftTop = rect_.GetLeftTopPoint();
        // 取得右下角点的位置
        CPnt ptRightBottom = rect_.GetRightBottomPoint();


        //std::cout<<"ptLeftTop"<<ptLeftTop.x <<" "<<ptLeftTop.y<<std::endl;
        //std::cout<<"ptRightBottom"<<ptRightBottom.x <<" "<<ptRightBottom.y<<std::endl;

        //由ndt地图重采样得到点云
        ndt_oru::CPointCloud resampledCloud;
        ndtMaps->at(i)->Resample(0.02,resampledCloud);

        if(fabs(ptRightBottom.x-ptLeftTop.x)<0.5 && fabs(ptLeftTop.y -ptRightBottom.y)<0.5)
            continue;

        //外扩10米
        double x0 = ptLeftTop.x - 5;
        double y0 = ptRightBottom.y - 5;

        double metersPerPixel = 0.1;
        int branch_and_bound_depth = 5;
        bool bGuass = false;

        double width =  ptRightBottom.x-ptLeftTop.x + 10;
        double height = ptLeftTop.y -ptRightBottom.y + 10;


        std::cout<<"metersPerPixel"<<metersPerPixel<<std::endl;
        std::cout<<"branch_and_bound_depth"<<branch_and_bound_depth<<std::endl;

        int nwidth = (width)/metersPerPixel;
        int nheight = (height)/metersPerPixel;


        double limits_max_x =  x0 + nwidth * metersPerPixel;
        double limits_max_y =  y0 + nheight * metersPerPixel;


        std::unique_ptr<mapping::ProbabilityGrid> map;
        map = nullptr;
        map = common::make_unique<mapping::ProbabilityGrid>( MapLimits(metersPerPixel, limits_max_x, limits_max_y, CellLimits(nheight, nwidth)));

        printf("create a %d X %d gridmap  %.3f m/pix  leftbottom  %f,  %f \n", nwidth,nheight,metersPerPixel,x0,y0);


        LUT lut;
        GridMap *globalGaussianMap = new GridMap;

        globalGaussianMap->makePixels(x0,y0,nwidth, nheight,metersPerPixel,(unsigned char)0,false);
        globalGaussianMap->makeGaussianLUT(1.0, 0, 1.0 / (0.06*0.06),lut);

        //高斯滤波
        for(int i=0;i<resampledCloud.size();i++)
        {
            if(bGuass)
                globalGaussianMap->drawDot(resampledCloud.at(i)(0), resampledCloud.at(i)(1), lut,lut.length);
            else
                globalGaussianMap->setValue(resampledCloud.at(i)(0), resampledCloud.at(i)(1),0xFF);

        }


        for (int i = 0; i < nheight * nwidth; i++)
        {

            int data ;
            unsigned char value = globalGaussianMap->data[i];
            double prob = ((double)value) /255.0;

            if(prob>0.9)
                prob = 0.9;
            if(prob<0.1)
                prob = 0.1;

            // get index in Grid2D corresponding to the k-th element in OccupancyGrid
            Eigen::Vector2i msgIdx((int)i / nwidth,i % nwidth);  // row major order

            const float padding =-0.01 * metersPerPixel;  // needed to make sure we get the interior
                                           // of the cell, instead of the boundary
            Eigen::Vector2f msg_cell_position(
                (msgIdx(1) + 1) * metersPerPixel + x0 +padding,
                (msgIdx(0) + 1) * metersPerPixel + y0 +padding);  // msgIdx is (horizontal, vertical)
            Eigen::Vector2i grid_index =  map->limits().GetCellIndex(msg_cell_position);  // cell index in carto::mapping::Grid2D

            map->SetProbability(grid_index, prob);

        }

        delete globalGaussianMap;

        proto::FastCorrelativeScanMatcherOptions2D fastmatchoption={0.5, 3.1415/9, branch_and_bound_depth};
        std::shared_ptr<scan_matching::FastCorrelativeScanMatcher2D> matcher;

        matcher= std::make_shared<scan_matching::FastCorrelativeScanMatcher2D>(*map, fastmatchoption);

        //子图ID
        int submapID = i;
        if (fwrite(&submapID, sizeof(int), 1, fp) != 1)
         return false;

        matcher->SaveBinary(fp);

    }

    fclose(fp);
    return true;
}

//
//   By lishen   clear map
//
bool CFastMatchMethod::UnloadMap()
{

   std::map<int, std::shared_ptr<mapping::scan_matching::FastCorrelativeScanMatcher2D>>::iterator iter;

   if(matchers_.size()<=0)
       return true;

   for(iter=matchers_.begin();iter!=matchers_.end();iter++)
    {
        if(iter->second!=nullptr)
         {
            iter->second.reset();
            iter->second = nullptr;
        }
    }
   matchers_.clear();

    return true;
}


bool CFastMatchMethod::Initialize()
{

    return true;
}

//
//   生成一个适用于本方法的定位参数块。
//
CLocalizationParam *CFastMatchMethod::CreateLocParam()
{
    return new CFastMatchParam;
}

//
//   应用指定的定位参数。
//
bool CFastMatchMethod::ApplyParam(const CLocalizationParam *p)
{
    if (p == NULL)
        return false;

    // 将参数复制到本对象中，以方便应用
  //  param_ = *((CTemplateLocalizationParam *)p);
    return true;
}

//
//   By lishen   分支限界 匹配范围
//
void CFastMatchMethod::SetParam(float linear_search_window,float angular_search_window)
{
    m_linearSearchWindow = linear_search_window;
    m_angularSearchWindow = angular_search_window;
}
//
//   By lishen   设置子图ID  目前只有一个子图
//
 void CFastMatchMethod::SetMapID(int submapID)
 {
     m_curSubmapId = submapID;
 }

 //
 //   By lishen  定位流程
 //

bool CFastMatchMethod::LocalizeProc(int localMode, const Eigen::Affine3d &initPose,
                                   const ndt_oru::CStampedPointCloud cloudIn,
                                   Eigen::Affine3d &estimatePose)
{

    //std::cout << "curSubmapId = " << m_curSubmapId << std::endl;
    if(matchers_.find(0)==matchers_.end())
    {
        std::cout << "curSubmapId return false" << std::endl;
        return false;
    }


    timeval time1;
    timeval time3;

    gettimeofday(&time1,NULL);

    sensor::PointCloud allPointcloud;
    sensor::PointCloud point_cloud;
    bool result = true;



    for (int i = 0; i < cloudIn.size(); i++)
    {
        CPnt pt;
        pt.x = cloudIn[i](0);
        pt.y = cloudIn[i](1);

        point_cloud.push_back({Eigen::Vector3f{pt.x, pt.y, 0.f}});

    }


    proto::AdaptiveVoxelFilterOptions adaptive_voxel_filter_options;
    adaptive_voxel_filter_options.max_length = 0.2;
    adaptive_voxel_filter_options.min_num_points = 0;
    adaptive_voxel_filter_options.max_range = 50.0;

    sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
        adaptive_voxel_filter_options);
    const sensor::PointCloud filtered_point_cloud =
        adaptive_voxel_filter.Filter(point_cloud);




    CPosture initposture = AffineToPosture(initPose);
    const transform::Rigid2d init_pose({initposture.x, initposture.y},initposture.fThita);
    transform::Rigid2d result_pose = init_pose;

  //  std::cout<<" point_cloud size =  "<<point_cloud.size()<<std::endl;

  //  std::cout<<" &&&&&&&&&&&&&&&& fastmatch init   = "<<initposture.x<<"  "<<initposture.y<<"  "<<initposture.fThita<<std::endl;

    double kMinScore = 0.1;



    if(localMode == FAST_MATCH)
    {
        if(m_linearSearchWindow<1.01)
            matchers_[0]->SetMaxTimeLimit(100);  //ms
        else
            matchers_[0]->SetMaxTimeLimit(-1);
        result = matchers_[0]->Match(init_pose,point_cloud, kMinScore, &m_score, &result_pose,m_linearSearchWindow,m_angularSearchWindow);

    }


    const CPosture resultPosture(result_pose.translation().x(),result_pose.translation().y(),result_pose.rotation().angle());
    estimatePose = PostureToAffine(resultPosture);


    matchInfo_.cloudIn = cloudIn;
    matchInfo_.pst_ = resultPosture;
    matchInfo_.initposture = initposture;

    gettimeofday(&time3,NULL);

    double time = (time3.tv_sec - time1.tv_sec)*1000 + (double)(time3.tv_usec -time1.tv_usec)/1000  ;

    std::cout<<" **** fastmatch time   =    "<<time<<"ms"<<std::endl;
    std::cout<<" **** fastmatch result = "<<result_pose.translation().x()<<"  "<<result_pose.translation().y()<<"  "<<result_pose.rotation().angle()<<std::endl;
    std::cout<< " score= "<< m_score<<std::endl;
    #ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "FastMatch Loc Over... Loc Time = ", static_cast<int>(time),
                      " InitPose : ", initposture.x, ", ", initposture.y, ", ", initposture.fThita, " |",
                      " EstimatePose : ", result_pose.translation().x(), ", ", result_pose.translation().y(),", ", result_pose.rotation().angle(),
                      " | ", m_score);
    #endif
    return result;



}

//
//   取得匹配数据。
//
CMatchInfo *CFastMatchMethod::GetMatchInfo()
{
    return &matchInfo_;
}

//
//   对定位质量进行评估。
//
bool CFastMatchMethod::EvaluateQuality(float &score)
{

    score = m_score ;
    if(m_score<0.3)
        return false;
    return true;
}

//
//  by lishen  从二进制文件装入地图。
//
bool CFastMatchMethod::LoadBinary(FILE *fpp, string filename,int floor, bool bChangeFloor)
{

    //

    UnloadMap();

    string str;

    if(bChangeFloor)
        str =filename + "Gridmap"+"_"+to_string(floor)+".map";

    else
        str= filename+"Gridmap.map";



    FILE *fp = fopen(str.c_str(), "rb");
    if (fp == NULL)
        return false;

    int submapnum = 0;
    if (fread(&submapnum, sizeof(int), 1, fp) != 1)
        return false;

    for(int i=0;i<submapnum ;i++)
    {

        int submapID = i;
        if (fread(&submapID, sizeof(int), 1, fp) != 1)
         return false;


        proto::FastCorrelativeScanMatcherOptions2D fastmatchoption={0.5, 3.1415/9, 6};

        std::shared_ptr<scan_matching::FastCorrelativeScanMatcher2D> matcher;

        matcher= std::make_shared<scan_matching::FastCorrelativeScanMatcher2D>(fastmatchoption,MapLimits(0.05, 0, 0, CellLimits(100, 100)));


        if(matcher->LoadBinary(fp))
        {

             matchers_[submapID]= matcher;
              std::cout<<"load gridmap.map sucess"<<std::endl;
        }
        else
        {
              std::cout<<"load gridmap.map failed"<<std::endl;
        }


    }

    fclose(fp);
    return true;
}

//
//   将地图写入二进制文件。
//
bool CFastMatchMethod::SaveBinary(FILE *fp, string filename)
{


    return true;
}

// By Sam For LegMethod
bool CFastMatchMethod::ReSetMethod()
{
    return true;
}




