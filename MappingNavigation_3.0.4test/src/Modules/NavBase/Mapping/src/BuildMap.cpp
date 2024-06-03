#include <stdafx.h>
#include "BuildMap.h"
#include "NdtExtLocalization_oru.h"
#include "FeatureMethod.h"
#include "TemplateMethod.h"
#include "gridmap.h"
#include "StampedAffine.h"

#include "probability_grid_range_data_inserter_2d.h"

///////////////////////////////////////////////////////////////////////////////

#define WRITE_AU10_JFF

namespace mapping {



CBuildMap::CBuildMap()
{

    pLocation = NULL;
}

bool CBuildMap::Create()
{
    Clear();

    methods_ = new CLocalizationMethods;
    if (methods_ == NULL)
        return false;
    CLocalizationMethod *method;
    // NDT方法

    method = new CNdtMethod;
    if(method==NULL)
        return false;

    method->Initialize();

    methods_->push_back(method);

    // 特征方法
    method = new CFeatureMethod;
    if (method == NULL)
        return false;
    methods_->push_back(method);

    // 模板方法
    method = new CTemplateMethod;
    if (method == NULL)
        return false;


    // 取得指向模板图的指针

    CStaticObjects *m_pStaticObjects = ((CTemplateMethod *)(method))->GetMap();
    if (m_pStaticObjects == NULL)
        return false;

    CStockedObjects *pStockedTemplates = new CStockedObjects;  //??????
    m_pStaticObjects->SetStockedObjects(pStockedTemplates);


    methods_->push_back(method);

    // Slam方法 dq 01.06
   /* method = new CSlamMethod;
    if (method == NULL)
        return false;
    methods_->push_back(method);*/

    // 在此设置CLocalizationRect类的静态成员methods_
    CLocalizationRect::SetLocalizationMethods(methods_);

    plan_ = new CLocalizationPlan;
    if (plan_ == NULL)
        return false;

    pLocation = (dynamic_cast<CNdtMethod*>(methods_->at(0)))->localization_;


    return true;
}

 bool CBuildMap::SetNdtSubmapParam(ndt_oru::CSubmapParam *param)
 {
     if (param != NULL && pLocation!=NULL)
     {
        pLocation->StartNewSubmap(param);

        //设置ndt定位矩形及参数
       // CRectangle r(-param->mapSizeX,param->mapSizeY,param->mapSizeX,-param->mapSizeY); //???
/*
        CRectangle r(-10000,-10000,-10000,-10000); //???

        CLocalizationRect *p = new CLocalizationRect(r);

        CNdtLocalizationParam *param = dynamic_cast<CNdtLocalizationParam*>(p->param_[0]);
        param->submapId = 0 ;          // 对应的NDT子图的ID号
        param->localMode = 0;         // NDT定位方式 (正常定位 -- 0   长廊定位 -- 1   局部定位 -- 2   单点模式 -- 3)
        param->enableSlam = false;       // 是否允许以SLAM方式进行定位
        param->maxSlamDist = 20;     // 允许以SLAM模式运动的最大距离
        param->threshNumX = 20;        // X方向的匹配个数阈值（个） 默认值 20
        param->threshNumY = 20;        // Y方向的匹配个数阈值（个） 默认值 20
        param->threshRatioX = 40;    // X方向的匹配效率阈值（%） 默认值 40.0
        param->threshRatioY = 40;    // Y方向的匹配效率阈值（%） 默认值 40.0
        param->disLimit = 10;        // 功能启用的距离限制（米） 默认值 10.0
       // param->N_ = ;                // 定位评价因子
        param->enableGridMatch = true;   //是否允许以栅格方式进行定位
        param->gridMatchType = FAST_MATCH;      //GRID定位类型
        param->fastMatchLineSearchWindow = 1;   //扩展定位XY方向匹配范围
        param->fastMatchAngleSearchWindow = PI/10; //扩展定位角度匹配范围

        plan_->push_back((CRectangle*)p);*/

     }

 }
void CBuildMap::SetScansParam(CScannerGroupParam *param)
{
    if (param != NULL && pLocation!=NULL)
    {
        pLocation->SetScannerGroupParam(param);
        m_pScannerGroupParam = param;
    }
}


 bool CBuildMap::TransformScanToCloud(const CScan &scan,
                                         ndt_oru::CStampedPointCloud* cloud_trans,bool bAddFarPoint)
 {
     if(m_pScannerGroupParam->size()<=0)
         return false;

     cloud_trans->Clear();
     cloud_trans->m_dwTimeStamp = scan.m_dwTimeStamp;
     float first_angle = scan.m_fStartAng;
     unsigned int num_pts = scan.m_nCount;
   //  short laser_id = pRawCloud->laser_id;
     float angular_increment = 0.0;
     if(num_pts > 0) {
         angular_increment = (scan.m_fEndAng - scan.m_fStartAng) / num_pts;
     }
    // if(num_pts > pRawCloud->distance.size()){
    //     return false;
    // }
  //   if(laser_id >= m_pScannerGroupParam->size()){
   //      return false;
  //   }

     double min_range =m_pScannerGroupParam->at(0).m_fMinRange;
     double max_range = m_pScannerGroupParam->at(0).m_fMaxRange;
     int iCriteritonThreshold = 254/*m_pScannerGroupParam->at(0).criteritonThreshold*/; //反光板识别强度门限 默认值(倍加福R2000:700；Hokuyo：7000；SickNano：254)
     Eigen::Vector3d pt;
     Eigen::Vector3d ptraw;

     float r = 0.0;
     float a = 0.0;
     unsigned short int intensity = 0.0;

     cloud_trans->reserve(num_pts);
     // 引入Z方向的噪声
     double varz = 0.05 / (double)INT_MAX;
     for (int i = 0 ;i < num_pts; i++){

         // 过虑掉不满足可视角度的点
        // if(!m_pScannerGroupParam.at(0).m_AppAngleRange.Contain(a)){
        //     r = 0.0;
        // }

         r = sqrt(scan.m_pPoints[i].x*scan.m_pPoints[i].x+scan.m_pPoints[i].y*scan.m_pPoints[i].y);



        // if(i<700 && i>690)
        // {
        //       std::cout<<"i = "<<i<<"r ="<<r<<std::endl;
        // }

         // 极径大于0，表示数据有效
         if(r >= 0.0 ){

             if(r<max_range || bAddFarPoint)
             {
                 pt(0) =  scan.m_pPoints[i].x;
                 pt(1) =  scan.m_pPoints[i].y;
     //            pt(2) = varz*rand();
                 pt(2) = 0;   // By Sam
                 cloud_trans->push_back(pt);
                 cloud_trans->vecIntensity.push_back(intensity);   //????
             }
         }
     }

     return true;
 }

#ifdef ONE_LASER
int CBuildMap::SaveProbGridMap(const string filename,map<int,mapping::CStepData> *pStepData,CPosture &ptLeftBottom,CPosture &ptRightTop,int frozenNodeNum)
{

     double resolution = 0.05;
     int result = 0;

     Eigen::Vector2d max ;

     max(0) = ptRightTop.x;
     max(1) = ptRightTop.y;

     int num_x = (ptRightTop.y - ptLeftBottom.y)/resolution;
     int num_y = (ptRightTop.x - ptLeftBottom.x)/resolution;


     std::unique_ptr<mapping::ProbabilityGrid> pProbMap;

     if(frozenNodeNum>0)
     {
         pProbMap = CreateProbabilityGridFromFile(filename);
         if(pProbMap==nullptr)
             return false;
     }
     else
         pProbMap = common::make_unique<mapping::ProbabilityGrid>(MapLimits(resolution, max.x(),max.y(),CellLimits(num_x,num_y)));


     //ProbabilityGrid probmap(MapLimits(resolution, max.x(),max.y(),CellLimits(num_x,num_y)));

     proto::ProbabilityGridRangeDataInserterOptions2D options;
     options.hit_probability = 0.550000 ;
     options.miss_probability = 0.49000 ;
     options.insert_free_space = true;

     mapping::ProbabilityGridRangeDataInserter2D range_data_inserter_(options);

    // resolution = 0.1;
    // num_x = (ptRightTop.y - ptLeftBottom.y)/resolution;
   //  num_y = (ptRightTop.x - ptLeftBottom.x)/resolution;
   //  ProbabilityGrid fastmap(MapLimits(resolution, max.x(),max.y(),CellLimits(num_x,num_y)));

    std::string probfilename;
    map<int,mapping::CStepData>::iterator iter;

    int step = 0;
    for(iter=pStepData->begin();iter!=pStepData->end();iter++)
    {
         step++;

         //std::cout<<"step = "<<step<<std::endl;

         if(step<frozenNodeNum)
             continue;

        Eigen::Affine3d estimatepos = PostureToAffine(iter->second.m_pstRobot);
        ndt_oru::CStampedPointCloud cloud;
        //TransformRawCloudToScan ( pstRobot, Param, pRawScans.at(i), scan);
        TransformScanToCloud(iter->second.m_scan,&cloud,true);
        // 将点云变换到机器人参考系内
        if(m_pScannerGroupParam->size()>0)
        {
              // std::cout<<" m_pScannerGroupParam.size()"<<std::endl;
            CLaserScannerParam &ScannerParam = m_pScannerGroupParam->at(0);
            Eigen::Affine3d sensor_pose = PostureToAffine(ScannerParam.m_pst);
            ndt_oru::transformPointCloudInPlace(sensor_pose, cloud);
            ndt_oru::CStampedPointCloud sensorpos;
            Eigen::Vector3d pt(0,0,0);

            sensorpos.push_back(pt);
            ndt_oru::transformPointCloudInPlace(sensor_pose, sensorpos);

            Pose xyt;
            xyt.x = iter->second.m_pstRobot.x;
            xyt.y = iter->second.m_pstRobot.y;
            xyt.theta = iter->second.m_pstRobot.fThita;
            double c = cos(xyt.theta), s = sin(xyt.theta);

            ///////////////////////////////////////////
            Pose sensorInMap;
            sensorInMap.x =  sensorpos[0](0)*c - sensorpos[0](1)*s + xyt.x;
            sensorInMap.y =  sensorpos[0](0)*s + sensorpos[0](1)*c + xyt.y;

            sensor::RangeData all_range_data;         
            const Eigen::Vector3f origin_in_local(sensorInMap.x,sensorInMap.y, 0);

            for (int i = 0; i < cloud.size(); ++i)
            {
                Pose pt;
                Pose res;

                pt.x = cloud[i](0);
                pt.y = cloud[i](1);

                res.x =  pt.x*c - pt.y*s + xyt.x;
                res.y = pt.x*s + pt.y*c + xyt.y;
                res.theta =  pt.theta;

                const Eigen::Vector3f hit_in_local(res.x,res.y, res.theta);
                const Eigen::Vector3f origin_in_local(sensorInMap.x,sensorInMap.y, 0);
                const Eigen::Vector3f delta = hit_in_local - origin_in_local;
                const float range = delta.norm();

                if (range >= m_pScannerGroupParam->at(0).m_fMinRange)
                {
                    if (range <= m_pScannerGroupParam->at(0).m_fMaxRange)
                        all_range_data.returns.push_back(hit_in_local);
                    else
                        all_range_data.misses.push_back(origin_in_local + 5.0 / range * delta);

                }
            }


            if(all_range_data.returns.size() >0)
            {
                Eigen::Vector3f tt(xyt.x,xyt.y,xyt.theta);
                all_range_data.origin = tt;
               // range_data_inserter_.Insert(all_range_data, &probmap);
                range_data_inserter_.Insert(all_range_data, pProbMap.get());

            }
        }
    }

  //  result = probmap.saveMap(filename);
    result = pProbMap->saveMap(filename);

     /////////////////save fast match gridmap

   // std::unique_ptr<ProbabilityGrid> ffmap = probmap.CreateLowerResolutionMap(2);
    std::unique_ptr<ProbabilityGrid> ffmap = pProbMap->CreateLowerResolutionMap(2);

    FILE *fp = fopen(WORK_PATH"Gridmap.map", "wb");
    if (fp == NULL)
        false;

    int submapnum = 1;
    if (fwrite(&submapnum, sizeof(int), 1, fp) != 1)
        return false;

    int branch_and_bound_depth = 3;
    proto::FastCorrelativeScanMatcherOptions2D fastmatchoption={0.5, 3.1415/9, branch_and_bound_depth};
    std::shared_ptr<scan_matching::FastCorrelativeScanMatcher2D> matcher;

    matcher= std::make_shared<scan_matching::FastCorrelativeScanMatcher2D>(*(ffmap.get()), fastmatchoption);

    //子图ID
    int submapID = 0;
    if (fwrite(&submapID, sizeof(int), 1, fp) != 1)
        return false;

    matcher->SaveBinary(fp);
    fclose(fp);
    //std::cout<<"save over   "<<std::endl;

     return result;


}

#else

int CBuildMap::SaveProbGridMap(const string filename,map<int,mapping::CStepData> *pStepData,CPosture &ptLeftBottom,CPosture &ptRightTop,int frozenNodeNum)
{
     double resolution = 0.05;
     int result = 0;

     Eigen::Vector2d max ;

     max(0) = ptRightTop.x;
     max(1) = ptRightTop.y;

     int num_x = (ptRightTop.y - ptLeftBottom.y)/resolution;
     int num_y = (ptRightTop.x - ptLeftBottom.x)/resolution;


    std::unique_ptr<mapping::ProbabilityGrid> pProbMaps[2];


     for(int m = 0; m<m_pScannerGroupParam->size();m++)
     {
         if(m>=2)
             break;
        // std::unique_ptr<mapping::ProbabilityGrid> pProbMap;

         if(frozenNodeNum>0 )
         {
             if(m==0)
             {
                 pProbMaps[m] = CreateProbabilityGridFromFile(filename);
                 if(pProbMaps[m]==nullptr)
                     return false;
             }
             if(m==1)
             {
                 pProbMaps[m] = CreateProbabilityGridFromFile("bottomProbMap.txt"); //???

                 std::cout<<"read bottomProbMap.txt over"<<std::endl;
                 if(pProbMaps[m]==nullptr)
                 {
                      std::cout<<"read bottomProbMap.txt nullptr"<<std::endl;
                     pProbMaps[m] = common::make_unique<mapping::ProbabilityGrid>(MapLimits(resolution, max.x(),max.y(),CellLimits(num_x,num_y)));

                 }
             }
         }
         else
             pProbMaps[m] = common::make_unique<mapping::ProbabilityGrid>(MapLimits(resolution, max.x(),max.y(),CellLimits(num_x,num_y)));


         proto::ProbabilityGridRangeDataInserterOptions2D options;
         options.hit_probability = 0.550000 ;
         options.miss_probability = 0.49000 ;
         options.insert_free_space = true;

         mapping::ProbabilityGridRangeDataInserter2D range_data_inserter_(options);


        map<int,mapping::CStepData>::iterator iter;

        int step = 0;
        for(iter=pStepData->begin();iter!=pStepData->end();iter++)
        {
             step++;

             std::cout<<"step = "<<step<<std::endl;

             if(step<frozenNodeNum)
                 continue;

              std::cout<<"frozenNodeNum = "<<step<<std::endl;

            Eigen::Affine3d estimatepos = PostureToAffine(iter->second.m_pstRobot);
            ndt_oru::CStampedPointCloud cloud;

            TransformScanToCloud(iter->second.m_scans.at(m),&cloud,true);

              std::cout<<"m_pScannerGroupParam->size() = "<<m_pScannerGroupParam->size()<<std::endl;
            // 将点云变换到机器人参考系内
            if(m_pScannerGroupParam->size()>0)
            {

                std::cout<<" m_pScannerGroupParam.size()"<<std::endl;
                CLaserScannerParam &ScannerParam = m_pScannerGroupParam->at(m);
                Eigen::Affine3d sensor_pose = PostureToAffine(ScannerParam.m_pst);
                ndt_oru::transformPointCloudInPlace(sensor_pose, cloud);
                ndt_oru::CStampedPointCloud sensorpos;
                Eigen::Vector3d pt(0,0,0);

                sensorpos.push_back(pt);
                ndt_oru::transformPointCloudInPlace(sensor_pose, sensorpos);

                Pose xyt;
                xyt.x = iter->second.m_pstRobot.x;
                xyt.y = iter->second.m_pstRobot.y;
                xyt.theta = iter->second.m_pstRobot.fThita;
                double c = cos(xyt.theta), s = sin(xyt.theta);

                ///////////////////////////////////////////
                Pose sensorInMap;
                sensorInMap.x =  sensorpos[0](0)*c - sensorpos[0](1)*s + xyt.x;
                sensorInMap.y =  sensorpos[0](0)*s + sensorpos[0](1)*c + xyt.y;

                sensor::RangeData all_range_data;
                const Eigen::Vector3f origin_in_local(sensorInMap.x,sensorInMap.y, 0);


                  std::cout<<"  cloud.size() "<<  cloud.size()<<std::endl;

                for (int i = 0; i < cloud.size(); ++i)
                {
                    Pose pt;
                    Pose res;

                    pt.x = cloud[i](0);
                    pt.y = cloud[i](1);

                    res.x =  pt.x*c - pt.y*s + xyt.x;
                    res.y = pt.x*s + pt.y*c + xyt.y;
                    res.theta =  pt.theta;

                    const Eigen::Vector3f hit_in_local(res.x,res.y, res.theta);
                    const Eigen::Vector3f origin_in_local(sensorInMap.x,sensorInMap.y, 0);
                    const Eigen::Vector3f delta = hit_in_local - origin_in_local;
                    const float range = delta.norm();

                    if (range >= m_pScannerGroupParam->at(m).m_fMinRange)
                    {
                        if (range <= m_pScannerGroupParam->at(m).m_fMaxRange)
                            all_range_data.returns.push_back(hit_in_local);
                        else
                            all_range_data.misses.push_back(origin_in_local + 5.0 / range * delta);

                    }
                }

                std::cout<<"  all_range_data.returns.size() "<<  all_range_data.returns.size()<<std::endl;

                if(all_range_data.returns.size() >0)
                {
                    Eigen::Vector3f tt(xyt.x,xyt.y,xyt.theta);
                    all_range_data.origin = tt;
                   // range_data_inserter_.Insert(all_range_data, &probmap);
                    range_data_inserter_.Insert(all_range_data, pProbMaps[m].get());

                }
            }
        }

        if(m==0)
        {
            result = pProbMaps[m]->saveMap(filename);
            /////////////////save fast match gridmap

          // std::unique_ptr<ProbabilityGrid> ffmap = probmap.CreateLowerResolutionMap(2);
           std::unique_ptr<ProbabilityGrid> ffmap = pProbMaps[m]->CreateLowerResolutionMap(2);

           FILE *fp = fopen(WORK_PATH"Gridmap.map", "wb");
           if (fp == NULL)
               false;

           int submapnum = 1;
           if (fwrite(&submapnum, sizeof(int), 1, fp) != 1)
               return false;

           int branch_and_bound_depth = 3;
           proto::FastCorrelativeScanMatcherOptions2D fastmatchoption={0.5, 3.1415/9, branch_and_bound_depth};
           std::shared_ptr<scan_matching::FastCorrelativeScanMatcher2D> matcher;

           matcher= std::make_shared<scan_matching::FastCorrelativeScanMatcher2D>(*(ffmap.get()), fastmatchoption);

           //子图ID
           int submapID = 0;
           if (fwrite(&submapID, sizeof(int), 1, fp) != 1)
               return false;

           matcher->SaveBinary(fp);
           fclose(fp);
           //std::cout<<"save over   "<<std::endl;

        }
        else if(m==1)
        {
             result = pProbMaps[m]->saveMap("bottomProbMap.txt");   //????
             pProbMaps[0]->MergePgm(std::move(pProbMaps[1]));
        }
     }

     return result;
}

#endif

bool CBuildMap::BuildNdtMap(map<int,mapping::CStepData> *pStepData)
{
    if(m_pScannerGroupParam->size()<=0)
         return false;
    if(pLocation==NULL)
         return false;

    map<int,mapping::CStepData>::iterator iter;

    int num = 0 ;

    for(iter=pStepData->begin();iter!=pStepData->end();iter++)
    {
        num++;
        if(num>2)
            break;

        std::cout<<"num = "<<num<<std::endl;


        Eigen::Affine3d estimatepos = PostureToAffine(iter->second.m_pstRobot);

        // std::cout<<" build map iter->second.m_pstRobot"<<iter->second.m_pstRobot.x<<std::endl;

        ndt_oru::CStampedPointCloud cloud;
        ndt_oru::CStampedPointCloud filtercloud;


#ifdef ONE_LASER
        TransformScanToCloud(iter->second.m_scan,&cloud,false);
#else
         TransformScanToCloud(iter->second.m_scans.at(0),&cloud,false);
#endif

        // 将点云变换到机器人参考系内
        if(m_pScannerGroupParam->size()>0)
        {
            CLaserScannerParam &ScannerParam = m_pScannerGroupParam->at(0);
            Eigen::Affine3d sensor_pose = PostureToAffine(ScannerParam.m_pst);
            ndt_oru::transformPointCloudInPlace(sensor_pose, cloud);
         }

         pLocation->pose_ = estimatepos;
         pLocation->FilterCloud(cloud, filtercloud);

         int status = pLocation->BuildMap(filtercloud, estimatepos, false);
    }
}

bool CBuildMap::SaveNdtMap()
{

     FILE *fp = fopen(WORK_PATH"FeatureMap.map", "wb");
     if (fp == NULL)
         return false;
 #ifdef WRITE_AU10_JFF

         char version_[8] = "3.0.1.5";
         char date_[6] = {0, 0, 0, 0, 0, 0};

         // 先读取文件的版本和日期
         if (fwrite(version_, sizeof(char), 8, fp) != 8 || fwrite(date_, sizeof(char), 6, fp) != 6)
         {
             fclose(fp);
             return false;
         }
 #endif

     if(SaveBinary(fp,WORK_PATH))
        fclose(fp);
 }

}

