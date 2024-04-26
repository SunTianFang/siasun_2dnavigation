#include "stdafx.h"
#include "NdtMethod.h"
#include "BaseOdometry.h"
#include "NdtExtLocalization_oru.h"
//#include "LocalizeFactory.h"


#include "LinuxSetting.h"

#include "Tools.h"
#include "RoboLocClnt.h"
#include "AutoOutPutBlackBox.h"
#include "blackboxhelper.hpp"
#include "ParameterObject.h"
#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif
extern bool readJffFormat;

///////////////////////////////////////////////////////////////////////////////
//   实现基于NDT的定位方法。

CNdtMethod::CNdtMethod()
{
    type_ = 0;
    m_bInitSlam = true;  // By Sam
    Create();
    corridor_dist = 0.0;
    corridor_firstTime = true;
}

CNdtMethod::~CNdtMethod()
{
    // 如果定位对象是自身生成的，需要在此释放
    if (localization_ != NULL)
        delete localization_;

    if (maps_ != NULL)
    {       
        delete maps_;          
    }
}

bool CNdtMethod::UnloadMap()
{
    //dq 9.2
/*
    if (maps_ != NULL)
    {
        delete maps_;
         maps_ = NULL;
    }
*/
    m_fastMatcher.UnloadMap();

    return true;

}

bool CNdtMethod::Create()
{
    maps_ = new ndt_oru::NDTMaps;
    if (maps_ == NULL)
    {
        return false;
    }

    localization_ = new ndt_oru::CNdtExtLocalization();
    if (localization_ == NULL)
        return false;

    return true;
}

//
//   取得当前所采用的NDT子图的编号。
//
int CNdtMethod::GetCurSubmapId()
{
    return localization_->m_curSubmapId;
}

bool CNdtMethod::Initialize()
{
    // 设置NDT地图

    localization_->SetMaps(maps_);

    if(maps_->size()>=1)
        localization_->SetMap(maps_->at(0));

    return true;
}

// By Sam Add: 20211213
void CNdtMethod::SetScannerGroupParam(CScannerGroupParam *pParam)
{

    localization_->SetLaserParams(pParam);

    //lishen
    m_pScannerGroupParam = pParam;
    m_fastMatcher.SetScannerGroupParam(pParam);
}

//
//   生成一个适用于本方法的定位参数块。
//
CLocalizationParam *CNdtMethod::CreateLocParam()
{
    return new CNdtLocalizationParam;
}

//
//   应用指定的定位参数。
//
bool CNdtMethod::ApplyParam(const CLocalizationParam *param)
{
    CNdtLocalizationParam *p = (CNdtLocalizationParam*)param;
    if (p == NULL)
        return false;

    param_ = *p;

    return true;
}

//
//   对应于该定位方法的定位流程。
//
bool CNdtMethod::LocalizeProc(int localMode, const Eigen::Affine3d &initPose,
                              const ndt_oru::CStampedPointCloud cloudIn,
                              Eigen::Affine3d &estimatePose)
{
    // By Sam Add: 20211213
    localization_->cloudAdjusted.Clear();
    localization_->cloudCopy.Clear();
    localization_->matcher2D.result.runTime = 0;

    // 将在CLocalizationManager中得到的初始位姿和点云复制到定位对象中
    // (Fix me: 此为临时措施!!将来应改进)
//    localization_->odomAdjusted = initPose;
    localization_->Tnow = initPose;
    CPosture initpos = AffineToPosture(initPose);
    localization_->cloudAdjusted = cloudIn;

//    std::cout<<"By yu Test :param_.submapId = "<<param_.submapId<<std::endl;
    std::cout << "By wt: initPose (" << initpos.x << ", " << initpos.y << ", " << initpos.fThita << ")" << std::endl;

    // 应用指定的NDT子图
    localization_->SetMapByID(param_.submapId);

    // 根据参数中的设置决定是否允许SLAM
    bool enableSlam = param_.enableSlam;
    bool enablegridMatch = param_.enableGridMatch;
    int  gridMatchType = param_.gridMatchType;
    float fastMatchLineSearchWindow = param_.fastMatchLineSearchWindow;
    float fastMatchAngleSearchWindow = param_.fastMatchAngleSearchWindow*PI/180.0;


//    // 实际进行定位，仅当定位函数返回值为0(定位失败)时，需要单独将结果位姿设置为初始位姿
//    if (localization_->LocalizationProc(estimatePose, enableSlam) == 0)
//    {
//        localization_->matcher2D.result.result_ = CMatchInfo::MATCH_FAIL;
//        localization_->matcher2D.result.pst_ = AffineToPosture(initPose);
//        return false;
//    }

    std::cout << "By Sam Test: LocalMode = " << param_.localMode << std::endl;

    NDTMatchInfo info_oru;
    NDTMatchInfo info_slam;


    long long int ndt_start_time0 = GetTickCount();
    // By Sam: Sam Add
    if (localization_->LocalizationProc(estimatePose, false))
    {
        // By Sam: Mean ndt has pose, but we not konw right or wrong
        info_oru = localization_->matchInfo;

        // By Sam test
        int local_point_cell_x = info_oru.lpx;
        int local_point_cell_y = info_oru.lpy;
        int local_point_cell_o = info_oru.lpo;
        int local_cell_x = info_oru.lx;
        int local_cell_y = info_oru.ly;
        int local_cell_o = info_oru.lo;

        float ratio_x = (float)(local_point_cell_x  + local_point_cell_o) / (local_cell_x + local_cell_o);//0324.增加o方向Cell
        float ratio_y = (float)(local_point_cell_y  + local_cell_o) / (local_cell_y + local_cell_o);
        float score = (float)(local_point_cell_x + local_point_cell_y + local_point_cell_o * 2) / (local_cell_x + local_cell_y + local_cell_o * 2);

        std::cout << "NDT Match info, lx = " << info_oru.lx <<
                     ", ly = " << info_oru.ly <<
                     ", lo = " << info_oru.lo <<
                     ", lpx = " << info_oru.lpx <<
                     ", lpy = " << info_oru.lpy <<
                     ", lpo = " << info_oru.lpo <<
                     ", mx = " << info_oru.mx <<
                     ", my = " << info_oru.my <<
                     ", mo = " << info_oru.mo << std::endl;

        std::cout << "NDT Match info, RatioX = " << ratio_x << ", RatioY = " << ratio_y << ", score = " << score << std::endl;

        bool match_flag = info_oru.isOK_2(param_.threshNumX, param_.threshNumY, param_.threshRatioX, param_.threshRatioY);
//        std::cout << "By wt: estimatePose (" << estipos.x << ", " << estipos.y << ", " << estipos.fThita << ")" << std::endl;
        if (match_flag)
        {
            corridor_dist = 0;

            localization_->matcher2D.result.result_ = CMatchInfo::MATCH_OK;
            localization_->matcher2D.result.pst_ = AffineToPosture(estimatePose);

            m_bInitSlam = true;
            std::cout << "By Sam: Local success !!!!" << std::endl;
            long long int ndt_end_time0= GetTickCount();
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "NDT Local Success!!Loc Time = " , static_cast<int>(ndt_end_time0 - ndt_start_time0)," NDT_Match info : ",
                      info_oru.lx,  ",", info_oru.ly, ",", info_oru.lo,
                      " | ", info_oru.lpx, ",", info_oru.lpy, ",", info_oru.lpo,
                      " | ", info_oru.mx, ",", info_oru.my, ",", info_oru.mo,
                      " | ", ratio_x," ", ratio_y," ", score);
#endif
            return true;
        }
        else
        {
            long long int ndt_end_time1= GetTickCount();
            long long int t1 = ndt_end_time1 - ndt_start_time0;
            localization_->matcher2D.result.runTime += t1;

#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "NDT Local Failed !!Loc time = ",static_cast<int>(ndt_end_time1 - ndt_start_time0),
                      " InitPos : ", initpos.x, "," , initpos.y, "," , initpos.fThita,
                      " NDT_Match info : ", info_oru.lx,  ",", info_oru.ly, ",", info_oru.lo,
                      " | ", info_oru.lpx, ",", info_oru.lpy, ",", info_oru.lpo,
                      " | ", info_oru.mx, ",", info_oru.my, ",", info_oru.mo,
                      " | ", ratio_x," ",ratio_y," ", score);
#endif
        }
        if( (!match_flag) && 1 == param_.localMode)
        {
            std::cout<<"By Yu  : In Corridor !!!!"<<std::endl;
            CPosture onePos;
            CPosture ndtPos;
            onePos = AffineToPosture(initPose);

            std::cout << "By Sam: In corridor mode, onePos (" << onePos.x << ", " << onePos.y << ")" << std::endl;
            std::cout << "By Sam: In corridor mode, ndtPos (" << ndtPos.x << ", " << ndtPos.y << ")" << std::endl;

#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "In corridor, corridor initPose (", onePos.x, "," ,onePos.y, ")" );
#endif

            if(CorridorJudge(onePos.x , onePos.y) && info_oru.inScope && ((info_oru.mx > param_.threshNumX -1) ||  (info_oru.my > param_.threshNumY - 1) ))
            {

                //By yu 20220805
//                long long int tmstart= GetTickCount();
//                CPosture odomTrans_two;
//                auto pOdometry = BaseOdomSingleton::GetInstance();

//                pOdometry->GetLocalOdomTrans(ndt_start_time0, tmstart, odomTrans_two);

//                Eigen::Affine3d TFromScan_two = PostureToAffine(odomTrans_two);
//                estimatePose = estimatePose * TFromScan_two;
                ndtPos = AffineToPosture(estimatePose);
//                std::cout << "By Sam: In corridor mode, estimatePose (" << ndtPos.x << ", " << ndtPos.y << ")" << std::endl;

//                Eigen::Affine3d Corridor_pose = initPose * TFromScan_two;
//                onePos = AffineToPosture(Corridor_pose);

                if(info_oru.lpx > info_oru.lpy)
                {
                    ndtPos.x = onePos.x;
                    estimatePose = PostureToAffine(ndtPos);
                }
                else
                {
                    ndtPos.y = onePos.y;
                    estimatePose = PostureToAffine(ndtPos);
                }


                std::cout << "By Sam: In corridor mode, estmatePose (" << ndtPos.x << ", " << ndtPos.y << ")" << std::endl;
                localization_->matcher2D.result.result_ = CMatchInfo::MATCH_CORRIDOR_OK;
                localization_->matcher2D.result.pst_ = AffineToPosture(estimatePose);
                m_bInitSlam = true;

                // By Sam Add 20220223;
                //Bu yu change: In corridor ,set uG =100,uN = 100;
                localization_->mp_Informer->mx = 100;
                localization_->mp_Informer->my = 100;
                localization_->mp_Informer->lpx = 100;
                localization_->mp_Informer->lpy = 100;

                return true;
            }
            else{
                localization_->matcher2D.result.result_ = CMatchInfo::MATCH_FAIL;
                localization_->matcher2D.result.pst_ = AffineToPosture(initPose);
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "In corridor, but do not meet corridor standards");
#endif
                return false;
            }
        }
//        if((!match_flag) && 1 == param_.localMode)
//        {
//            CPosture onePos;
//            CPosture ndtPos;
//            onePos = AffineToPosture(initPose);
//            ndtPos = AffineToPosture(estimatePose);

//            std::cout << "By Sam: In corridor mode, estmatePose (" << onePos.x << ", " << onePos.y << ")" << std::endl;
//            std::cout << "By Sam: In corridor mode, localPose (" << ndtPos.x << ", " << ndtPos.y << ")" << std::endl;

//            if(info_oru.lpx > info_oru.lpy)
//            {
//                ndtPos.x = onePos.x;
//                estimatePose = PostureToAffine(ndtPos);
//            }
//            else
//            {
//                ndtPos.y = onePos.y;
//                estimatePose = PostureToAffine(ndtPos);
//            }

//            localization_->matcher2D.result.result_ = CMatchInfo::MATCH_CORRIDOR_OK;
//            localization_->matcher2D.result.pst_ = AffineToPosture(estimatePose);
//            m_bInitSlam = true;
//            std::cout << "By Sam: In corridor mode, corridorPose (" << ndtPos.x << ", " << ndtPos.y << ")" << std::endl;
//#ifdef USE_BLACK_BOX
//        FILE_BlackBox(LocBox, "In corridor mode !! Finally using corridor! CorridorPose : ", ndtPos.x, ",", ndtPos.y);
//#endif
////            // By Sam Add 20220223
//            localization_->mp_Informer->mx = localization_->matchInfo.lx;
//            localization_->mp_Informer->my = localization_->matchInfo.ly;

//            return true;
//        }
        if((!match_flag) && 3 == param_.localMode) //single_feature mode  wt_add_20221201
        {
            CPosture initPos;
            CPosture estiPos;
            initPos = AffineToPosture(initPose);
            estiPos = AffineToPosture(estimatePose);

            std::cout << "By wt: In single_feature mode, initPose (" << initPos.x << ", " << initPos.y << ")" << std::endl;
            std::cout << "By wt: In single_feature mode, estimatePose1 (" << estiPos.x << ", " << estiPos.y << ")" << std::endl;

            if(info_oru.lpx > info_oru.lpy)
            {
                estiPos.x = initPos.x + 1000;
                estimatePose = PostureToAffine(estiPos);
            }
            else
            {
                estiPos.y = initPos.y + 1000;
                estimatePose = PostureToAffine(estiPos);
            }

            std::cout << "By wt: In single_feature mode, estimatePose2 (" << estiPos.x << ", " << estiPos.y << ")" << std::endl;


            localization_->matcher2D.result.result_ = CMatchInfo::MATCH_TO_SINGLEFEATURE;
            return false;
        }
    }
    else
    {
        long long int ndt_end_time2= GetTickCount();
        long long int t1 = ndt_end_time2 - ndt_start_time0;
        localization_->matcher2D.result.runTime += t1;

#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "NDT Local Failed !! Loc Time = ",static_cast<int>(ndt_end_time2 - ndt_start_time0));
#endif
    }

    // By LiShen
    if(enablegridMatch  &&  gridMatchType == FAST_MATCH)
    {
        long long int FastMatch_start_time = GetTickCount();
        std::cout<< "In FastMatchModle "<< std::endl;

        //ndt定位失败时，采用fastmatch 定位, 定位结果作为ndt初值
        Eigen::Affine3d fastResultPose;

        m_fastMatcher.SetParam(fastMatchLineSearchWindow,fastMatchAngleSearchWindow);
        m_fastMatcher.SetMapID(param_.submapId - 1); //By yu. Make them correspond. ndt_submap min = 1,fastmach_submap min = 0.
        bool res = m_fastMatcher.LocalizeProc(gridMatchType, initPose, cloudIn,fastResultPose);
        long long int ndt_start_time1 = GetTickCount();
        if(res)
        {
		    localization_->Tnow = fastResultPose;
            if (localization_->LocalizationProc(estimatePose, false))
            {
                info_oru = localization_->matchInfo;

                int local_point_cell_x = info_oru.lpx;
                int local_point_cell_y = info_oru.lpy;
                int local_cell_x = info_oru.lx;
                int local_cell_y = info_oru.ly;

                float ratio_x = (float)local_point_cell_x / local_cell_x;
                float ratio_y = (float)local_point_cell_y / local_cell_y;
                float score = (float)(local_point_cell_x + local_point_cell_y) / (local_cell_x + local_cell_y);

                std::cout << "After FastMatch, lx = " << info_oru.lx <<
                             ", ly = " << info_oru.ly <<
                             ", lo = " << info_oru.lo <<
                             ", lpx = " << info_oru.lpx <<
                             ", lpy = " << info_oru.lpy <<
                             ", lpo = " << info_oru.lpo <<
                             ", mx = " << info_oru.mx <<
                             ", my = " << info_oru.my <<
                             ", mo = " << info_oru.mo << std::endl;

                std::cout << "After FastMatch, RatioX = " << ratio_x << ", RatioY = " << ratio_y << ", score = " << score << std::endl;

                if (info_oru.isOK_2(param_.threshNumX, param_.threshNumY, param_.threshRatioX, param_.threshRatioY))
                {
                    localization_->matcher2D.result.result_ = CMatchInfo::MATCH_OK;
                    localization_->matcher2D.result.pst_ = AffineToPosture(estimatePose);
                    m_bInitSlam = true;
                    std::cout << "FastMatch Loc success.After FastMatch, NDT Local success !!!!" << std::endl;
                    long long int ndt_end_time3= GetTickCount();

#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "After FastMatch, NDT Local success!! NDT Loc Time = ", static_cast<int>(ndt_end_time3 - ndt_start_time1),
                      " NDT_Match info : ", info_oru.lx,  ",", info_oru.ly, ",", info_oru.lo,
                      " | ", info_oru.lpx, ",", info_oru.lpy, ",", info_oru.lpo,
                      " | ", info_oru.mx, ",", info_oru.my, ",", info_oru.mo,
                      " | ", ratio_x, ratio_y, score);
#endif
                    return true;
                }
                else
                {
                    long long int ndt_end_time4= GetTickCount();
                    long long int t1 = ndt_end_time4 - ndt_start_time1;
                    localization_->matcher2D.result.runTime += t1;
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "After FastMatch, NDT Failed (Matching_rate not reach)!! NDT Loc Time = " ,static_cast<int>(ndt_end_time4 - ndt_start_time1),
                      " NDT_Match info : ", info_oru.lx,  ",", info_oru.ly, ",", info_oru.lo,
                      " | ", info_oru.lpx, ",", info_oru.lpy, ",", info_oru.lpo,
                      " | ", info_oru.mx, ",", info_oru.my, ",", info_oru.mo,
                      " | ", ratio_x, ",",ratio_y, ",", score);
#endif
                     localization_->Tnow = initPose;
                }
            }
            else
            {
                long long int ndt_end_time5 = GetTickCount();
                long long int t1 = ndt_end_time5 - ndt_start_time1;
                localization_->matcher2D.result.runTime += t1;
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "After FastMatch, NDT Failed!! NDT Loc Time = ",static_cast<int>(ndt_end_time5 - ndt_start_time1));
#endif
            }
        }
        else
        {
            long long int ndt_end_time6= GetTickCount();
            long long int t1 = ndt_end_time6 - ndt_start_time1;
            localization_->matcher2D.result.runTime += t1;
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "FastMatch Local Failed !! NDT Loc Time = ",static_cast<int>(ndt_end_time6 - ndt_start_time1));
#endif
        }

    }

    if (enableSlam)
    {
//        std::cout << "By Yu Test: initPose (" << initPose.translation().x() <<
//                     ", " << initPose.translation().y() <<
//                     ", " << (initPose.rotation().eulerAngles(0, 1, 2)(2) / 3.14) * 180 <<
//                     ")" << std::endl;
//        std::cout << "By Yu Test: estimatePose_0 (" << estimatePose.translation().x() <<
//                     ", " << estimatePose.translation().y() <<
//                     ", " << (estimatePose.rotation().eulerAngles(0, 1, 2)(2) / 3.14) * 180 <<
//                     ")" << std::endl;

        localization_->Tnow = initPose;
        long long int slam_start_time = GetTickCount();
        if(localization_->SlamProc(estimatePose, m_bInitSlam))
        {
            //By yu. 使第一帧slam一定成功
            if(m_bInitSlam)
            {
                m_bInitSlam = false;
                localization_->mp_Informer->mx = 100;
                localization_->mp_Informer->my = 100;
                localization_->mp_Informer->lpx = 100;
                localization_->mp_Informer->lpy = 100;
                localization_->matcher2D.result.result_ = CMatchInfo::MATCH_SLAM_OK;
                localization_->matcher2D.result.pst_ = AffineToPosture(estimatePose);
                return true;
            }

            info_slam = localization_->matchInfo;

/*            int local_point_cell_x = info_slam.lpx;
            int local_point_cell_y = info_slam.lpy;
            int local_cell_x = info_slam.lx;
            int local_cell_y = info_slam.ly;

            float ratio_x = (float)local_point_cell_x / local_cell_x;
            float ratio_y = (float)local_point_cell_y / local_cell_y;
            float score = (float)(local_point_cell_x + local_point_cell_y) / (local_cell_x + local_cell_y)*/;

#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox,  "SLAM_Match info : ", info_slam.lx,  ",", info_slam.ly, ",", info_slam.lo,
                      " | ", info_slam.lpx, ",", info_slam.lpy, ",", info_slam.lpo,
                      " | ", info_slam.mx, ",", info_slam.my, ",", info_slam.mo);
#endif
            float matchRate = 0.5; //By yu : slam matchRate. Need to do setting in Mapping.
//            bool Slam_flag = info_slam.isOK_2(param_.threshNumX, param_.threshNumY, param_.threshRatioX, param_.threshRatioY);
            bool Slam_flag = info_slam.isOK_3(param_.threshNumX, param_.threshNumY, matchRate);

            long long int slam_end_time0 = GetTickCount();
            if (Slam_flag)
            {
                localization_->matcher2D.result.result_ = CMatchInfo::MATCH_SLAM_OK;
                localization_->matcher2D.result.pst_ = AffineToPosture(estimatePose);
                std::cout << "By Sam: Slam success !!!!" << std::endl;
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "Slam Local Success !! Slam Loc Time = ",static_cast<int>(slam_end_time0 - slam_start_time));
#endif
                    return true;
            }
            else
            {
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "Slam Local Failed (Matching_rate not reach)!! Slam Loc Time = ",static_cast<int>(slam_end_time0 - slam_start_time));
#endif
            }
        }
        else
        {
            long long int slam_end_time1 = GetTickCount();
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "Slam Local Failed !! Slam Loc Time = ",static_cast<int>(slam_end_time1 - slam_start_time));
#endif
        }
    }



    localization_->matcher2D.result.result_ = CMatchInfo::MATCH_FAIL;
    localization_->matcher2D.result.pst_ = AffineToPosture(initPose);

    std::cout << "  By Sam: Local & Slam all failed !!!!" << std::endl;
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "NDT && FastMatch && Slam Local all Failed !!! ");
#endif
    return false;
}

//
//   取得匹配数据。
//
CMatchInfo *CNdtMethod::GetMatchInfo()
{
//    float percent = (float)localization_->matcher2D.result.countGood / (float)localization_->matcher2D.result.countSourceNDT * 100;
//    localization_->matcher2D.result.matchRatio_ = percent;
//    localization_->matcher2D.result.matchNum_ = localization_->matcher2D.result.countGood;

//    // By Sam: Add
    int local_point_cell_x = localization_->mp_Informer->lpx;
    int local_point_cell_y = localization_->mp_Informer->lpy;
    int local_cell_x = localization_->mp_Informer->lx;
    int local_cell_y = localization_->mp_Informer->ly;
    //By yu Add.
    int local_point_cell_o = localization_->mp_Informer->lpo;
    int local_cell_o = localization_->mp_Informer->lo;
//    float ratio_x = (float)local_point_cell_x / local_cell_x;
//    float ratio_y = (float)local_point_cell_y / local_cell_y;
    float score = (float)(local_point_cell_x + local_point_cell_y + 2 *local_point_cell_o) / (local_cell_x + local_cell_y);
    localization_->matcher2D.result.matchRatio_ = score * 100;
    localization_->matcher2D.result.matchNum_ = localization_->mp_Informer->mx + localization_->mp_Informer->my + 2 * localization_->mp_Informer->mo;//Change By yu.uN should contain "mo"
//#ifdef USE_BLACK_BOX
//        FILE_BlackBox(LocBox, "report_info = " , static_cast<int>(ndt_end_time0 - ndt_start_time0)," NDT_Match info : ",
//                      info_oru.lx,  ",", info_oru.ly, ",", info_oru.lo,
//                      " | ", info_oru.lpx, ",", info_oru.lpy, ",", info_oru.lpo,
//                      " | ", info_oru.mx, ",", info_oru.my, ",", info_oru.mo,
//                      " | ", ratio_x, ratio_y, score);
//#endif

    return &(localization_->matcher2D.result);
}

//
//   对定位质量进行评估。
//
bool CNdtMethod::EvaluateQuality(float &score)
{
    // By Sam: Add
    int local_point_cell_x = localization_->mp_Informer->lpx;
    int local_point_cell_y = localization_->mp_Informer->lpy;
    int local_cell_x = localization_->mp_Informer->lx;
    int local_cell_y = localization_->mp_Informer->ly;

    float ratio_x = (float)local_point_cell_x / local_cell_x;
    float ratio_y = (float)local_point_cell_y / local_cell_y;
    score = (float)(local_point_cell_x + local_point_cell_y) / (local_cell_x + local_cell_y);

    std::cout << "In EvaluateQuality, lx = " << localization_->mp_Informer->lx <<
                 ", ly = " << localization_->mp_Informer->ly <<
                 ", lo = " << localization_->mp_Informer->lo <<
                 ", lpx = " << localization_->mp_Informer->lpx <<
                 ", lpy = " << localization_->mp_Informer->lpy <<
                 ", lpo = " << localization_->mp_Informer->lpo <<
                 ", mx = " << localization_->mp_Informer->mx <<
                 ", my = " << localization_->mp_Informer->my <<
                 ", mo = " << localization_->mp_Informer->mo << std::endl;

    std::cout << "In EvaluateQuality, RatioX = " << ratio_x << ", RatioY = " << ratio_y << ", score = " << score << std::endl;

    return true;
}

//
//   从二进制文件装入NDT地图。
//
bool CNdtMethod::LoadBinary(FILE *fp, string filename,int floor, bool bChangeFloor)
{
    if (!maps_->LoadBinary(fp))
        return false;

    if (!readJffFormat)
    {
        int dummy;
        fread(&dummy, sizeof(int), 1, fp);
    }

    // 设置NDT地图
    localization_->SetMaps(maps_);
    localization_->SetMap(maps_->at(0));
        m_fastMatcher.LoadBinary(fp,filename,floor,bChangeFloor);
  

    return true;
}

//
//   将NDT地图写入二进制文件。
//
bool CNdtMethod::SaveBinary(FILE *fp, string filename)
{
    if (!localization_->GetMaps()->SaveBinary(fp))
        return false;

    int dummy = 0;
    fwrite(&dummy, sizeof(int), 1, fp);


   /* FILE *gridfp = fopen(WORK_PATH"Gridmap.map", "wb");
    if (gridfp == NULL)
        false;

     m_fastMatcher.CreateGridMap(maps_,gridfp);*/

    return true;
}

bool CNdtMethod::CorridorJudge(float x, float y)
{
            // 计算位移变化量
            unsigned long long corridor_timeNow = GetTickCount();
               //corridor_firstTime = true;
            if(corridor_firstTime){
                corridor_timeBefore = corridor_timeNow;
                corridor_firstTime = false;
            }

            double cycle_dist = 0.0;
            long long cycle_time = static_cast<long long>(corridor_timeNow - corridor_timeBefore);
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

            corridor_timeBefore = corridor_timeNow;

    #if defined USE_BLACK_BOX
        FILE_BlackBox(LocBox, "TEST : In corridor, corridor_dist = ", corridor_dist, ", cycle_dist = ", cycle_dist,
                      ", cycle_time = ", (int)cycle_time, ", m_corridorDistanceLimit = ", m_corridorDistanceLimit);
    #endif
            //m_corridorDistanceLimit. Need to do setting in Mapping.
            m_corridorDistanceLimit = 150;
//            std::cout<<"cycle_dist = "<<cycle_dist<<std::endl;
//            std::cout<<"corridor_dist = " <<corridor_dist<<std::endl;
            if(corridor_dist < m_corridorDistanceLimit)
            {
                corridor_dist += cycle_dist;

                #if defined USE_BLACK_BOX
                    FILE_BlackBox(LocBox, "In corridor !!!!!!!!");
                #endif

                return true;
            }
            else
            {
                #if defined USE_BLACK_BOX
                    FILE_BlackBox(LocBox, "In corridor, but corridor_dist = ", corridor_dist, ", out CORRIDOR_DIS_LIMIT!!!!!!!!");
                #endif
            }
        return false;

//    return true;
}

// By Sam 
bool CNdtMethod::ReSetMethod()
{
    return true;
}
