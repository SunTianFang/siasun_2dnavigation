#include "stdafx.h"
#include <stdlib.h>
#include <math.h>
#include "FeatureLocalization.h"
#include "MatchTabSet.h"
#include "Scan.h"
#include "Project.h"
#include "blackboxhelper.hpp"

#define VEL_TIME_OUT_CYCLES          3

#define FEATURE_MATCH_SUCCESS                      1
#define FULL_MAP_ERROR_NO_MATCH                   -1
#define FEATURE_MATCH_POINTS_NOT_ENOUGH           -2
#define FEATURE_MATCH_MATRIX_ERROR                -3
#define FULL_MAP_ERROR_MATCH_TIME_OUT             -4
#define FEATURE_LOC_OVER_RANGE                    -5
#define FEATURE_MATCH_UNKOWN_ERROR                -10

#define DistToLinePoint                           0.250 //250

#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

//
//    给定激光头姿态和一条直线段，计算对应于线段两个端点的扫描角(假定以逆时针方向扫描)。
//
void FindScanAngles(CPosture& pstScanner, CLine& Line, CAngle& ang1, CAngle& ang2)
{
    // 取得激光头的位置和方向角度
    CPnt ptScanner = pstScanner.GetPntObject();
    CAngle angScanner = pstScanner.GetAngle();

    // 构造从激光头到线段起点的直线，并计算它与激光头姿态角的夹角
    CLine lnToStartPoint(ptScanner, Line.m_ptStart);
    CAngle angStartLine = lnToStartPoint.m_angSlant - angScanner;

    // 构造从激光头到线段终点的直线，并计算它与激光头姿态角的夹角
    CLine lnToEndPoint(ptScanner, Line.m_ptEnd);
    CAngle angEndLine = lnToEndPoint.m_angSlant - angScanner;

    // 假定激光头按照逆时针方向旋转，判断激光光线是先扫到直线的起点还是终点
    CAngle angDiff = angEndLine - angStartLine;

    // 如果上述角度差在I, II象限，说明先扫到m_ptStart
    if (angDiff.m_fRad < PI)
    {
        ang1 = angStartLine;
        ang2 = angEndLine;
    }

    // 否则说明先扫到m_ptEnd
    else
    {
        ang1 = angEndLine;
        ang2 = angStartLine;
    }
}

CFeatureLocalization::CFeatureLocalization() : m_LeastSquareMethod(40, 4)
{
    m_pWorldLayer = NULL;
    m_pWorldLines = NULL;
    m_pRefWorldLines = NULL;
    m_fRatio = 60.0f;
    m_bVelIsNew = false;           // 尚未获得激光头速度

    m_iLegFalseNum = 0;  // By Sam For LegMethod
    m_lastLegsCenter = CPosture(0, 0, 0);
    m_iLegTimes = 0;
    m_iFaileTimes = 0;
}


//
//   
//
bool CFeatureLocalization::ReSetLegPose()
{
    m_lastLegsCenter = CPosture(0, 0, 0);
    m_iLegTimes = 0;
    m_RecentPoses.clear();
    m_iLegPoseCount = 0;
    m_iFaileTimes = 0;

    return true;
}


//
//   设置特征图。
//
bool CFeatureLocalization::SetFeatureMap(CFeatureMap* pFeatureMap)
{
    m_pWorldLayer = pFeatureMap->GetPointFeatures();
    m_pWorldLines = pFeatureMap->GetLineFeatures();

    return true;
}

//   设置激光扫描器参数。
//
bool CFeatureLocalization::SetScannerParam(float fStartAng, float fEndAng, float fAngReso, float fMaxRange)
{
    if (!CScanMatcher::SetScannerParam(fStartAng, fEndAng, fAngReso, fMaxRange))
        return false;

    //	m_nWorkMode = FULL_MAP_MODE;      // 起始工作状态为“全局映射”

    m_bVelIsNew = false;              // 尚未获得激光头速度
    return true;
}

//
//   设置初始姿态。
//
void CFeatureLocalization::SetInitPosture(const CPosture& pst, bool bRough)
{
    CScanMatcher::SetInitPosture(pst, bRough);
    //d_FirstMatchTick = 0;

}

//
//   根据里程计设置姿态。
//
void CFeatureLocalization::SetOdometricPosture(const CPosture& pst, const int workmode)
{
    CScanMatcher::SetOdometricPosture(pst);
    m_pstOdRe = m_pstOld = m_pstCur = m_pstEstimate = pst;

    m_nWorkMode = workmode;
}

//
//   主定位流程。
//
int CFeatureLocalization::LocalizationDistribution(CPosture* pResultPosture)
{
    int nStatus;
    switch(m_nWorkMode)
    {
        case FULL_MAP_MODE:
            {
                nStatus = FullMapLocalization(pResultPosture);
                if(nStatus == FEATURE_MATCH_SUCCESS)
                    m_nWorkMode = QUICK_MAP_MODE;
            }
            break;
       case QUICK_MAP_MODE:
            {
                nStatus = QuickMapLocalization(pResultPosture);
            }
            break;
        default:
            nStatus = FEATURE_MATCH_UNKOWN_ERROR;
            break;
    }
    return nStatus;
}

//
//   货架腿定位流程。
//
int CFeatureLocalization::LocalizationByLeg(CPosture* pResultPosture)
{
    int nStatus = -1;
    CPosture Center(0, 0, 0);   //(0.7, 0, 0)(0.4, 0, 0)
    CPosture InvCenter(0, 0, 0);    //((-0.66, 0, 0)(-0.96, 0, 0))
    *pResultPosture = CPosture(0, 0, 0);

    std::cout<<"test by yu : m_PointMatchList ="<<m_PointMatchList.GetCount()<<std::endl;
    if(m_PointMatchList.GetCount() == 1)
    {
        // 计算货架在激光坐标系中的角度和位置
        // By Sam 注意: SICK的点云是从第三象限开始逆时针转到第二象限
        //              计算货架腿角度需要根据激光点云的起始点及象限决定
        //              以SICK激光为例，legPair.m_ptLocal为激光头正方向的右侧点(四象限)，legPair.m_ptWorld为激光头正方向的左侧点(一象限)

        CPointMatchPair legPair = m_PointMatchList.m_PointPair[0];

        std::cout << "By Sam: left = ( " << legPair.m_ptWorld.x << ", " << legPair.m_ptWorld.y <<
                     "), right = (" << legPair.m_ptLocal.x << ", " << legPair.m_ptLocal.y << ")" << std::endl;

        std::cout << "By yu: left = ( " << legPair.m_ptWorld.r * cos(legPair.m_ptWorld.a) << ", " << legPair.m_ptWorld.r * sin(legPair.m_ptWorld.a) <<
                     "), right = (" << legPair.m_ptLocal.r * cos(legPair.m_ptLocal.a) << ", " << legPair.m_ptLocal.r * sin(legPair.m_ptLocal.a) << ")" << std::endl;




//        if (legPair.m_ptLocal.y > 0/* && legPair.m_ptWorld.y > 0*/)
//        {
////            return -1;  // 超限报错，暂时屏蔽掉
//            std::cout << "By Sam: ERROR ERROR, all goods leg in First quadrant !!!" << std::endl;
//        }

//        if (legPair.m_ptWorld.y < 0 /*&& legPair.m_ptLocal.y < 0*/)
//        {
////            return -1;  // 超限报错，暂时屏蔽掉
//            std::cout << "By Sam: ERROR ERROR, all goods leg in Fourth quadrant !!!" << std::endl;
//        }


        float xx, yy;
        CPosture goodsCen;
        xx = (legPair.m_ptLocal.x + legPair.m_ptWorld.x) / 2;
        yy = (legPair.m_ptLocal.y + legPair.m_ptWorld.y) / 2;
        goodsCen.x = xx;
        goodsCen.y = yy;
        std::cout << "By Sam: Goods center is (" << goodsCen.x << ", " << goodsCen.y <<")" << std::endl;

        float angle_temp = 0;
        if (yy == 0.0f)
            angle_temp = 0;
        else
            angle_temp = atan(yy / xx);

        //车的位置和左右反光板，构成三角形。车体坐标系下计算垂直于左右反光板连线的垂点。
//        CPnt Vertical_point;
        float detax ;
        float detay ;
        detax = legPair.m_ptLocal.x - legPair.m_ptWorld.x;
        detay = legPair.m_ptLocal.y - legPair.m_ptWorld.y;


        float K = 0;
        float k = 0;
        float  vertical_deviation= 0; //纵向偏差
        float  lateral_deviation= 0;//左右偏差

        if (detax == 0.0f){
            lateral_deviation = goodsCen.y;
            vertical_deviation = goodsCen.x;
        }else
        {
            k = detay/detax;
            K = -1/k;
//            std::cout << "By yu: K&k is (" << K << ", " << k<< ")" << std::endl;
            lateral_deviation =fabs( (goodsCen.y - K * goodsCen.x) / sqrt(K * K + 1));
            vertical_deviation = (goodsCen.y - k * goodsCen.x) / sqrt(k * k + 1);
        }


        if(legPair.m_ptWorld.r < legPair.m_ptLocal.r)
            lateral_deviation= -lateral_deviation;


        CPosture relative_deviation;
        relative_deviation.x = vertical_deviation;
        relative_deviation.y = lateral_deviation;
        relative_deviation.fThita = angle_temp;

        Center = relative_deviation;
        Center.InvTransform(m_pstCur);



        std::cout << "By yu: relative_deviation is (" << relative_deviation.x << ", " << relative_deviation.y <<
                     ", " << (relative_deviation.fThita / 3.14) * 180 << ")"<<" m_iLegTimes = "<<m_iLegTimes << std::endl;


        // Check Leg Pose
        if(m_iLegTimes < 5)
        {
            nStatus = 1;
            m_iLegTimes ++;
            m_lastLegsCenter = Center;
            *pResultPosture = Center;
            std::cout << "By Sam: USE Front Leg, time " << m_iLegTimes << std::endl;

#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "By Sam: USE Front Leg, time ", m_iLegTimes);
#endif
            return nStatus;
        }
        else
        {

            float diff_x = abs(Center.x - m_lastLegsCenter.x);
            float diff_y = abs(Center.y - m_lastLegsCenter.y);
            float diff_angle = abs(Center.fThita - m_lastLegsCenter.fThita);
            if(diff_angle <= 0.5 && diff_x <= 0.1 && diff_y <= 0.1)
            {
                nStatus = 1;
                m_iFaileTimes = 0;
                m_lastLegsCenter = Center;
                std::cout << "By Sam: Front Leg right, same to last!!!!!!" << std::endl;

#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "By Sam: Front Leg right, same to last!!!!!!");
#endif
            }else
            {
                m_lastLegsCenter = Center;

                if(m_iFaileTimes > 5)
                {
                    nStatus = -1;
                    m_iFaileTimes++;
                    std::cout << "By yu: Error, Diff is so Big ,Return Failed !!!!!!" << std::endl;
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "By yu: Error, Diff is so Big ,Return Failed !!!!!!");
#endif
                }else
                {
                    nStatus = 1; //累计5次才报错
                    m_iFaileTimes++;
                    std::cout << "By yu: Error, Diff is so Big ,Return Failed !!!!!!" << std::endl;

#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "By yu: Error, Diff is so Big !!!!!!");
#endif
                }

            }

        }
    }
    else
    {
        std::cout << "By Sam: Can Not Find Goods Leg !!!!!!" << std::endl;
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "By Sam: Can Not Find Goods Leg !!!!!!");
#endif
    }

    // outPut result
//    FilterLegPose(Center);
    m_lastLegsCenter = Center;
    *pResultPosture = Center;

    // For Display Center
    m_PointMatchList.m_PointPair[0].m_ptLocalToWorld.x = Center.x;
    m_PointMatchList.m_PointPair[0].m_ptLocalToWorld.y = Center.y;

    return nStatus;
}


//
//   对计算出的结果姿态进行平滑滤波。
//
bool CFeatureLocalization::FilterLegPose(CPosture& pose)
{
    int filter_box = 5;
    if(m_iLegPoseCount >= filter_box) {
        m_RecentPoses.pop_front();
        m_RecentPoses.push_back(pose);
        m_iLegPoseCount = filter_box;
    }
    else {
        m_RecentPoses.push_back(pose);
        m_iLegPoseCount += 1;
    }

    unsigned long pose_size = m_RecentPoses.size();
    CPosture pose_filtered(0, 0, 0);

    for (unsigned long i = 0 ; i < pose_size; i++)
    {
        pose_filtered.x += m_RecentPoses[i].x;
        pose_filtered.y += m_RecentPoses[i].y;

        // 角度标准化
        if(i == 0)
        {
            pose_filtered.fThita += m_RecentPoses[i].fThita;
        }
        else
        {
            float diff = pose_filtered.fThita - m_RecentPoses[i].fThita;
            std::cout << "By Sam: angle diff = " << (diff/PI) << ", adv_angle = " << (pose_filtered.fThita/PI) * 180 <<
                         "now_angle = " << (m_RecentPoses[i].fThita/PI) * 180 << std::endl;

            if(diff > PI)
            {
                float da_1 = pose_filtered.fThita - 2 * PI;

                if(abs(da_1) > m_RecentPoses[i].fThita)
                    pose_filtered.fThita = ((da_1 + m_RecentPoses[i].fThita) / 2) + (2 * PI);
                else
                    pose_filtered.fThita = (da_1 + m_RecentPoses[i].fThita) / 2;

                std::cout << "By Sam: angle diff > PI, pose_filtered.fThita =" << (pose_filtered.fThita/PI) * 180 << std::endl;
            }
            else if(diff < -PI)
            {
                float da_2 = m_RecentPoses[i].fThita - 2 * PI;

                if(abs(da_2) > pose_filtered.fThita)
                    pose_filtered.fThita = ((da_2 + pose_filtered.fThita) / 2) + (2 * PI);
                else
                    pose_filtered.fThita = (da_2 + pose_filtered.fThita) / 2;

                std::cout << "By Sam: angle diff < -PI, pose_filtered.fThita =" << (pose_filtered.fThita/PI) * 180 << std::endl;
            }
            else
            {
                pose_filtered.fThita = (pose_filtered.fThita + m_RecentPoses[i].fThita) / 2;
                std::cout << "By Sam: angle diff adv, pose_filtered.fThita =" << (pose_filtered.fThita/PI) * 180 << std::endl;
            }
        }
    }    

    pose_filtered.x = pose_filtered.x / pose_size;
    pose_filtered.y = pose_filtered.y / pose_size;

    pose = pose_filtered;

    std::cout << "By Sam: FilterLegPose ( " <<  pose_filtered.x << ", " << pose_filtered.y <<
                 ", " << (pose_filtered.fThita/PI) * 180 << "), filter " << pose_size << " times !!!!!!!!!" << std::endl;

#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "FilterLegPose ( ",pose_filtered.x,", ",pose_filtered.y,", ",(pose_filtered.fThita/3.14) * 180,
                  "), filter ", pose_size, " times !!!!!!!!!");
#endif

    return true;
}

//
//长廊定位函数
//wt_add_20221205
int CFeatureLocalization::LocalizationWithCorridor(CPosture* pResultPosture, CPosture &estimatePost)
{
    cout << "single_test_corridor_estimate x: " << estimatePost.x << ", y: " << estimatePost.y << ", thita: " << estimatePost.fThita << endl;
    int res = FEATURE_MATCH_UNKOWN_ERROR;

    if(estimatePost.x > 500 && estimatePost.x < 10000)
        estimatePost.x = estimatePost.x - 1000;

    if(estimatePost.y > 500 && estimatePost.y < 10000)
        estimatePost.y = estimatePost.y - 1000;



    if(estimatePost.x > 10000 && estimatePost.x < 30000)
    {
        estimatePost.x = estimatePost.x - 20000;
    }
    if(estimatePost.x > 30000 && estimatePost.x < 50000)
    {
        estimatePost.x = estimatePost.x - 40000;
        //return FEATURE_MATCH_UNKOWN_ERROR;
    }
    if(estimatePost.y > 10000 && estimatePost.y < 30000)
    {
        estimatePost.y = estimatePost.y - 20000;
    }
    if(estimatePost.y > 30000 && estimatePost.y < 50000)
    {
        estimatePost.y = estimatePost.y - 40000;
        //return FEATURE_MATCH_UNKOWN_ERROR;
    }

    /*
    if(estimatePost.x > 10000 && estimatePost.x < 30000
       && estimatePost.y >10000 &&  estimatePost.y < 30000)
    {
        estimatePost.x = estimatePost.x - 20000;
        estimatePost.y = estimatePost.y - 20000;
       // res = FEATURE_MATCH_SUCCESS;
    }

    if(estimatePost.x > 30000 && estimatePost.x < 50000
       && estimatePost.y >30000 &&  estimatePost.y < 50000)
    {
        estimatePost.x = estimatePost.x - 40000;
        estimatePost.y = estimatePost.y - 40000;
    }
*/

    cout << "single_test_corridor x: " << estimatePost.x << ", y: " << estimatePost.y << ", thita: " << estimatePost.fThita << endl;

    CTransform trans;
    trans.Create(estimatePost.x, estimatePost.y, estimatePost.fThita);

    EvaluateTransform(trans);

    //设置新姿态
    m_pstOld = m_pstCur;
    m_pstCur = trans;
    SwitchRatio(m_pstCur);

    if(pResultPosture != NULL)
        *pResultPosture = m_pstCur;

    return res;
}

//
//单反光板定位函数
//wt_add_20221202
int CFeatureLocalization::LocalizationWithSinglePoint(CPosture* pResultPosture, CPosture &estimatePost)
{

    float lr = m_PointMatchList.m_PointPair[0].m_ptLocal.r;
    float la = m_PointMatchList.m_PointPair[0].m_ptLocal.a;
    float wx = m_PointMatchList.m_PointPair[0].m_ptWorld.x;
    float wy = m_PointMatchList.m_PointPair[0].m_ptWorld.y;

    cout << "lr: " << lr << ", la: " << la << ", wx: " << wx <<", wy: " << wy <<  endl;

    float angle = estimatePost.fThita;
    float x = wx - lr * cos(la + angle);
    float y = wy - lr * sin(la + angle);

    cout << "single_test x: " << x << ", y: " << y << ", thita: " << angle << endl;


    if(estimatePost.x > 500 && estimatePost.x < 10000) //判断需要修正的方向
    {
        y = estimatePost.y;
        estimatePost.x = estimatePost.x - 1000;
    }

    if(estimatePost.y > 500 && estimatePost.y < 10000)
    {
        x = estimatePost.x;
        estimatePost.y = estimatePost.y - 1000;
    }

    if(estimatePost.x > 10000 && estimatePost.x < 30000)
    {
        estimatePost.x = estimatePost.x - 20000;
        y = estimatePost.y;
    }
    if(estimatePost.x > 30000 && estimatePost.x < 50000)
    {
        estimatePost.x = estimatePost.x - 40000;
        y = estimatePost.y;
        return FEATURE_MATCH_UNKOWN_ERROR;
    }
    if(estimatePost.y > 10000 && estimatePost.y < 30000)
    {
        estimatePost.y = estimatePost.y - 20000;
        x = estimatePost.x;
    }
    if(estimatePost.y > 30000 && estimatePost.y < 50000)
    {
        estimatePost.y = estimatePost.y - 40000;
        x = estimatePost.x;
        return FEATURE_MATCH_UNKOWN_ERROR;
    }
    /*
    if(estimatePost.x > 10000 && estimatePost.x < 30000
       && estimatePost.y >10000 &&  estimatePost.y < 30000)
    {
        estimatePost.x = estimatePost.x - 20000;
        estimatePost.y = estimatePost.y - 20000;
    }

    if(estimatePost.x > 30000 && estimatePost.x < 50000
       && estimatePost.y >30000 &&  estimatePost.y < 50000)
    {
        estimatePost.x = estimatePost.x - 40000;
        estimatePost.y = estimatePost.y - 40000;
        return FEATURE_MATCH_UNKOWN_ERROR;
    }
*/

    cout << "single_test_fix x: " << x << ", y: " << y << ", thita: " << angle << endl;
    cout << "single_test_estimate x: " << estimatePost.x << ", y: " << estimatePost.y << ", thita: " << estimatePost.fThita << endl;

    if(fabs(x - estimatePost.x) > 0.45 || fabs(y - estimatePost.y) > 0.45) //wt_fix 20230131 添加定位超差修正
    {     
        x = estimatePost.x;
        y = estimatePost.y;
//            return FEATURE_LOC_OVER_RANGE;
    }

    CTransform trans;
    trans.Create(x, y, angle);

    EvaluateTransform(trans);

    if(m_fScore < m_fRatio)
        return FEATURE_MATCH_UNKOWN_ERROR;

    //设置新姿态
    m_pstOld = m_pstCur;
    m_pstCur = trans;
    SwitchRatio(m_pstCur);

    if (pResultPosture != NULL)
        *pResultPosture = m_pstCur;

    return FEATURE_MATCH_SUCCESS;
}

//
//双反光板定位函数
//wt_add_20221202
int CFeatureLocalization::LocalizationWithDoublePoint(CPosture* pResultPosture, CPosture &estimatePost)
{
    // 重新设置最小二乘矩阵数据
    m_LeastSquareMethod.Start();

    // 将关于所有匹配点对的最小二乘数据加入到LSM求解器中
    m_PointMatchList.CreateLeastSquareData(&m_LeastSquareMethod);

    // 求解最小二乘法
    CTransform trans;
    float f[4];
    if (m_LeastSquareMethod.Solve(f, 4))
    {
        float sin = f[0];
        float cos = f[1];
        float angle = (float)atan2(sin, cos);
        float x = f[2];
        float y = f[3];

        cout << "double_test_ x: " << x << ", y: " << y << ", thita: " << angle << endl;

        angle = estimatePost.fThita;

        if(estimatePost.x > 500 && estimatePost.x < 10000) //判断需要修正的方向
        {
            y = estimatePost.y;
            estimatePost.x = estimatePost.x - 1000;
        }

        if(estimatePost.y > 500 && estimatePost.y < 10000)
        {
            x = estimatePost.x;
            estimatePost.y = estimatePost.y - 1000;
        }

        if(estimatePost.x > 10000 && estimatePost.x < 30000)
        {
            estimatePost.x = estimatePost.x - 20000;
            y = estimatePost.y;
        }
        if(estimatePost.x > 30000 && estimatePost.x < 50000)
        {
            estimatePost.x = estimatePost.x - 40000;
            y = estimatePost.y;
            return FEATURE_MATCH_UNKOWN_ERROR;
        }
        if(estimatePost.y > 10000 && estimatePost.y < 30000)
        {
            estimatePost.y = estimatePost.y - 20000;
            x = estimatePost.x;
        }
        if(estimatePost.y > 30000 && estimatePost.y < 50000)
        {
            estimatePost.y = estimatePost.y - 40000;
            x = estimatePost.x;
            return FEATURE_MATCH_UNKOWN_ERROR;
        }
/*
        if(estimatePost.x > 10000 && estimatePost.x < 30000
           && estimatePost.y >10000 &&  estimatePost.y < 30000)
        {
            estimatePost.x = estimatePost.x - 20000;
            estimatePost.y = estimatePost.y - 20000;
        }

        if(estimatePost.x > 30000 && estimatePost.x < 50000
           && estimatePost.y >30000 &&  estimatePost.y < 50000)
        {
            estimatePost.x = estimatePost.x - 40000;
            estimatePost.y = estimatePost.y - 40000;
            return FEATURE_MATCH_UNKOWN_ERROR;
        }
*/

        cout << "double_test_fix x: " << x << ", y: " << y << ", thita: " << angle << endl;
        cout << "double_test_estimate x: " << estimatePost.x << ", y: " << estimatePost.y << ", thita: " << estimatePost.fThita << endl;

        if(fabs(x - estimatePost.x) > 0.45 || fabs(y - estimatePost.y) > 0.45) //wt_fix 20230131 添加定位超差修正
        {
            x = estimatePost.x;
            y = estimatePost.y;
//            return FEATURE_LOC_OVER_RANGE;
        }

        trans.Create(x, y, angle);
    }
    else
        return FEATURE_MATCH_MATRIX_ERROR;

    EvaluateTransform(trans);

    if(m_fScore < m_fRatio)
        return FEATURE_MATCH_UNKOWN_ERROR;

    // 设置新姿态
    m_pstOld = m_pstCur;
    m_pstCur = trans;
    SwitchRatio(m_pstCur);

    if (pResultPosture != NULL)
        *pResultPosture = m_pstCur;

    return FEATURE_MATCH_SUCCESS;
}

//
// 在指定的层内进行快速影射定位。
// 重点考虑快速匹配问题
int CFeatureLocalization::QuickMapLocalization(CPosture* pResultPosture)
{
    // 重新设置最小二乘矩阵数据
    m_LeastSquareMethod.Start();

    if(m_PointMatchList.GetCount() > m_Param->nMostMatchCount_)
        m_PointMatchList.m_nCount = m_Param->nMostMatchCount_;

    if(m_LineMatchListX.m_nFootInLineCount > 0 &&
            m_LineMatchListY.m_nFootInLineCount > 0 &&
            m_LineMatchList.m_nFootInLineCount > 2)
        m_LineMatchList.m_nCount = m_LineMatchList.m_nFootInLineCount;
    else if(m_PointMatchList.GetCount() > 0 &&
            m_PointMatchList.GetCount() + m_LineMatchList.m_nFootInLineCount > 3)
        m_LineMatchList.m_nCount = m_LineMatchList.m_nFootInLineCount;

    // 将关于所有匹配点对的最小二乘数据加入到LSM求解器中
    if (!m_PointMatchList.CreateLeastSquareData(&m_LeastSquareMethod))
    {
        //		return FEATURE_MATCH_MATRIX_ERROR;
    }
    // 将关于所有匹配线段对的最小二乘数据加入到LSM求解器中
    if (!m_LineMatchList.CreateLeastSquareData(&m_LeastSquareMethod))
    {
        //		return FEATURE_MATCH_MATRIX_ERROR;
    }
    // 求解最小二乘法
    CTransform trans;
    float f[4];
    if (m_LeastSquareMethod.Solve(f, 4))
    {
        float sin = f[0];
        float cos = f[1];
        float angle = (float)atan2(sin, cos);

        float x = f[2];
        float y = f[3];
        trans.Create(x, y, angle);
        cout << "full_test x: " << x << ", y: " << y << ", thita: " << angle << endl;
    }
    else
        return FEATURE_MATCH_MATRIX_ERROR;

    EvaluateTransform(trans);

    //add by zhaojc 20200708
    if(m_fScore < m_fRatio)
    {
        return FEATURE_MATCH_UNKOWN_ERROR;
    }
    // 设置新姿态
    m_pstOld = m_pstCur;
    m_pstCur = trans;
    SwitchRatio(m_pstCur);

    if (pResultPosture != NULL)
        *pResultPosture = m_pstCur;

    return FEATURE_MATCH_SUCCESS;
}

void CFeatureLocalization::EvaluateTransform(const CTransform& trans)
{
    if(m_LineMatchList.GetCount() > 0)
    {
        m_LineMatchList.trans = trans;
        m_LineMatchList.EvaluateTransform();
    }
    if(m_PointMatchList.GetCount() > 0)
    {
        m_PointMatchList.trans = trans;
        m_PointMatchList.EvaluateTransform();
    }
    m_fScore = (m_PointMatchList.overlap_ratio * m_PointMatchList.GetCount() + m_LineMatchList.m_fScore * m_LineMatchList.GetCount()) /
               (m_PointMatchList.GetCount() + m_LineMatchList.GetCount());
}
//
//   在当前层内进行快速配准，并生成按极径从小到大顺序的匹配表（单帧定位）。
//
bool CFeatureLocalization::PointFeatureQuickMatch()
{
    // 清空配准表
    m_PointMatchList.Clear();
    int *iRefCheckBuffer = NULL;
    int iRefSize = m_RefLayer.GetCount();
    if(iRefSize > 0)
    {
        iRefCheckBuffer = new int[iRefSize];
        memset(iRefCheckBuffer,0,iRefSize * sizeof(int));
    }
    else
        return false;

    int  *iLocalCheckBuffer = NULL;
    int iLocalSize = m_LocalLayer.GetCount();
    if(iLocalSize > 0)
    {
        iLocalCheckBuffer = new int[iLocalSize];
        memset(iLocalCheckBuffer,0,iLocalSize * sizeof(int));
    }
    else
    {
        if(iRefCheckBuffer)
        {
            delete iRefCheckBuffer;
            iRefCheckBuffer = NULL;
        }

        return false;
    }

    CPointMatchPair pair;

    for(int i = 0; i < m_RefLayer.GetCount(); i++)
    {
        // 从参考点集中取出下一点
        CPnt& pntWorld = *m_RefLayer[i];
        for(int j = 0; j < m_LocalLayer.GetCount(); j++)
        {
            // 从局部点集中取出下一点
            CPnt& pntLocal = *m_LocalLayer[j];
            // 计算出极径和极角差距
            float fRangeDiff = fabs(pntWorld.r - pntLocal.r);//半径差
            float fAngleDiff = fabs(CAngle::NormAngle(pntWorld.a) - CAngle::NormAngle(pntLocal.a));//角度差
            float fDistance = sqrt(pntWorld.r * pntWorld.r + pntLocal.r * pntLocal.r - 2 * pntWorld.r * pntLocal.r * cos(fAngleDiff));//余弦定理求两点的距离
//            cout<<"test: "<<j<<", "<<pntWorld.r<<", "<<pntLocal.r<<", "<<fRangeDiff<<", "<<pntWorld.a<<", "<<pntLocal.a<<" "<<fAngleDiff<<endl;//匹配不成功打开
            // 仅当极径在规定的范围内，且极径差、极角差不超过规定的门限时，才视为找到一对配准点
            if(pntLocal.r < m_Param->maxRange &&
                    pntLocal.r > m_Param->minRange &&
                    fAngleDiff < m_Param->angleEqualLimit &&
                    fRangeDiff < m_Param->rangeEqualLimit)
            {
                // 向匹配表中添加此点对(按极径从小到大顺序)
                if(iRefCheckBuffer[i] == 0 && iLocalCheckBuffer[j] == 0)
                {
                    pair.Create(pntLocal, pntWorld);
                    iRefCheckBuffer[i]++;
                    iLocalCheckBuffer[j]++;
                    m_PointMatchList.AddInOrder(pair);
//                  break;
                }
            }
        }
    }

    if(iLocalCheckBuffer)
    {
        delete []iLocalCheckBuffer;
        iLocalCheckBuffer = NULL;
    }

    if(iRefCheckBuffer)
    {
        delete []iRefCheckBuffer ;
        iRefCheckBuffer = NULL;
    }

    m_PointMatchList.Filter(); //wt_fix 20220906 点对多重匹配过滤条件修改为点对距离大小

    // 在此应用NearestN参数，对参与匹配运算的反光板数量进行限制
    if (m_PointMatchList.GetCount() > m_Param->nMostMatchCount_)
        m_PointMatchList.m_nCount = m_Param->nMostMatchCount_;

    return (m_PointMatchList.GetCount() + m_LineMatchList.GetCount() > 2); //wt_fix 20220906 特征点数大于2个
}

//
//   在当前层内进行快速配准，并生成按极径从小到大顺序的匹配表（多帧定位）。
//
bool CFeatureLocalization::PointFeatureQuickMatchUseDequeReflectors(CScan *pCurScan)
{
    // 清空配准表
    m_PointMatchList.Clear();
    vector<CPointMatchPair> CPointMatchPairSet;
    CPointMatchPairSet.clear();

    Matrix2d cov;//协方差矩阵

    long long int locStart = GetTickCount();
    for(int i = 0; i < m_RefLayer.GetCount(); i++) //环境文件中该位置可识别反光板的个数
    {
        CPointMatchPair pair;
        pair.m_ptLocalSet.clear();
        CPointFeature* pntWorld = m_RefLayer[i]; //从参考点集中取出下一点
        cov = pntWorld->m_stVariousDis.m_cov;

        for(int j = 0; j < m_DequeReflectors.size(); j++) //多帧定位中的帧数
        {
            deque<stReflector> & ReflectorTemp = m_DequeReflectors.at(j);

//            cout << "ReflectorTemp.size: " << ReflectorTemp.size() << endl;

            for(int k = 0; k < ReflectorTemp.size(); k++)
            {
                stReflector &stReflectorTemp = ReflectorTemp.at(k);
                CPnt pntWorldTemp(pntWorld->x, pntWorld->y);
                if(stReflectorTemp.m_bIsMatched)
                {
                    CPnt pntTemp = stReflectorTemp.m_ptMatchMapWorld;
//                    cout << "IsMatched, pntTemp: " << pntTemp.x << ", " << pntTemp.y << endl;
                    if(pntTemp == pntWorldTemp)
                        pair.Add(stReflectorTemp.m_ptLocal, *pntWorld);
                }
                else
                {
                    // 从局部点集中取出下一点
                    CPnt pntLocal = stReflectorTemp.m_ptLocal;
                    CPnt t(pntLocal.x, pntLocal.y);
                    t.InvTransform(m_pstCur);
                    stVariousDis ostVariousDis;
                    double MaDis = ostVariousDis.CalculateMahalanobisDistance(t, *pntWorld, cov);
                    // 计算出极径和极角差距
                    float fRangeDiff = fabs(pntWorld->r - pntLocal.r); //半径差
                    float fAngleDiff = fabs(CAngle::NormAngle(pntWorld->a) - CAngle::NormAngle(pntLocal.a)); //角度差
//                    cout<<"test: k"<<k<<", "<<pntWorld->r<<", "<<pntLocal.r<<", "<<fRangeDiff<<", "<<CAngle::NormAngle(pntWorld->a)<<", "<<CAngle::NormAngle(pntLocal.a)<<" "<<fAngleDiff<<endl;//匹配不成功打开
//                    cout << "test: k" << k << ", " << pntWorld->r << ", " << pntLocal.r << ", " << fRangeDiff << ", " << pntWorld->a
//                       << ", " << pntLocal.a << " " << fAngleDiff << endl; //匹配不成功打开
                    // 仅当极径在规定的范围内，且极径差、极角差不超过规定的门限时，才视为找到一对配准点

                    if(pntLocal.r < m_Param->maxRange && pntLocal.r > m_Param->minRange &&
                            fAngleDiff < m_Param->angleEqualLimit && fRangeDiff < m_Param->rangeEqualLimit)
                    {
                        pair.Add(stReflectorTemp.m_ptLocal, *pntWorld);
                        stReflectorTemp.m_bIsMatched = true;
                        stReflectorTemp.m_ptMatchMapWorld = pntWorldTemp;
                    }
                }
            }
        }

        for(int m = 0; m < ((CFeatureScan *)pCurScan)->m_ptHighBrightPnt.size(); m++) //最新一帧的高亮点
        {
            stReflector &stReflectorTemp = ((CFeatureScan *)pCurScan)->m_ptHighBrightPnt.at(m);

            CPnt pntWorldTemp(pntWorld->x, pntWorld->y);
            if(!stReflectorTemp.m_bIsMatched)
            {
                // 从局部点集中取出下一点
                CPnt pntLocal = stReflectorTemp.m_ptLocal;
                CPnt t(pntLocal.x,pntLocal.y);
                t.InvTransform(m_pstCur);
                stVariousDis ostVariousDis;
                double MaDis = ostVariousDis.CalculateMahalanobisDistance(t, *pntWorld, cov);
                // 计算出极径和极角差距
                float fRangeDiff = fabs(pntWorld->r - pntLocal.r); //半径差
                float fAngleDiff = fabs(CAngle::NormAngle(pntWorld->a) - CAngle::NormAngle(pntLocal.a)); //角度差
//                cout << "test: m" << m << ", " << pntWorld->r << ", " << pntLocal.r << ", " << fRangeDiff << ", " << pntWorld->a
//                   << ", " << pntLocal.a << " " << fAngleDiff << endl; //匹配不成功打开
                // 仅当极径在规定的范围内，且极径差、极角差不超过规定的门限时，才视为找到一对配准点
                if(pntLocal.r < m_Param->maxRange && pntLocal.r > m_Param->minRange &&
                        fAngleDiff < m_Param->angleEqualLimit && fRangeDiff < m_Param->rangeEqualLimit)
                {
                    pair.Add(stReflectorTemp.m_ptLocal, *pntWorld);
                    stReflectorTemp.m_bIsMatched = true;
                    stReflectorTemp.m_ptMatchMapWorld = pntWorldTemp;
                }
            }
        }
//        cout << "Pair.m_ptLocalSet.size: " << pair.m_ptLocalSet.size() << endl;
        if(pair.m_ptLocalSet.size() > (m_Param->iMultiFrameLocDequeSize - 1))
            CPointMatchPairSet.push_back(pair);
    }

    long long int locEnd = GetTickCount();
    long long int locCostTime =  static_cast<int>(locEnd - locStart);

//    std::cout << "locCostTime::" << locCostTime << std::endl;

    for(int i = 0; i < CPointMatchPairSet.size(); i++)
    {
        CPointMatchPair PairTemp = CPointMatchPairSet.at(i);
        CPnt ptLocalTemp;
        CPnt ptAngleLocalTemp;
        bool ptAngleOver = false; //角度越界判断标识符 wt_add 20230207

        cout << "PairTemp.m_ptLocalSet.size: " << PairTemp.m_ptLocalSet.size() << endl;

        for(int j = 0; j < PairTemp.m_ptLocalSet.size(); j++)
        {
            CPnt pnttemp = PairTemp.m_ptLocalSet.at(j);

            if(fabs(pnttemp.a - ptAngleLocalTemp.a) > 1.57 && ptAngleLocalTemp.a != 0)
            {
                ptLocalTemp.a = 0;
                ptAngleLocalTemp.a = 0;
                ptLocalTemp.r = 0;
                ptLocalTemp.x = 0;
                ptLocalTemp.y = 0;
                ptAngleOver = true;
                break; //角度跨越-180度时，需要清零，调整区间，重新累计 wt_add 20230207
            }
            ptLocalTemp.a += pnttemp.a;
            ptAngleLocalTemp.a = pnttemp.a;
            ptLocalTemp.r += pnttemp.r;
            ptLocalTemp.x += pnttemp.x;
            ptLocalTemp.y += pnttemp.y;
        }

        if(ptAngleOver)
        {
            for(int k = 0; k < PairTemp.m_ptLocalSet.size(); k++)
            {
                CPnt pnttemp = PairTemp.m_ptLocalSet.at(k);

                if(pnttemp.a < 0)
                    pnttemp.a += 6.28318; //将-180度的角度调整区间至0~360度 wt_add 20230207

                ptLocalTemp.a += pnttemp.a;
                ptLocalTemp.r += pnttemp.r;
                ptLocalTemp.x += pnttemp.x;
                ptLocalTemp.y += pnttemp.y;
            }
        }

        float fSum = PairTemp.m_ptLocalSet.size();

        PairTemp.m_ptLocal.a = CAngle::NormAngle2(ptLocalTemp.a / fSum);
        PairTemp.m_ptLocal.r = ptLocalTemp.r / fSum;
        PairTemp.m_ptLocal.x = ptLocalTemp.x / fSum;
        PairTemp.m_ptLocal.y = ptLocalTemp.y / fSum;

        cout << "PairTemp.m_ptLocal.a: " << PairTemp.m_ptLocal.a << endl;

        m_PointMatchList.AddInOrder(PairTemp);
    }

//    m_PointMatchList.Filter();

    // 在此应用NearestN参数，对参与匹配运算的反光板数量进行限制
    if(m_PointMatchList.GetCount() > m_Param->nMostMatchCount_)
        m_PointMatchList.m_nCount = m_Param->nMostMatchCount_;

    cout << "m_PointMatchList.size: " << m_PointMatchList.GetCount() << endl;

    return (m_PointMatchList.GetCount() + m_LineMatchList.GetCount() > 2);
}


///////////////////////////////////////////////////////////////////////////////

//
//   在当前层内进行全局映射定位。
//   返回值：
//     1 - 成功
//    -1 - 全局定位没有找到合适的匹配
//    -2 - 全局定位匹配点对数量小于
//
int CFeatureLocalization::FullMapLocalization(CPosture* pResultPosture)
{
    // 在有解的情况下，优选出最优解
    for(int i = 0; i < m_MatchTabSet.count; i++)
    {
        m_MatchTabSet.tab[i].FindTransform();
        m_MatchTabSet.tab[i].EvaluateTransform();
    }
    // 在匹配表集合中选出最优的那一个
    int index = m_MatchTabSet.FindBestMatch();

    if(index >= 0)
    {
        SwitchRatio(m_MatchTabSet.tab[index].trans);
        if(m_MatchTabSet.tab[index].overlap_ratio > m_fRatio)
        {
            m_PointMatchList = m_MatchTabSet.tab[index]; //确定了最优匹配表
            m_fScore = m_MatchTabSet.tab[index].overlap_ratio; //20201015zjc
            m_pstOld = m_pstCur = m_PointMatchList.trans; //得到激光头姿态

            if(pResultPosture != NULL)
                *pResultPosture = m_pstCur;

            m_nFullMapCount ++;
            return FEATURE_MATCH_SUCCESS;
        }
        else
            return FEATURE_MATCH_UNKOWN_ERROR;
    }
    else
        return FEATURE_MATCH_POINTS_NOT_ENOUGH; //匹配对数小于5
}

//
//   在全局范围内进行粗略地配准，并将结果保存于m_MatchTabSet中。
//
bool CFeatureLocalization::PointFeatureFullMapMatch(float equal_limit)
{
    float fEqualLimit, fLineValidLimit;

    m_MatchTabSet.Clear();
    if(m_bLocalFullMap)
    {
        m_RefLayer.CreateFromOtherSet(*m_pWorldLayer, m_pstOld,m_Param->maxRange,m_Param->vecSpecialPntList_);
        m_RefLayer.CreateDistanceCache(true);
    }
    else
    {
        m_RefLayer.Clear();
        for(int i = 0; i < m_pWorldLayer->GetCount(); i++)
        {
            CPointFeature* pFeature = m_pWorldLayer->at(i);
            m_RefLayer += *pFeature;
#if 0
            CPointFeature* pNewFeature = pFeature->Duplicate();
            m_RefLayer.Add(pNewFeature);
#endif
        }
        m_RefLayer.CreateDistanceCache(true);
    }

    int nLocalCount = m_LocalLayer.GetCount();
    int nRefCount = m_RefLayer.GetCount();
    ///////////////////////////////////

    // 仅计算局部测量点云内各点之间的距离(世界点云已预处理计算过)
    m_LocalLayer.CreateDistanceCache(true);

#ifdef USE_LENGTH_SQUARE_COMPARE
    fEqualLimit = equal_limit*equal_limit;
    fLineValidLimit = m_Param->m_fMinLineLen*m_Param->m_fMinLineLen;
#else
    fEqualLimit = equal_limit;
    fLineValidLimit = m_Param->fMinLineLen_;
#endif

    for(int i = 0; i < nLocalCount - 1; i++)
    {
        // 取m_LocalLayer点集中的第一点
        CPnt& pnt11 = *m_LocalLayer[i];//.GetPoint(i);

        for(int j = i + 1; j < nLocalCount; j++)
        {
            // 取m_LocalLayer点集中的第二点
            CPnt& pnt12 = *m_LocalLayer[j];//.GetPoint(j);

            for(int m = 0; m < nRefCount - 1; m++)
            {
                // 取world点集中的第一点
                CPnt& pnt21 = *m_RefLayer[m];//.GetPoint(m);

                for(int n = m + 1; n < nRefCount; n++)
                {
                    // 取world点集中的第二点
                    CPnt& pnt22 = *m_RefLayer[n];//.GetPoint(n);

                    // dist1为pnt11和pnt12之间的距离
                    float dist1 = m_LocalLayer.PointDistance(i, j);

                    // dist2为pnt21和pnt22之间的距离
                    float dist2 = m_RefLayer.PointDistance(m, n);

                    // 计算两条线段的长度差
                    float dist_err = (float)fabs(dist1 - dist2);

                    // 如两条线段长度差太大，则此两条线段匹配失败
                    if(dist_err > fEqualLimit)
                        continue;

                    // 如这两条线段之中任一线段过短，也不符合匹配条件
                    if(dist1 < fLineValidLimit || dist2 < fLineValidLimit)
                        continue;

                    CPointMatchPair pair1(pnt11, pnt21);
                    CPointMatchPair pair2(pnt12, pnt22);
                    CPointMatchPair pair3(pnt11, pnt22);
                    CPointMatchPair pair4(pnt12, pnt21);

                    // 如点对已在当匹配表中包含，则跳过
                    if(m_MatchTabSet.Search(pair1, pair2) >= 0	|| m_MatchTabSet.Search(pair3, pair4) >= 0)
                        continue;

                    CLine line1(pnt11, pnt12);
                    CLine line2(pnt21, pnt22);

                    CTransform trans;
                    trans.Create(line2, line1);

                    CPointMatchList tab;
                    tab.Add(pair1);
                    tab.Add(pair2);

                    if(DirectRegister(trans, &tab) > 0)
                    {
                        tab.SetTrans(trans);
                        m_MatchTabSet.Add(tab);
                        continue;
                    }

                    CLine line3(pnt22, pnt21);
                    trans.Create(line3, line1);

                    tab.Clear();
                    tab.Add(pair3);
                    tab.Add(pair4);

                    if(DirectRegister(trans, &tab) > 0)
                    {
                        tab.SetTrans(trans);
                        m_MatchTabSet.Add(tab);
                    }
                }
            }
        }
    }

    return (m_MatchTabSet.count > 0);
}


//
//   在激光坐标系内寻找货架腿对
//
bool CFeatureLocalization::PointFeatureFindLegPairs(float equal_limit)
{
    // 清空配准表
    m_PointMatchList.Clear();
    float fEqualLimit, fLineValidLimit;

    int nLocalCount = m_LocalLayer.GetCount();

    // 仅计算局部测量点云内各点之间的距离(世界点云已预处理计算过)
//    m_LocalLayer.CreateDistanceCache(true);   // By Sam Temp Delete

    fEqualLimit = equal_limit;
    fLineValidLimit = m_Param->fMinLineLen_;

    float disLaser = std::numeric_limits<float>::max(); // Max
    float delta = 0.1;   // By Sam For LegSeg
    CPointMatchPair legPair;
    bool legOK = false;

    for (int i = 0; i < nLocalCount - 1; i++)
    {
        // 取m_LocalLayer点集中的第一点
        float addR = 0;
        CPnt& pnt11 = *m_LocalLayer[i];
        std::cout<<"By yu : pnt11.r,a = ("<<pnt11.r<<","<< pnt11.a<<")"  <<std::endl;

        if (pnt11.r > 3)
            continue;

        for (int j = i+1; j < nLocalCount; j++)
        {
            // 取m_LocalLayer点集中的第二点
            CPnt& pnt12 = *m_LocalLayer[j];//.GetPoint(j);
            std::cout<<"By yu : pnt12.r,a = ("<<pnt12.r<<","<< pnt12.a<<")"  <<std::endl;

            if (pnt12.r > 3)
                continue;

            if (abs(pnt11.a + pnt12.a) > 1.1)//0.34
                continue;
            std::cout<<"By yu : Find Result, pnt11.r,a = ("<<pnt11.r<<","<< pnt11.a<<")" <<" ("<<pnt12.r<<","<< pnt12.a<<")" <<std::endl;

            float dist1 = m_LocalLayer.PointDistance(i, j);

//            if (dist1 < fLineValidLimit && dist1 < fLineValidLimit)
//                continue;
            if (dist1 > equal_limit + delta || dist1 < equal_limit - delta)
                continue;

            addR = pnt11.r + pnt12.r;
            std::cout << "By Sam: " << i << " " << j << " PointDistance = " << dist1 <<
                         ", R = " << addR << std::endl;

            if (addR < disLaser)
            {
                disLaser = addR;
                legPair.Create(pnt11, pnt12);
                legOK = true;
                std::cout << "By Sam: Create point_" << i << " is (" << pnt11.x << ", " << pnt11.y <<
                             "), point_" << j << " is (" << pnt12.x << ", " << pnt12.y << ")" << std::endl;
            }

        }
    }

    if (legOK)
    {
        m_iLegFalseNum = 0;
        m_PointMatchList.AddInOrder(legPair);

#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "By Sam: Found Goods Leg");
#endif

        return true;
    }
//    else
//    {
//        m_iLegFalseNum++;
//        if(m_iLegFalseNum > 5)
//        {
////            m_lastLegsCenter = CPosture(0, 0, 0);
//            ReSetLegPose();  // By Sam: If not found leg 5 times, reset !!!!
//            m_iLegFalseNum = 5;
//        }
//    }

//#ifdef USE_BLACK_BOX
//    FILE_BlackBox(LocBox, "By Sam: Can Not Find Goods Leg !!!!!!");
//#endif

    return false;
}



//
//   根据当前匹配表中的数据，尝试直接对两个点云进行配准。
//
short CFeatureLocalization::DirectRegister(CTransform trans, CPointMatchList* tab)
{

    for(int i = 0; i < m_LocalLayer.GetCount(); i++)
    {
        // 从本地扫描点集中取一点
        CPnt pnt1 = *m_LocalLayer[i];//.GetPoint(i);


        // 如果此本地点已在匹配表中，则跳过此点
        if(tab->SearchByLocalPoint(pnt1) >= 0)
            continue;

        // 将该点直接变换到参考点集坐标系中
        CPnt pnt2 = trans.GetWorldPoint(pnt1);

        // 将此点与参考点集中的所有点进行比对，看是不是有与它近乎于重合的点
        for(int j = 0; j < m_RefLayer.GetCount(); j++)
        {
            CPnt pnt3 = *m_RefLayer[j];//.GetPoint(j);
            // 如果此世界点已在匹配表中，则跳过此点
            if(tab->SearchByWorldPoint(pnt3) >= 0)
                continue;

#ifdef USE_LENGTH_SQUARE_COMPARE
            float cost = pnt2.Distance2To(pnt3);

            if(cost < (m_Param->m_fSamePointMaxDist*m_Param->m_fSamePointMaxDist))
            {
                CPointMatchPair pair(pnt1, pnt3);
                tab->Add(pair);
                break;
            }
#else
            float cost = pnt2.DistanceTo(pnt3);

            // 如果发现近乎于重合的点，视为发现一对匹配点对
            if(cost < m_Param->fSamePointMaxDist_)
            {
                CPointMatchPair pair(pnt1, pnt3);
                tab->Add(pair);
                break;
            }
#endif
        }
    }

    // 看看匹配点对的数量是否足够
    if(tab->m_nCount >= m_Param->nLeastMatchCount_)
        return 1;

    return 0;
}

//////////////////////////////////////////////////////////////////////////////

//
//   在点云中找到所有的“点状特征”，在当前扫描的点云反光板集合构建m_LocalLayer
//
bool CFeatureLocalization::CreatLocalLayerFromCurScan(CScan* pScan)
{
    m_LocalLayer.Clear();
    float fMaxCreatFromScanRange = 0.0f;
    for(int i = 0; i < ((CFeatureScan*)pScan)->m_ptReflectors.size(); i++)
    {
        CPnt pntLocal;
        pntLocal.r = ((CFeatureScan*)pScan)->m_ptReflectors[i].r;
        pntLocal.a = ((CFeatureScan*)pScan)->m_ptReflectors[i].a;
        pntLocal.id = i;
        CPointFeature* pNewFeature = new CPointFeature;
        pNewFeature->SetCenterPoint(pntLocal);
        pNewFeature->SetType(1);
        if(pntLocal.r > fMaxCreatFromScanRange)
            fMaxCreatFromScanRange = pntLocal.r;
//        m_LocalLayer.Add(pNewFeature);        // 添加此点特征
        m_LocalLayer += pNewFeature;        // 添加此点特征
    }

//    m_LocalLayer.SortByAngle();

    m_LocalLayer.UpdateCartisian();
    return true;
}

bool CFeatureLocalization::AddPntToLocalLayer(CPnt pntLocal)
{
    CPointFeature* pNewFeature = new CPointFeature;
    pNewFeature->SetCenterPoint(pntLocal);
    pNewFeature->SetType(1);
    m_LocalLayer += pNewFeature;        // 添加此点特征
    m_LocalLayer.UpdateCartisian();
    return true;
}

//
//    计算一个姿态角到一条有方向直线的逆时针转角。
//
CAngle PostureAngleToLine(CPosture& pst, CLine& ln)
{
    return (ln.m_angSlant - pst.GetAngle());
}

//
//   在点云中找到所有的“直线特征”。
//
bool CFeatureLocalization::MatchLineFeatures(CScan* pCurScan)
{
    CPosture pstScanner = m_pstCur;
    // 取得激光头所在的位置和角度
    CPnt ptScanner = pstScanner.GetPntObject();
    CAngle angScanner = pstScanner.GetAngle();
    // 取得参考直线集和当前直线集
//    CLineFeatureSet* pRefLineFeatures = m_pWorldLines->Duplicate();
    CLineFeatureSet* pRefLineFeatures = m_pRefWorldLines;
    CLineFeatureSet* pCurLineFeatures = ((CFeatureScan*)pCurScan)->m_pLineFeatures->Duplicate();
    m_trans.Init(m_pstCur);

    bool bIsPushOk = false;
    // 针对参数直线段集合，依次进行考察
    for (int i = 0; i < pRefLineFeatures->size(); i++)
    {
        CLineMatchPair Pair;
        CPnt ptFoot1;

        // 取得参考直线段
        CLineFeature& Line1 = *(pRefLineFeatures->at(i));

        // 判断参考直线段的有效识别面是否与当前姿态相符
        CPnt& ptStart = Line1.m_ptStart;
        CAngle ang(ptStart, ptScanner);
        CAngle angDiff = ang - Line1.SlantAngle();

        // 对于单面有效的情况，核对工作面是否相符
        if (Line1.m_nWhichSideToUse == FEATURE_DIR_FRONT_SIDE_ONLY)
        {
            if (angDiff.Quadrant() > 2)          // 角度差大于180度，工作面不符
                continue;
        }
        else if (Line1.m_nWhichSideToUse == FEATURE_DIR_BACK_SIDE_ONLY)
        {
            if (angDiff.Quadrant() <= 2)         // 角度差小于180度，工作面不符
                continue;
        }

        Line1.m_nId = i;

        // 计算当前姿态在参考线段上的垂直投影点ptFoot1，并取得投影线长度
        float fDist1 = Line1.DistanceToPoint(false, ptScanner, NULL, &ptFoot1);

        // 构造从激光头到上述垂足点的直线段
        CLine lnToFoot1(ptScanner, ptFoot1);

        // 计算从当前激将头姿态到参考直线的(逆时针)转角
        CAngle ang1 = PostureAngleToLine(pstScanner, lnToFoot1);

        CAngle angScan11, angScan12;

        // 计算线段两个端点到激光头的连线与激光头当前姿态的夹角
        FindScanAngles(pstScanner, Line1, angScan11, angScan12);

        // 得到下面Line2扫描角的允许范围
        angScan11 -= SIASUN_MATCHER_ANGLE_WINDOW;
        angScan12 += SIASUN_MATCHER_ANGLE_WINDOW;

        // 依次考察当前直线集合成员
        for(int j = 0; j < pCurLineFeatures->size(); j++)
        {
            CPnt ptOrigin(0, 0);
            CPnt ptFoot2;

            // 取得当前直线段
            CLineFeature& line2 = *pCurLineFeatures->at(j);

            if(line2.m_nWhichSideToUse == 6 || Line1.m_nWhichSideToUse == 6)
            {
                if(Line1.m_nWhichSideToUse != line2.m_nWhichSideToUse)
                    continue;
            }

            CPnt ptWorldStart = m_trans.GetWorldPoint(line2.m_ptStart);
            CPnt ptWorldEnd = m_trans.GetWorldPoint(line2.m_ptEnd);//line2.m_ptEnd;
            CPnt ptStart = line2.m_ptStart;//m_trans.GetLocalPoint(line2.m_ptStart);
            CPnt ptEnd = line2.m_ptEnd;//m_trans.GetLocalPoint(line2.m_ptEnd);

            if(!bIsPushOk)
            {
                CLine LineWorld(ptWorldStart, ptWorldEnd);
                vecLines_.push_back(LineWorld);
            }

            CLine Line2(ptStart, ptEnd);
            Line2.m_nId = j;

            float fLen2 = Line2.Length();

            // 计算当前姿态在参考线段上的垂直投影点ptFoot2，并取得投影线长度
            float fDist2 = Line2.DistanceToPoint(false, ptOrigin, NULL, &ptFoot2);

            // 构造从激光头到上述垂足点的直线段
            CLine lnToFoot2(ptOrigin, ptFoot2);

            // 计算从当前激头姿态到当前直线的(逆时针)转角
            CAngle ang2 = lnToFoot2.m_angSlant;

            CAngle angScan21, angScan22;

            // 计算线段两个端点到激光头的连线与激光头当前姿态的夹角
            CPosture cp(0, 0, 0);
            FindScanAngles(cp, Line2, angScan21, angScan22);

            // 距离变化量不超过400，角度变化不超过15度，且两个端点所在扫描角在允许范围之内
            if((fabs(fDist2 - fDist1) < SIASUN_MATCHER_DIST_WINDOW) &&
                    (ang1.GetDifference(ang2) < SIASUN_MATCHER_ANGLE_WINDOW) /*&&
                                    angScan21.InRange(angScan11, angScan12) &&
                                    angScan22.InRange(angScan11, angScan12)*/)       //20210510zjc
            {

                CAngle a = angScanner + (ang1 - ang2);
                // float m_fRad = ang1.m_fRad - ang2.m_fRad + angScanner.m_fRad;
                //CAngle a(m_fRad);

                CPnt ptFootStart, ptFootEnd;
                float fDistS = Line1.DistanceToPoint(false, ptWorldStart, NULL, &ptFootStart);
                float fDistE = Line1.DistanceToPoint(false, ptWorldEnd, NULL, &ptFootEnd);


                if((Line1.ContainPoint(ptFootStart) && Line1.ContainPoint(ptFootEnd))
                        /* 1.两个垂足均在直线内*/
                        ||(Line1.ContainPoint(ptFootStart) &&
                           (fabs(ptFootEnd.DistanceTo(Line1.m_ptStart)) < DistToLinePoint ||
                            fabs(ptFootEnd.DistanceTo(Line1.m_ptEnd)) < DistToLinePoint))
                        /* 2.其中一个垂足在直线内另一个在其中一个端点附近*/
                        ||(Line1.ContainPoint(ptFootEnd) &&
                           (fabs(ptFootStart.DistanceTo(Line1.m_ptStart)) < DistToLinePoint||
                            fabs(ptFootStart.DistanceTo(Line1.m_ptEnd)) < DistToLinePoint))
                        /* 3.其中一个垂足在直线内另一个在其中一个端点附近*/
                        ||((fabs(ptFootEnd.DistanceTo(Line1.m_ptStart)) < DistToLinePoint ||
                            fabs(ptFootEnd.DistanceTo(Line1.m_ptEnd)) < DistToLinePoint) &&
                           (fabs(ptFootStart.DistanceTo(Line1.m_ptStart)) < DistToLinePoint ||
                            fabs(ptFootStart.DistanceTo(Line1.m_ptEnd)) < DistToLinePoint)))
                    /* 4.两个垂足均在直线两个端点附近*/
                {

                    if(fLen2 >= 1.0f)//1000.0f
                    {
                        //                        std::cout << "PointCount: " << line2.m_nPointCount << std::endl;
                        Pair.Create(Line2, Line1, m_pstCur, a);
                        Pair.m_fMatchDiff = (fabs(fDist2 - fDist1));
                        Pair.m_fMatchDiffa = (ang1.GetDifference(ang2));
                        //                        float dis = 0.0f;
                        float dis = fabs(ptFoot2.DistanceTo(Line2.m_ptStart)) ?
                                    fabs(ptFoot2.DistanceTo(Line2.m_ptEnd)):
                                    fabs(ptFoot2.DistanceTo(Line2.m_ptStart)) <
                                    fabs(ptFoot2.DistanceTo(Line2.m_ptEnd));
                        if(Line2.ContainPoint(ptFoot2) || (dis/Line2.Length()) < 0.5f)
                            Pair.m_bFootInLine = TRUE;
                        if ((Pair.m_lnWorld.m_angSlant.m_fRad >= (PI  / 4) &&
                             Pair.m_lnWorld.m_angSlant.m_fRad < (PI * 3 / 4)) ||
                                (Pair.m_lnWorld.m_angSlant.m_fRad >= (PI * 5 / 4) &&
                                 Pair.m_lnWorld.m_angSlant.m_fRad < (PI * 7 / 4)))
                            m_LineMatchListY.AddInOrder(Pair);
                        else
                            m_LineMatchListX.AddInOrder(Pair);


                        //////////////////////
                        //                        std::cout << "/**************************/" << std::endl;
                        //                        std::cout <<"Wstart: "<< Line1.m_ptStart.x << " , " << Line1.m_ptStart.y << " , "
                        //                                 <<" end: "<< Line1.m_ptEnd.x << " , " << Line1.m_ptEnd.y<<" ,rad: "<<ang1.m_fRad
                        //                                <<" Lstart: "<< line2.m_ptStart.x << " , " << line2.m_ptStart.y << " , "
                        //                               <<" end: "<< line2.m_ptEnd.x << " , " << line2.m_ptEnd.y<<" ,rad: "<<ang2.m_fRad
                        //                              <<" len: "<<line2.Length() <<" line.count: "<<line2.m_nPointCount
                        //                             <<" dis: "<<fabs(fDist2 - fDist1)<<" difa: "<<(ang1.GetDifference(ang2))<< std::endl;
                        //                        std::cout << "/**************************/" << std::endl;
                        //////////////////////
                    }
//                    else if(((fLen2 >= 500.0f && fDist2 < 5000.0f) ||
//                             (fLen2 >= 300.0f && fDist2 < 3000.0f)) &&
//                            fabs(fDist2 - fDist1) < SIASUN_MATCHER_SHORTDIST_WINDOW)
                    else if(((fLen2 >= 0.5f && fDist2 < 5.0f) ||
                             (fLen2 >= 0.300f && fDist2 < 3.0f)) &&
                            fabs(fDist2 - fDist1) < SIASUN_MATCHER_SHORTDIST_WINDOW)
                    {// LJ 8.17;/
                        Pair.Create(Line2, Line1, m_pstCur, a);
                        Pair.m_fMatchDiff = (fabs(fDist2 - fDist1));
                        Pair.m_fMatchDiffa = (ang1.GetDifference(ang2));

                        if ((Pair.m_lnWorld.m_angSlant.m_fRad >= (PI  / 4) &&
                             Pair.m_lnWorld.m_angSlant.m_fRad < (PI * 3 / 4)) ||
                                (Pair.m_lnWorld.m_angSlant.m_fRad >= (PI * 5 / 4) &&
                                 Pair.m_lnWorld.m_angSlant.m_fRad < (PI * 7 / 4)))
                            m_LineMatchListY.AddInOrder(Pair);
                        else
                            m_LineMatchListX.AddInOrder(Pair);

                        //////////////////////
                        //                        std::cout <<"Wstart: "<< Line1.m_ptStart.x << " , " << Line1.m_ptStart.y << " , "
                        //                                 <<" end: "<< Line1.m_ptEnd.x << " , " << Line1.m_ptEnd.y
                        //                                <<" ,rad: "<<ang1.m_fRad
                        //                               <<" Lstart: "<< line2.m_ptStart.x << " , " << line2.m_ptStart.y << " , "
                        //                              <<" end: "<< line2.m_ptEnd.x << " , " << line2.m_ptEnd.y
                        //                             <<" ,rad: "<<ang2.m_fRad
                        //                            <<" len: "<<line2.Length() <<" line.count: "<<line2.m_nPointCount
                        //                           <<" dis: "<<fabs(fDist2 - fDist1)<<" difa: "<<(ang1.GetDifference(ang2)) << std::endl;
                        //////////////////////
                    }
                }
            }
        }
        bIsPushOk = true;
    }

    // 根据匹配直线提取角点
//    GenerateCornerAndEndPointByLML();

    // 最后对匹配表进行过滤处理，去除那些不合理的、或者是多重对应的项
    m_LineMatchListX.Filter();
    m_LineMatchListY.Filter();

    // Sort Lines By Distance and Line's Length
    //    m_LineMatchListX.SortLines();
    //    m_LineMatchListY.SortLines();
    AddLinePair();

//    if(pRefLineFeatures)
//    {
//        delete pRefLineFeatures;
        pRefLineFeatures = NULL;
//        m_pRefWorldLines = NULL;
//    }
    if(pCurLineFeatures)
    {
        delete pCurLineFeatures;
        pCurLineFeatures = NULL;
    }


    return true;
}

void CFeatureLocalization::AddLinePair()
{
    bool bPolar = true;
    int m = 0, n = 0;
    if(m_LineMatchListX.GetCount() > 3)
        m_LineMatchListX.m_nCount = 3;
    if(m_LineMatchListY.GetCount() > 3)
        m_LineMatchListY.m_nCount = 3;
    for (int i = 0; i < (m_LineMatchListX.GetCount() + m_LineMatchListY.GetCount()); i++)
    {
        if (bPolar == true)
        {
            if (n < m_LineMatchListX.GetCount())
            {
                m_LineMatchList.AddInOrder(m_LineMatchListX.GetAt(n));
                n++;
                bPolar = false;
                continue;
            }
            bPolar = false;
        }
        if (bPolar == false)
        {
            if (m < m_LineMatchListY.GetCount())
            {
                m_LineMatchList.AddInOrder(m_LineMatchListY.GetAt(m));
                m++;
                bPolar = true;
                continue;
            }
            bPolar = true;
        }
    }
}

//
//   提供匹配功能(成功时返回1，失败时返回负值)。
//匹配定位函数
int CFeatureLocalization::LocalizationPro(CScan* pCurScan, CPosture& pstNew, CPosture &pstEstimate)
{
    long long int locStart = GetTickCount();

    int iRet = -1;

    if(m_Param->isUseUseMultiFrameLoc && m_DequeReflectors.size() > (m_Param->iMultiFrameLocDequeSize - 1))
        iRet = FeatureLocalizationUseMultiframe(pCurScan, pstNew, pstEstimate);
    else
        iRet = FeatureLocalizationUseSingleframe(pCurScan, pstNew, pstEstimate);

    deque<stReflector> Reflectors;
    Reflectors.clear();
    for(int i = 0; i < ((CFeatureScan*)pCurScan)->m_ptHighBrightPnt.size(); i++)
    {
        stReflector ostReflector;
        ostReflector = ((CFeatureScan*)pCurScan)->m_ptHighBrightPnt.at(i);
        ostReflector.m_ptWorld = ostReflector.m_ptLocal;
        if(iRet == FEATURE_MATCH_SUCCESS)
            ostReflector.m_ptWorld.InvTransform(pstNew);
        else
            ostReflector.m_ptWorld.InvTransform(m_pstOdRe);

        Reflectors.push_back(ostReflector);
    }
    m_DequeReflectors.push_back(Reflectors);

    if(m_DequeReflectors.size() > m_Param->iMultiFrameLocDequeSize)
        m_DequeReflectors.pop_front();

    long long int locEnd = GetTickCount();

//    std::cout<<"LocalizationPro:"<<(locEnd-locStart)<<std::endl;

    return iRet;
}

int CFeatureLocalization::FeatureLocalizationUseMultiframe(CScan *pCurScan, CPosture &pstNew, CPosture &pstEstimate)
{
        ClearMatchList();
        /*****************************反光板匹配*****************************/
        // 在m_Param->maxRange范围内挑选出点特征参考集合
        if(m_Param->onlyUseInRectFeature)
            m_RefLayer.CreateFromOtherSet(*m_pWorldLayer, m_pstOld, m_Param->maxRange, m_Param->vecSpecialPntList_, localizationRect_);
        else
            m_RefLayer.CreateFromOtherSet(*m_pWorldLayer, m_pstOld, m_Param->maxRange, m_Param->vecSpecialPntList_);

        if(m_RefLayer.size() > 0)
            ((CFeatureScan *)pCurScan)->FindHighBrightPnt(); // 生成点特征

        long long int locStart = GetTickCount();
        m_LocalLayer.Clear();
        for(int i = 0; i < m_DequeReflectors.size(); i++)
        {
            deque<stReflector> & ReflectorTemp = m_DequeReflectors.at(i);
            for(int j = 0; j < ReflectorTemp.size(); j++)
            {
                stReflector &stReflectorTemp = ReflectorTemp.at(j);
                // 世界系坐标　转为　本地坐标
                CPnt PntTemp(stReflectorTemp.m_ptWorld.x, stReflectorTemp.m_ptWorld.y);
                PntTemp.Transform(m_pstOld);
                // 构造扫描线
                CLine ln(((CFeatureScan *)pCurScan)->m_poseScanner.GetPntObject(), PntTemp);
                // 计算扫描角
                CAngle ang = ln.SlantAngle() - ((CFeatureScan *)pCurScan)->m_poseScanner.GetAngle();
                PntTemp.r = ln.Length();
                PntTemp.a = CAngle::NormAngle2(ang.m_fRad);

                stReflectorTemp.m_ptLocal = PntTemp;
            }
        }
        long long int locEnd = GetTickCount();
//        std::cout << "m_DequeReflectors times:" << (locEnd - locStart) << std::endl;

        for(int i = 0; i < ((CFeatureScan *)pCurScan)->m_ptHighBrightPnt.size(); i++)
        {
            //加入m_LocalLayer
            AddPntToLocalLayer(((CFeatureScan *)pCurScan)->m_ptHighBrightPnt.at(i).m_ptLocal);
        }

        m_fScore = -1.0f;
        if(m_nWorkMode == QUICK_MAP_MODE)
        {
            // 进行快速配准，结果保存于m_PointMatchList中
            if(!PointFeatureQuickMatchUseDequeReflectors(pCurScan))
            {
                if(m_Param->isUseSinglePointLoc)
                {
                    if(m_PointMatchList.GetCount() > 1)
                        return LocalizationWithDoublePoint(&pstNew, pstEstimate);
                    if(m_PointMatchList.GetCount() > 0)
                        return LocalizationWithSinglePoint(&pstNew, pstEstimate);
                    else
                        return LocalizationWithCorridor(&pstNew, pstEstimate);
                }
                else
                    return FEATURE_MATCH_POINTS_NOT_ENOUGH;
            }
        }
        else
            return FEATURE_MATCH_UNKOWN_ERROR;

        return LocalizationDistribution(&pstNew); //解算定位
}

int CFeatureLocalization::FeatureLocalizationUseSingleframe(CScan *pCurScan, CPosture &pstNew, CPosture &pstEstimate)
{
    ClearMatchList();
    /*****************************直线匹配 **********************************/

    if(m_nWorkMode == QUICK_MAP_MODE)
    {
        if(GetRefWorldLinesNum() > 0) // 如果参考集中有直线特征
        {
            m_pRefWorldLines = new CLineFeatureSet;
            //在m_Param->maxRange范围内挑选出线特征参考集合
            if(m_Param->onlyUseInRectFeature)
            {
               m_pRefWorldLines->CreateFromLinesSet(m_pWorldLines, m_pstOld, m_Param->maxRefLineRange,m_Param->vecSpecialLineList_,localizationRect_);
            }
            else
            {
                m_pRefWorldLines->CreateFromLinesSet(m_pWorldLines, m_pstOld, m_Param->maxRefLineRange,m_Param->vecSpecialLineList_);
            }
            if(m_pRefWorldLines->size() > 0) // 如果参考集中没有直线特征就不往下运行了
            {
                 ((CFeatureScan*)pCurScan)->CreateLineFeatures(); // 生成直线特征集合

                if (!MatchLineFeatures(pCurScan)) // 从扫描得到的点云数据中提取可能的直线特征数据  构建直线匹配对
                    return -1;
            }
            if(m_pRefWorldLines)
            {
                delete m_pRefWorldLines;
                m_pRefWorldLines = NULL;
            }
        }
        /***************************** 货架腿临时借用,目前单帧定位只进行货架腿定位 *****************************************************/
        // 生成点特征
        ((CFeatureScan *)pCurScan)->CreateLegFeatures();

        m_LocalLayer.Clear();
        if (!CreatLocalLayerFromCurScan(pCurScan))
            return FULL_MAP_ERROR_NO_MATCH;

        // 解算中心位置
        std::cout << "By Sam: Reflector point size = " << m_LocalLayer.GetCount() << std::endl;
        if (m_LocalLayer.GetCount() >= 2)
        {
           // PointFeatureFindLegPairs(0.335);  // By Sam: need add a param
            PointFeatureFindLegPairs(0.445);

            return LocalizationByLeg(&pstNew);
        }
        else
        {
            return FEATURE_MATCH_POINTS_NOT_ENOUGH;
        }

        return FULL_MAP_ERROR_NO_MATCH;
    }
    /*****************************反光板匹配*****************************************************/
    //在m_Param->maxRange范围内挑选出点特征参考集合
    if(m_Param->onlyUseInRectFeature)
        m_RefLayer.CreateFromOtherSet(*m_pWorldLayer, m_pstOld, m_Param->maxRange,m_Param->vecSpecialPntList_,localizationRect_);
    else
        m_RefLayer.CreateFromOtherSet(*m_pWorldLayer, m_pstOld, m_Param->maxRange,m_Param->vecSpecialPntList_);

    if(m_RefLayer.size() > 0)
        ((CFeatureScan *)pCurScan)->CreatePointFeaturesNew(); //生成点特征

    if(!CreatLocalLayerFromCurScan(pCurScan))
        return FULL_MAP_ERROR_NO_MATCH;

    m_fScore = -1.0f;
    if(m_nWorkMode == FULL_MAP_MODE)
    {
        ULONG tick = GetTickCount();
        for(int i = 0; i < LINE_COMPARE_COUNT; i++)
        {
            if(PointFeatureFullMapMatch(m_Param->fLineEqualLimit_[i]))
                break;
            if((GetTickCount() - tick) > 800) //全局匹配超时返回
                break;
        }
        // 如果没发现匹配，定位失败
        if(m_MatchTabSet.count == 0)
            return FULL_MAP_ERROR_NO_MATCH; //找到不到匹配集
    }
    else if(m_nWorkMode == QUICK_MAP_MODE)
    {
        if(!PointFeatureQuickMatch()) //进行快速配准，结果保存于m_PointMatchList中
        {
            if(m_Param->isUseSinglePointLoc)
            {
                if(m_PointMatchList.GetCount() > 1)
                    return LocalizationWithDoublePoint(&pstNew, pstEstimate);
                if(m_PointMatchList.GetCount() > 0)
                    return LocalizationWithSinglePoint(&pstNew, pstEstimate);
                else
                    return LocalizationWithCorridor(&pstNew, pstEstimate);
            }
            else
                return FEATURE_MATCH_POINTS_NOT_ENOUGH;
        }
    }
    else
        return FEATURE_MATCH_UNKOWN_ERROR;

    return LocalizationDistribution(&pstNew); //解算定位
}

void CFeatureLocalization::ClearMatchList()
{
    m_LineMatchList.Clear();
    m_LineMatchListX.Clear();
    m_LineMatchListY.Clear();
    m_PointMatchList.Clear();
}

void CFeatureLocalization::CopyMatchPair(CFeatureMatchInfo  &MatchInfo)
{
    MatchInfo.vecPointPair_.clear();
    MatchInfo.vecLinePair_.clear();
    for(int i = 0 ; i < m_PointMatchList.GetCount() ; i++)
    {
        MatchInfo.vecPointPair_.push_back(m_PointMatchList.GetAt(i));
    }
    for(int i = 0 ; i < m_LineMatchList.GetCount();i++)
    {
        MatchInfo.vecLinePair_.push_back(m_LineMatchList.GetAt(i));
    }
}

void CFeatureLocalization::CopyIdentifierFeature(CFeatureMatchInfo  &MatchInfo)
{
    MatchInfo.vecReflectors_.clear();
    for(int i  = 0 ; i <m_LocalLayer.size() ; i++ )
    {
        CPnt t(m_LocalLayer.at(i)->x,m_LocalLayer.at(i)->y);
        t.InvTransform(m_pstOdRe);
        MatchInfo.vecReflectors_.push_back(t);
    }
    MatchInfo.vecLines_.clear();
    for(int i  = 0  ; i < vecLines_.size(); i ++)
    {
        MatchInfo.vecLines_.push_back(vecLines_.at(i));
    }
    vecLines_.clear();
}
//
//   设置经匹配运算校正后的姿态。
//
void CFeatureLocalization::SetCorrectedPosture(const CPosture& pst)
{
    CScanMatcher::SetCorrectedPosture(pst);

    m_pstOld = m_pstCur = m_pstEstimate = pst;
}

long CFeatureLocalization::GetRefWorldLinesNum()
{
    return m_pWorldLines->size();
}

void CFeatureLocalization::SwitchRatio(const CPosture& pos)
{
    for(int i = 0; i < SpecialRatioSet.size(); i++)
    {
        if(pos.x > SpecialRatioSet[i].m_fXmin &&
                pos.x < SpecialRatioSet[i].m_fXmax &&
                pos.y > SpecialRatioSet[i].m_fYmin &&
                pos.y < SpecialRatioSet[i].m_fYmax)
        {
            m_fRatio = SpecialRatioSet[i].m_fRatio;
            cout << "m_fRatio: " << m_fRatio << endl;
            return;
        }
    }
    m_fRatio = 60.0f;
}
