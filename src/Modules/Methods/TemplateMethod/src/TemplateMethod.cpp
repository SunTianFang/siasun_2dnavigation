#include "stdafx.h"
#include "TemplateMethod.h"
#include "AffinePosture.h"

#include "LineElement.h"
#include "CircleElement.h"
#include "BasObject.h"
#include <iostream>

#include "RoboLocClnt.h"
#include "AutoOutPutBlackBox.h"
#include "blackboxhelper.hpp"
#include "ParameterObject.h"
#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

extern bool readJffFormat;

///////////////////////////////////////////////////////////////////////////////
//   实现基于模板的定位方法。

CTemplateMethod::CTemplateMethod()
{
    type_ = 2;

/*
    // 需要根据不同激光器（线数）构造CScan类
    int rayNum = 3300;
    sourceScan_ = new CScan(rayNum);   //3300
    if (sourceScan_ == NULL)
        return;

    targetScan_ = new CScan(rayNum);
    if (targetScan_ == NULL)
        return;

    clusterSourceScan_ = new CScan(rayNum);
    if (clusterSourceScan_ == NULL)
        return;

    filterTargetScan_ = new CScan(rayNum);
    if (filterTargetScan_ == NULL)
        return;
*/
    // 初始化给NULL，具体定位时再根据激光param线数构造
    sourceScan_ = NULL;
    targetScan_ = NULL;
    clusterSourceScan_ = NULL;
    filterTargetScan_ = NULL;
    lastScan_ = NULL;
    viewTargetScan_ = NULL;

    lastPose_.Create(0, 0, 0);
    viewLastPose_.Create(0, 0, 0);

    map_ = CreateMap();

    Initialize();
}

CTemplateMethod::~CTemplateMethod()
{
    if (map_ != NULL)
        delete map_;
}

bool CTemplateMethod::UnloadMap()
{
    //dq 9.2
    /*
    if (map_ != NULL)
    {
        delete map_;
         map_ = NULL;
    }
    */
}


CStaticObjects *CTemplateMethod::CreateMap()
{
    return new CStaticObjects;
}

bool CTemplateMethod::Initialize()
{
    // 聚类半径
    clusterMaxDist_ = 0.9; //1.2;
//    clusterMaxDist_ = param_.templateRatio;
    point2LineDist_ = 0.05;
    // 质量评估标准
    p2lError_ = 0.008;                   // 点线距离(m)，即source点到其target中对应点对线段距离
    p2pError_ = 0.01;                    // 点点距离(m)，即对应点对间距离
    validPercent_ = 0.38;               // 匹配对点数比例（匹配点数占合格扫描线数的百分比）,可以开放参数，设置质量评估
    validNum_ = 15;                     // 匹配点数个数，可以开放参数，设置质量评估

    tolerance = 0;

    firstIn = false;

    // 初始化模板点
    m_pPointTemplate_ = CreateTemplatePointsSet();
    if (m_pPointTemplate_ == NULL)
        return false;
//    m_pPointTemplate_ = NULL;

    return true;
}

//
//   生成一个适用于本方法的定位参数块。
//
CLocalizationParam *CTemplateMethod::CreateLocParam()
{
    return new CTemplateLocalizationParam;
}

//
//   应用指定的定位参数。
//
bool CTemplateMethod::ApplyParam(const CLocalizationParam *p)
{
    if (p == NULL)
        return false;

    // 将参数复制到本对象中，以方便应用
    param_ = *((CTemplateLocalizationParam *)p);
    return true;
}

//
//   对应于该定位方法的定位流程。
//

/** jzz:支持多模板物体导航
 * step1:获取使用定位的所有模板
 * step2:（目前虚拟扫描只支持直线）
 * step3:所有静态物体以直线为单位，重新组成新的静态物体
 * step4:对（组成）的大模板做target，source
 * step5:csm匹配
 */
bool CTemplateMethod::LocalizeProc(int localMode, const Eigen::Affine3d &initPose,
                                   const ndt_oru::CStampedPointCloud cloudIn,
                                   Eigen::Affine3d &estimatePose)
{
    // 构造点云
    int rayNum = 0;
    for (int i = 0; i < (int)m_pScannerGroupParam->size(); i++)
    {
        if(i == 0)
            rayNum += m_pScannerGroupParam->at(i).m_nLineCount;    // 前后头线数加在一起
        cout << "rayNum: " << rayNum << endl;
    }

    if(NULL == targetScan_)
    {
        targetScan_ = new CScan(rayNum);
    }
    else
        targetScan_->Create(rayNum);

    if(NULL == sourceScan_)
    {
        sourceScan_ = new CScan(rayNum);
    }
    else
        sourceScan_->Create(rayNum);

    if(NULL == filterTargetScan_)
    {
        filterTargetScan_ = new CScan(rayNum);
    }
    else
        filterTargetScan_->Create(rayNum);

    if(NULL == clusterSourceScan_)
    {
        clusterSourceScan_ = new CScan(rayNum);
    }
    else
        clusterSourceScan_->Create(rayNum);

    // 上一帧点云，用于帧间匹配，作为target
    if(NULL == lastScan_)
    {
        lastScan_ = new CScan(rayNum);
    }

    // 由于lastScan_会被覆盖掉，因此用viewTargetScan_单独用作显示用;viewLastPose同理
    if (NULL == viewTargetScan_)
    {
        viewTargetScan_ = new CScan(rayNum);
    }

    if (!vObjTS_.empty())
        vObjTS_.clear();

    CPosture pstInit = AffineToPosture(initPose);

    CBasObject *allObj = new CBasObject;
    for (int objNum = 0; objNum < param_.useTemplateNum; objNum++)
    {        
        objTargetSource tmpObjTS;
        // 取得指向参考物体对象的指针
        string str = param_.objNames[objNum];

        int index = map_->FindIndexByName(str);
        if (index < 0)
            return false;

        CBasObject *obj = map_->at(index);

        // jzz:获取模板特征的几何中心
        vector<CPnt> mLineCenter;
        for (int i = 0; i < (int)obj->size(); i++)
        {
             CLineElement *le = dynamic_cast<CLineElement *>(obj->at(i));
             // 目前，必须是直线元素
             if (le == NULL)
                 continue;
             allObj->push_back(le);
             CPnt tmpLineCenter = le->m_line.GetMidpoint();
             std::cout << "i: " << i << "tmpLineCenter x: " << tmpLineCenter.x << " " << "y: " << tmpLineCenter.y << std::endl;
             mLineCenter.push_back(tmpLineCenter);          // 获取组成该模板形状的每条线段的中心点
        }

        double point_x = 0 ,point_y = 0;
        CPnt templateCenter(0, 0);
        for (int i = 0; i < (int)mLineCenter.size(); i++)
        {
            point_x += mLineCenter[i].x;
            point_y += mLineCenter[i].y;

            if (i == (int)mLineCenter.size() -1)
            {
                point_x /= (int)mLineCenter.size();
                point_y /= (int)mLineCenter.size();
            }
        }
        // 获取模板的中心点坐标:templateCenter
        templateCenter.x = point_x;
        templateCenter.y = point_y;
//        std::cout << "mLineCenter size: " << mLineCenter.size() << std::endl;
//        std::cout << "template center point x: " << point_x << " " << "y: " << point_y << std::endl;
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "CTemplateMethod mLineCenter size: = ",mLineCenter.size(), ", template center point: ", point_x, ", ",point_y);
#endif
        tmpObjTS.centerPoint = templateCenter;
        vObjTS_.push_back(tmpObjTS);

        vector<CPnt>().swap(mLineCenter);
    }
    // 生成目标点云数据
//    allObj->CreateScanPointCloud(lastPose_, -PI, PI, 3600, *targetScan_);  // 原版写法（不用）。基于上次定位结果生成target点云数据，此处生成的target是局部坐标系
    float endAngle = /*-PI;*/  m_pScannerGroupParam->at(0).m_fEndAngle;
    float startAngle = /*PI;*/ m_pScannerGroupParam->at(0).m_fStartAngle;
    /** 1:拆点法生成target点云； 0:几何法生成target点云  (一共需要更改3处)*/
#if 1
    ////// 拆点，生产target点云，此处target点是map坐标系
    /// 使用拆点法，注意target的位姿给（0,0,0)
    int targetScanPointN = 0;
    for (int i = 0; i < (int)allObj->size(); i++)
    {
         const CLineElement *line = dynamic_cast<const CLineElement *>(allObj->at(i));
         float resolution = 0.001;
//         float angle = line->m_line.SlantAngleRad();  // 获取线段倾斜角
         while (resolution < line->m_line.Length() )
         {
             CPnt pt = line->m_line.TrajFun(resolution);
             CScanPoint scanPt;
             scanPt.Set(pt.x, pt.y);
             scanPt.UpdatePolar();

             targetScan_->m_pPoints[targetScanPointN] = scanPt;
             targetScanPointN++;
             resolution += 0.01;   // 拆点分辨率 m
         }
    }
#else
    // 仅用前头做虚拟扫描，前后头合并用有问题！  几何法生成target，用拆点法时注释掉。
    allObj->CreateScanPointCloud(lastPose_, startAngle, endAngle, rayNum, *targetScan_);  // 对组合的“大”模板生成target点云
#endif
    targetScan_->m_pstScanner = lastPose_;
    targetScan_->m_fStartAng = startAngle;
    targetScan_->m_fEndAng = endAngle;


    // 生成源点云数据(将以CStampledPointCloud形式描述的点云转换为以CScan形式描述的点云)
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "CTemplateMethod cloudIn size: ", cloudIn.size());
#endif
//    cout << "cloudIn size: " << cloudIn.size() << endl;
    cloudIn.ToScan(sourceScan_);
    sourceScan_->InvTransform(pstInit);
    sourceScan_->m_pstScanner = pstInit;                                 // 基于当前位姿获取source点云数据

    sourceScan_->m_bVisible = targetScan_->m_bVisible;
    sourceScan_->m_poseRelative = targetScan_->m_poseRelative;
    sourceScan_->m_fStartAng = targetScan_->m_fStartAng;
    sourceScan_->m_fEndAng = targetScan_->m_fEndAng;
    // jzz: 对sourceScan点云进行聚类筛选，只保留模板附近的点云进行匹配 (都在map坐标系)
    int clusterSourceScanN = 0;
   // cout << "clusterMaxDist_: " << clusterMaxDist_ << endl;
    for (int i = 0; i < sourceScan_->m_nCount; i++)
    {
        CPnt tmpPoint;
        tmpPoint.x = sourceScan_->m_pPoints[i].x;
        tmpPoint.y = sourceScan_->m_pPoints[i].y;
        tmpPoint.UpdatePolar();
        // 计算sourceScan每个点到模板中心的距离，遍历所有中心点，计算到所有中心点的距离，求出最近距离，并判断该距离是否满足阈值。
        float dist = std::numeric_limits<float>::max();
        for (int j = 0; j < (int)vObjTS_.size(); j++)
        {
            // dist比较,取最近的
            float d = tmpPoint.DistanceTo(vObjTS_[j].centerPoint);
            if (d < dist)
                dist = d;
        }

        // 生成聚类筛选后的点云
        if (dist < clusterMaxDist_)
        {
            clusterSourceScan_->m_pPoints[clusterSourceScanN] = sourceScan_->m_pPoints[i];
            clusterSourceScanN++;
        }
    }
//std::cout<<"clusterSourceScanN"<<clusterSourceScanN<<std::endl;
    if (clusterSourceScanN == 0)
        return false;

    clusterSourceScan_->m_nCount = clusterSourceScanN;
    clusterSourceScan_->m_pstScanner = sourceScan_->m_pstScanner;

    clusterSourceScan_->m_bVisible = targetScan_->m_bVisible;
    clusterSourceScan_->m_poseRelative = targetScan_->m_poseRelative;
    clusterSourceScan_->m_fStartAng = targetScan_->m_fStartAng;
    clusterSourceScan_->m_fEndAng = targetScan_->m_fEndAng;

    vector<objTargetSource>().swap(vObjTS_);

    // 对target点云进行筛选，保留与模板物体扫描到的点
    int filterTargetScanN = 0;
    for (int i = 0; i < targetScan_->m_nCount; i++)
    {
        if (targetScan_->m_pPoints[i].r < 0.01f)
            continue;
        filterTargetScan_->m_pPoints[filterTargetScanN] = targetScan_->m_pPoints[i];
        filterTargetScanN++;
    }

#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "CTemplateMethod filterTargetScanN: ", filterTargetScanN);
#endif
    if (filterTargetScanN == 0)
    {
        cout << "filterTargetScanN: " << filterTargetScanN << endl;
//        return false;   // 有bug，lastPose_ 一直为0，导致上面的target无法生产
    }

    filterTargetScan_->m_nCount = filterTargetScanN;
    filterTargetScan_->m_pstScanner = targetScan_->m_pstScanner;

    filterTargetScan_->m_bVisible = targetScan_->m_bVisible;
    filterTargetScan_->m_poseRelative = targetScan_->m_poseRelative;
    filterTargetScan_->m_fStartAng = targetScan_->m_fStartAng;
    filterTargetScan_->m_fEndAng = targetScan_->m_fEndAng;
#if 0
    cout << "sourceScan_ size: " << sourceScan_->m_nCount << endl;
    cout << "clusterSourceScan_ size: " << clusterSourceScan_->m_nCount << endl;
    cout << "targetScan size: " << targetScan_->m_nCount << endl;
    cout << "filterTargetScan_ size: " << filterTargetScan_->m_nCount << endl;
    cout << "filterTargetScan_->m_pstScanner: " << filterTargetScan_->m_pstScanner.x << filterTargetScan_->m_pstScanner.y << filterTargetScan_->m_pstScanner.fThita << endl;
    cout << "clusterSourceScan_->m_pstScanner: " << clusterSourceScan_->m_pstScanner.x << clusterSourceScan_->m_pstScanner.y << clusterSourceScan_->m_pstScanner.fThita << endl;
#endif
    CFrame frame;
    CCorrList corrList;
    sm_result result;
    // 将带匹配的点云（target，source）变换到激光坐标系下
    clusterSourceScan_->Transform(pstInit);        // ??执行Transform函数会使m_pstScanner变为0??
//    sourceScan_->Transform(pstInit);

    // 进行坐标变换后，filterTragetScan、clusterSourceScan的m_pstScanner变为0，需要重新赋值。此处赋值决定csm的初始位姿估计。不重新赋值，此处值变为0，虽然不影响计算，但是csm收敛范围受到影响
    // 会在特征不足时（扫描的点x,y方向不均匀）出现误匹配，此处从新赋值，初始位姿估计不为0（为上次匹配结果）鲁棒性提升。

    filterTargetScan_->m_pstScanner = targetScan_->m_pstScanner; // 用于正常交点法生成target计算模板匹配，此时的target是局部坐标
    clusterSourceScan_->m_pstScanner = sourceScan_->m_pstScanner;

    //////////////////////////////////////////////////
    // 初始帧情况不定位，获取lastPose，从第2帧开始进行csm
    if (!firstIn)
    {
        matchInfo_.result_ = CMatchInfo::MATCH_FAIL;
        matchInfo_.pst_ = pstInit;
        lastPose_ = pstInit;

        matchInfo_.matchNum_ = 100;
        matchInfo_.matchRatio_ = 100;
        estimatePose = PostureToAffine(lastPose_);

//        lastScan_->m_nCount = clusterSourceScan_->m_nCount;
//        lastScan_->m_bVisible = clusterSourceScan_->m_bVisible;
//        lastScan_->m_poseRelative = clusterSourceScan_->m_poseRelative;
//        lastScan_->m_fStartAng = clusterSourceScan_->m_fStartAng;
//        lastScan_->m_fEndAng = clusterSourceScan_->m_fEndAng;
//        for (int i = 0; i < clusterSourceScan_->m_nCount; i++)
//        {
//            lastScan_->m_pPoints[i] = clusterSourceScan_->m_pPoints[i];
//        }
//        lastScan_->m_pstScanner = lastPose_;
        firstIn = true;
        cout << "firstIn: " << firstIn << endl;

        return firstIn;
    }
    //    cout << "lastPose_ x: " << lastPose_.x << " " << "y: " << lastPose_.y << " " << "theta: " << lastPose_.fThita << endl;
    /////////////////////////////////////////////////

#if 1   // 1:模板匹配，或者帧间匹配模式（都是相邻帧做csm）; 0:建图模式，当前帧与累计的map做csm
#if 1   // 1: 模板匹配; 0: 帧间匹配
    // 模板匹配
    // 拆点计算时打开,几何法注释掉
    filterTargetScan_->m_pstScanner.Create(0,0,0);  // 用于拆点计算模板匹配，因为拆点是全局坐标系，因此target的坐标给(0,0,0），注意下面对应计算出的pose不是位姿变化量，而是真实的source位置
    long long int match_start_time = GetTickCount();
    bool ok = clusterSourceScan_->PointCloudMatch(*filterTargetScan_, frame, corrList, &result);   // 使用筛选后的target，source点云做csm匹配
    long long int match_end_time = GetTickCount();
    // csm match time (ms)
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "Template Loc Time = ",static_cast<int>(match_end_time - match_start_time));
#endif
        //std::cout<< "time: " << match_end_time - match_start_time << std::endl;
//    viewLastPose_ = lastPose_;  // 此处的lastPose_是上一帧的位姿，赋值给viewLastPose_用做显示
#else
    /// 帧间匹配测试
    bool ok = clusterSourceScan_->PointCloudMatch(*lastScan_, frame, corrList, &result);   // 使用筛选后的target，source点云做csm匹配

    // 用于（帧间匹配）显示用，更新上一帧点云（target）

    viewTargetScan_->m_nCount = lastScan_->m_nCount;
    viewTargetScan_->m_pstScanner = lastScan_->m_pstScanner;
    viewTargetScan_->m_bVisible = lastScan_->m_bVisible;
    viewTargetScan_->m_poseRelative = lastScan_->m_poseRelative;
    viewTargetScan_->m_fStartAng = lastScan_->m_fStartAng;
    viewTargetScan_->m_fEndAng = lastScan_->m_fEndAng;
    for (int i = 0; i < lastScan_->m_nCount; i++)
    {
        viewTargetScan_->m_pPoints[i] = lastScan_->m_pPoints[i];
    }

    viewLastPose_ = lastPose_;  // 此处的lastPose_是上一帧的位姿，赋值给viewLastPose_用做显示

    // 用于匹配用的，更新上一帧点云信息，当前帧作为target，与下一帧匹配
    lastScan_->m_nCount = clusterSourceScan_->m_nCount;
    lastScan_->m_bVisible = clusterSourceScan_->m_bVisible;
    lastScan_->m_poseRelative = clusterSourceScan_->m_poseRelative;
    lastScan_->m_fStartAng = clusterSourceScan_->m_fStartAng;
    lastScan_->m_fEndAng = clusterSourceScan_->m_fEndAng;
    for (int i = 0; i < clusterSourceScan_->m_nCount; i++)
    {
        lastScan_->m_pPoints[i] = clusterSourceScan_->m_pPoints[i];
    }

    ///
#endif
    // 如果匹配成功，则填写estimatePose
    // TODO:帧间匹配,生成点云地图, 目前只适用地图中存在一个模板定位区域的情况，如果有多个模板区域会出错，需要增加参数来区分区域。

    matchInfo_.error = result.error;
    matchInfo_.error2 = result.error2;
    matchInfo_.nvalid = result.nvalid;
    matchInfo_.correspondencePercent = result.correspondencePercent;
    matchInfo_.histCorrelation = result.histCorrelation;
    matchInfo_.valid_percent = result.valid_percent;

    CPosture pstResult(result.x[0], result.x[1], CAngle::NormAngle(result.x[2]));
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "Template Loc Result = ",result.x[0],", ",result.x[1],", ",CAngle::NormAngle(result.x[2]),", ok = ",ok);
#endif
    // 如果csm匹配成功了，再进行pose的核查判断，csm如果匹配失败，则表示定位失败了，不用再核查
    if (ok)
    {
        ok = MotionFilter(pstResult);   //by lishen
        tolerance = 0;
    }
    else
    {
        // 如果csm匹配失败，尝试定位（盲走）几帧，如果一直失败则认为丢导航
        pstResult = pstInit;
        tolerance++;
        ok = true;

        if (tolerance > 5)
        {
            ok = false;
            tolerance = 0;
        }
    }

    if (ok)
    {
//        CPosture pstResult(result.x[0], result.x[1], result.x[2]);
        // pstResult是csm匹配的位姿变化量，需要与lastPose进行累加得到当前位姿
#if 0   // 几何法:1,拆点法0
        lastPose_.x += pstResult.x * cos(lastPose_.fThita) - pstResult.y * sin(lastPose_.fThita);
        lastPose_.y += pstResult.y * cos(lastPose_.fThita) + pstResult.x * sin(lastPose_.fThita);
        lastPose_.fThita = CAngle::NormAngle(pstResult.fThita + lastPose_.fThita);
        // 帧间匹配、几何法模板匹配用上面的，模板拆点匹配用下面。由于使用拆点法生产target点云，因此target是全局坐标系，原理同下面建图模式，即pstResult就是当前位姿，不需要转换、累加。
#else
        lastPose_.x = pstResult.x;
        lastPose_.y = pstResult.y;
        lastPose_.fThita = CAngle::NormAngle(pstResult.fThita);
#endif
        estimatePose = PostureToAffine(lastPose_);
        matchInfo_.result_ = CMatchInfo::MATCH_OK;
//        matchInfo_.pst_ = pstResult;
        matchInfo_.pst_ = lastPose_;
        matchInfo_.matchNum_ = 100;
        matchInfo_.matchRatio_ = 100;

        //////帧间测试
//        lastScan_->m_pstScanner = lastPose_;
        //////

        // 匹配成功，则累计点云地图（帧间匹配建图），显示在RoboMappintgView.cpp中，此处只累计帧
//        clusterSourceScan_->InvTransform(lastPose_); // 转换至世界坐标系（根据当前定位的位姿）
//        scanMap_.push_back(*clusterSourceScan_);

        return true;
    }
    else
    {
        // 如果定位失败，将当前pstInit（里程位姿）给lastPose
        matchInfo_.result_ = CMatchInfo::MATCH_FAIL;
        matchInfo_.pst_ = pstInit;
        lastPose_ = pstInit;

        matchInfo_.matchNum_ = 0;
        matchInfo_.matchRatio_ = 0;
        estimatePose = PostureToAffine(lastPose_);

        ///帧间测试
//        lastScan_->m_pstScanner = lastPose_;
        ////

        return false;
    }

#else
    // TODO: 仅用前头建图，提升准确性
    bool ok = 0;

    if (!scanMap_.empty())
    {
        int pointsNum = scanMap_.size() * sourceScan_->m_nCount;
        CScan *pointsMap = new CScan(pointsNum);
        pointsMap->m_nCount = pointsNum;
        pointsMap->m_bVisible = sourceScan_->m_bVisible;
        pointsMap->m_poseRelative = sourceScan_->m_poseRelative;
        pointsMap->m_fStartAng = sourceScan_->m_fStartAng;
        pointsMap->m_fEndAng = sourceScan_->m_fEndAng;
        pointsMap->m_pstScanner.Create(0, 0, 0);        // 注意！！此时target的位姿是0！！
        int np = 0;
        for (int i = 0; i < (int)scanMap_.size(); i++)
        {
            for (int j = 0; j < scanMap_.at(i).m_nCount;/*clusterSourceScan_->m_nCount;*/ j++)
            {
                pointsMap->m_pPoints[np] = scanMap_.at(i).m_pPoints[j];
                np++;
            }
        }
        // 建图时source坐标系不用转换到全局(map)下，csm内部会转换，target需要转换到全局坐标系下，注意r（极径）也要转
        ok = clusterSourceScan_->PointCloudMatch(*pointsMap, frame, corrList, &result);   // 使用筛选后的target，source点云做csm匹配

        if (NULL == viewTargetScan_)
            viewTargetScan_ = new CScan(pointsNum);
        else
            viewTargetScan_->Create(pointsNum);

        viewTargetScan_->m_nCount = pointsMap->m_nCount;
        viewTargetScan_->m_pstScanner = pointsMap->m_pstScanner;
        viewTargetScan_->m_bVisible = pointsMap->m_bVisible;
        viewTargetScan_->m_poseRelative = pointsMap->m_poseRelative;
        viewTargetScan_->m_fStartAng = pointsMap->m_fStartAng;
        viewTargetScan_->m_fEndAng = pointsMap->m_fEndAng;
        float sampleDown = 0.5;   // 降采样系数
        for (int i = 0; i < pointsMap->m_nCount; i++)
        {
            int j = i * sampleDown;
            viewTargetScan_->m_pPoints[j] = pointsMap->m_pPoints[j];
        }

        pointsMap->Clear();

        viewLastPose_ = lastPose_;  // 此处的lastPose_是上一帧的位姿，赋值给viewLastPose_用做显示

        if (scanMap_.size() > 100)
            vector<CScan>().swap(scanMap_);

    }

    if (ok)
    {
        CPosture pstResult(result.x[0], result.x[1], result.x[2]);
        // 此时pstResult是csm匹配的位姿，不需要与lastPose进行累加得到当前位姿，因为target的m_pstScanner是（0,0,0），source的m_pstScanner是当前位姿估计（上一帧位姿）
        lastPose_.x = pstResult.x;
        lastPose_.y = pstResult.y;
        lastPose_.fThita = CAngle::NormAngle(pstResult.fThita);

        estimatePose = PostureToAffine(lastPose_);
        matchInfo_.result_ = CMatchInfo::MATCH_OK;
        matchInfo_.pst_ = lastPose_;
        matchInfo_.matchNum_ = 100;
        matchInfo_.matchRatio_ = 100;

        // 匹配成功，则累计点云地图
        clusterSourceScan_->InvTransform(lastPose_); // 转换至世界坐标系（根据当前定位的位姿）
        scanMap_.push_back(*clusterSourceScan_);     // 生成target(map)点云，需转至世界坐标系

        return true;
    }
    else
    {
        // 如果定位失败，将当前pstInit（里程位姿）给lastPose
        matchInfo_.result_ = CMatchInfo::MATCH_FAIL;
        matchInfo_.pst_ = pstInit;
        lastPose_ = pstInit;

        matchInfo_.matchNum_ = 0;
        matchInfo_.matchRatio_ = 0;
        estimatePose = PostureToAffine(lastPose_);

        clusterSourceScan_->InvTransform(lastPose_); // 转换至世界坐标系（根据当前定位的位姿）
        scanMap_.push_back(*clusterSourceScan_);

        return false;
    }

#endif

    delete allObj;

}
bool CTemplateMethod::ReSetMethod()
{

    return true;
}
//
//   取得匹配数据。
//
CMatchInfo *CTemplateMethod::GetMatchInfo()
{
    return &matchInfo_;
}

//
//   对定位质量进行评估。
//
bool CTemplateMethod::EvaluateQuality(float &score)
{
    // 暂未实现，待将来完成
    // 距离判断，角度偏移判断，跳变不能太大
    // 匹配点数判断，与上一帧成功匹配对的比例是否相等

    return true;
}

//
//   从二进制文件装入模板地图。
//
bool CTemplateMethod::LoadBinary(FILE *fp, string filename,int floor, bool bChangeFloor)
{
    if (readJffFormat)
        return true;

    if (!map_->LoadBinary(fp))
        return false;

    int dummy = 0;
    fread(&dummy, sizeof(int), 1, fp);

    return true;
}

//
//   将模板地图写入二进制文件。
//
bool CTemplateMethod::SaveBinary(FILE *fp, string filename)
{
    if (!map_->SaveBinary(fp))
        return false;

    int dummy = 0;
    fwrite(&dummy, sizeof(int), 1, fp);

    return true;
}


bool CTemplateMethod::MotionFilter(const CPosture poseNow)
{
    // dist
    float dist_x = poseNow.x - lastPose_.x;
    float dist_y = poseNow.y - lastPose_.y;
    float dist = sqrt(pow(dist_x, 2) + pow(dist_y, 2));
    // angle
    float angle = CAngle::NormAngle2(poseNow.fThita - lastPose_.fThita);
    angle = angle / PI * 180;

   // if (dist < 0.3 && angle < 30)
    //    return true;

    if (dist < 0.15 && angle < 10)
        return true;
    return false;

}



