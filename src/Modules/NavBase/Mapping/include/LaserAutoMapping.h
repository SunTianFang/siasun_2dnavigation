//
//   The interface of class "CLaserMapping".
//

#pragma once

#include <stdio.h>
#include <deque>
#include <pthread.h>
#include <atomic>
#include "ZTypes.h"
#include "Geometry.h"
#include "MagicSingleton.h"
#include "SensorFamily.h"
#include "NaviInterface.h"
#include <queue>
#include <mutex>

#include "ndt_pointcloud.h"
#include "Scan.h"
#include "SendMap.h"
#include "navigation.h"
#include "StampedPosture.h"

// dq VISION 20230707
#include "topcampub.h"
#include <fstream>
#include <iostream>



////////////////////////////////////////////////
//   实现自动建图功能
//   采用了cartographer中slam实现自动建图
//   Author: lishen
//   Date:   2022. 5.
///////////////////////////////////////////////


#define  AUTO_MAPPING_MIN_CTRL_CYCLE 50     // 10ms
#define  AUTO_MAPPING_MAX_CTRL_CYCLE 100    // 50ms

namespace mapping {


//  by  lishen Step数据信息

#ifdef ONE_LASER


class CStepData{
public:
    int id;                             //节点ID
    CPosture m_pstOdom;
    CPosture m_pstRobot;                // 机器人的绝对姿态(实际计算使用)
    CScan    m_scan;                    // 扫描点云

    CStampedPosture m_pstMoveEst;       // 机器人的估测姿态变化量
    CStampedPosture m_pst;              // 由数据集记录的机器人的绝对姿态(仅供参考)
    CPosture        m_vel;              // 速度向量，借用CPosture结构来表示
    // CVectScan m_scanLocal;           // 相对于扫描姿态的局部点云
    // CVectScan m_scanGlobal;          // 全局激光扫描数据


    CStepData()
    {

    }

    CStepData(const CStepData& other)
    {
        this->id            = other.id;
        this->m_pstOdom     = other.m_pstOdom;
        this->m_pstRobot    = other.m_pstRobot;
        this->m_scan        = other.m_scan;
        this->m_pstMoveEst  = other.m_pstMoveEst;
        this->m_pst         = other.m_pst;
        this->m_vel         = other.m_vel;
    }

    CStepData& operator = (const CStepData& obj)
    {
        this->id            = obj.id;
        this->m_pstOdom     = obj.m_pstOdom;
        this->m_pstRobot    = obj.m_pstRobot;
        this->m_scan        = obj.m_scan;
        this->m_pstMoveEst  = obj.m_pstMoveEst;
        this->m_pst         = obj.m_pst;
        this->m_vel         = obj.m_vel;
        return *this;
    }

    CStepData& operator = (CStepData& obj)
    {
        this->id            = obj.id;
        this->m_pstOdom     = obj.m_pstOdom;
        this->m_pstRobot    = obj.m_pstRobot;
        this->m_scan        = obj.m_scan;
        this->m_pstMoveEst  = obj.m_pstMoveEst;
        this->m_pst         = obj.m_pst;
        this->m_vel         = obj.m_vel;
        return *this;
    }
};

#else

class CStepData{
public:
    int id;                             //节点ID
    CPosture m_pstOdom;
    CPosture m_pstRobot;                // 机器人的绝对姿态(实际计算使用)
    //CScan    m_scan;                    // 扫描点云
    vector<CScan>  m_scans;                    // 扫描点云

    CStampedPosture m_pstMoveEst;       // 机器人的估测姿态变化量
    CStampedPosture m_pst;              // 由数据集记录的机器人的绝对姿态(仅供参考)
    CPosture        m_vel;              // 速度向量，借用CPosture结构来表示
    // CVectScan m_scanLocal;           // 相对于扫描姿态的局部点云
    // CVectScan m_scanGlobal;          // 全局激光扫描数据


    CStepData()
    {

    }

    CStepData(const CStepData& other)
    {
        this->id            = other.id;
        this->m_pstOdom     = other.m_pstOdom;
        this->m_pstRobot    = other.m_pstRobot;
        this->m_scans        = other.m_scans;
        this->m_pstMoveEst  = other.m_pstMoveEst;
        this->m_pst         = other.m_pst;
        this->m_vel         = other.m_vel;
    }

    CStepData& operator = (const CStepData& obj)
    {
        this->id            = obj.id;
        this->m_pstOdom     = obj.m_pstOdom;
        this->m_pstRobot    = obj.m_pstRobot;
        this->m_scans        = obj.m_scans;
        this->m_pstMoveEst  = obj.m_pstMoveEst;
        this->m_pst         = obj.m_pst;
        this->m_vel         = obj.m_vel;
        return *this;
    }

    CStepData& operator = (CStepData& obj)
    {
        this->id            = obj.id;
        this->m_pstOdom     = obj.m_pstOdom;
        this->m_pstRobot    = obj.m_pstRobot;
        this->m_scans        = obj.m_scans;
        this->m_pstMoveEst  = obj.m_pstMoveEst;
        this->m_pst         = obj.m_pst;
        this->m_vel         = obj.m_vel;
        return *this;
    }
};

#endif

typedef enum
{
    Mapping_STAND_BY,
    Mapping_MAP_BUILDING,
    Mapping_MAP_OPTING,
    Mapping_MAP_WAITING,
    Mapping_MAP_SAVING,
    Mapping_MAP_CANCELING,
    Mapping_MAP_WAITING_EXPAND,
    Mapping_MAP_READPB_FAILED


}MappingStatus;

typedef enum
{
    Mode_BuildMap,
    Mode_ExpandMap,
    Mode_UpdateMap,

}MappingMode;

class CLaserAutoMapping
{

private:

    bool                m_bInit;            //是否初始化
    bool                m_bStarted;         // 开始建图标志位
    bool                m_bStopping;        // 结束建图标志位
    bool                m_bMapping;
    bool                m_bSaving;

    unsigned int        m_nStartTime;
    bool                m_bFirstTime;
    std::atomic_uchar   m_aWorkMode;
    atomic_ullong       m_CurTimeStamp;
    atomic_ullong       m_PreTimeStamp;
    atomic_bool         m_aFileSaving;
    CPosture            m_CurOdomPst;
    CPosture            m_PreOdomPst;

    // dq VISION 20230707
    //CPosture            m_PreOdomPst1;
    vector<CPosture>       transform_odom;
    topCamPub tp;
    bool rtn;
    vector<cv::Mat> img_record;
    string imgname;
    vector<uint64_t> time_record;


    std::mutex          build_mtx;
    CScannerGroupParam  m_scannerParam;     // 激光参数
    CScannerGroupParam  m_scannerParamExpandDx;


    Pose                m_expandInitPos;

    std::deque<sensor::CRawScan>    m_dequeRawScan;     //原始数据帧队列
    map<int,sensor::CRawScan>       m_rawScans;         //节点数据帧
    map<int,CStepData>              m_stepDatas;        //节点数据信息

    Pose                            m_RecordPose;
    bool                            m_SetPose;
    bool                            m_bFrozenNode;

    bool                            m_bCameraFlag;


public:

  MappingStatus       m_status;
   bool                m_bPadAnswer;
    CCartoSlam                      *m_pCartoSlam;
    bool                m_bFirstScan;       // 第一帧创建地图标志位
    MappingMode         m_mode;
    bool                m_bWaitingExpand;

    HANDLE     m_hKillThread;       // Handle of "Kill thread" event
    HANDLE     m_hThreadDead;       // Handle of "Thread dead" event
    HANDLE     m_hExpandMap;

    pthread_t  m_pMappingThread;
    pthread_t  m_pCsmOptThread;

    atomic_ullong m_mappingCurTime;
    atomic_ullong m_mappingPreTime;
    atomic_ullong m_mappingEndTime;

    int submap_version_record = 0;
    int submap_index_record =0;
    common::Time stamp_record = 0;
    std::deque<float> submap_pose ={0};
    std::deque<float> submap_pose_record;


    Eigen::Affine3d ToEigen(const transform::Rigid3d& rigid3) {
      return Eigen::Translation3d(rigid3.translation()) * rigid3.rotation();
    }
private:
    // 构造函数
    CLaserAutoMapping();

    // 析构函数
    ~CLaserAutoMapping();

    friend MagicSingleton2<CLaserAutoMapping>;

private:


    //根据数据集得到创建地图需要左上角和右下角坐标
    bool GetMapRange(CPosture &ptLeftBottom, CPosture &ptRightTop);

    bool Stop();

    bool UpdateScan(sensor::CRawScan& rawScan);

    bool AddScan(sensor::CRawScan& rawScan);

    bool WriteLaserParm(const char* filename);

    bool WriteScanData(const char* filename);

    bool TransformRawCloudToScan(const CPosture &pstRobot, const CLaserScannerParam &Param,
                                                   const sensor::CRawScan& rawScan, CScan &scan);
    bool TransformRawCloudToLaserMsg(const sensor::CRawScan& rawScan, laserscan_msg &lscan);

    bool TransformRawCloudLaserScan(const sensor::CRawScan& rawScan, sensor_msgs::LaserScan &lscan);


    bool TransformRawCloudToScan(const CPosture &pstRobot,  const CScannerGroupParam &Params,
                                  const sensor::CRawScan& rawScan, vector<CScan> &scans);


   // bool ReadLaserParm(FILE* pFile);

public:

    // slam reset 回调函数
    void SlamMappingReset(void);

    //创建地图线程函数
    void MappingProc();

    //开始创建
    bool StartMapping();

    //停止创建
    bool StopMapping();

    //保存地图
    bool SaveMap();

    //保存dx 数据
    bool SaveDxFile(const char* filename);

    //保存pb 数据
    bool SavePbFile(const char* filename);

    bool LoadPbFile(char* filename);


    void SetMappingMode(MappingMode uMode);

    //
    bool RotateMap(double angle);

    //初始化
    bool Initialize();

    //创建地图过程回调函数
    static void CartoSlamCallback(slam_result *slam, std::vector<opt_result> *opt);

    //carto创建地图结束回调函数
    static void BuildMapOverCallback();

    //处理优化后的位姿
    void HandleOptPose(slam_result *slam, std::vector<opt_result> *opt);

    //取消保存地图
    bool CancelSaveMap();

    void SetInitPos(Pose &pos);

    void SetExpandFrozen(bool frozen);

    void ExpandMap();

    void SetPadAnswer(){m_bPadAnswer = true;}


    inline int SetMappingStatus(MappingStatus emStatus) { m_status = emStatus; }
    inline int GetMappingStatus()
    {
        return m_status;
    }
    inline int GetMappingMode()
    {
        return m_mode;
    }

    void SendSubMaps(int num, int8_t *pSubmapIds);




};

} // namespace mapping

using LaserAutoMappingSingleton = MagicSingleton2<mapping::CLaserAutoMapping>;
