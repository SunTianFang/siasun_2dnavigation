#pragma once

#include "MagicSingleton.h"
#include "topcampub.h"
#include <fstream>
#include <iostream>
#include "Geometry.h"
#include "json/json.h"

#include "SensorFamily.h"





enum
{
    RECORDIMG_DISABLE,
    RECORDIMG_INIT_FALURE,
    RECORDIMG_LASER_ERROR,
    RECORDIMG_START_SUCCESS,
    RECORDIMG_ISNOT_RECORDING,
    RECORDIMG_OPENFILE_ERROR,
    RECORDIMG_STOP_SUCCESS
};

namespace mapping {


class  CRecTopvisionImage
{
  private:
    CRecTopvisionImage();
    ~CRecTopvisionImage();


    topCamPub tp;
    bool rtn;
    vector<cv::Mat> img_record;
    string imgname;

   // vector<uint64_t> time_record;

    //vector<CPosture>       transform_odom;

    bool m_bCameraInit;
    bool m_bLaserParm;
    bool m_bFirstTime;

    CScannerGroupParam  m_scannerParam;     // 激光参数

    float angle_step;
    float min_angle;
    float max_angle;

    HANDLE     m_hKillThread;
    HANDLE     m_hThreadDead;
    pthread_t  m_pRecordThread;


    std::mutex          m_mtxRecord;

    vector<Json::Value> m_vtLasers;
    vector<Json::Value> m_vtImgs;
    vector<Json::Value> m_vtPoses;
    vector<Json::Value> m_vtOdoms;


    CPosture            m_PreOdomPst;

    CPosture            m_PreRobotPst;


    friend MagicSingleton2<CRecTopvisionImage>;

  public:

    int StartRecord();

    bool RecordImage(CPosture &robotPos);

    int StopRecord();

    void Reset();

    bool RecordLaserAndPos(const sensor::CRawScan& rawScan,CPosture &robotPose);
};
}
using CRecTopvisionImageSingleton = MagicSingleton2<mapping::CRecTopvisionImage>;
