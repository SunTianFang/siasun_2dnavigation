//
//   The interface of class "CBaseOdometry".
//

#pragma once

#include <stdio.h>
#include <deque>
#include <mutex>
#include <atomic>
#include "Geometry.h"
#include "MagicSingleton.h"
#include "RawScan.h"
#include "TimeStamp.h"

namespace mapping {

class StVelocity
{
public:
    short Vx;
    short Vy;
    short Vtheta;
    short SteerAngle;
    short HeadingAngle;
    int GyroAngle;
    unsigned long long lGyroTime;
    unsigned long long lRawTime;
    unsigned char coordBase;
    unsigned long long lRecvTime;

public:
    void Initialize()
    {
        Vx = 0;
        Vy = 0;
        Vtheta = 0;
        SteerAngle = 0;
        HeadingAngle = 0;
        GyroAngle = 0;
        lGyroTime = 0;
        lRawTime = 0;
        coordBase = 0;
        lRecvTime = 0;
    }

    StVelocity()
    {
        Initialize();
    }

    ~StVelocity()
    {
    }

    StVelocity(const StVelocity& other)
    {
        Initialize();
        this->Vx = other.Vx;
        this->Vy = other.Vy;
        this->Vtheta = other.Vtheta;
        this->SteerAngle = other.SteerAngle;
        this->HeadingAngle = other.HeadingAngle;
        this->GyroAngle = other.GyroAngle;
        this->lGyroTime = other.lGyroTime;
        this->lRawTime = other.lRawTime;
        this->coordBase = other.coordBase;
        this->lRecvTime = other.lRecvTime;
    }

    StVelocity& operator = (const StVelocity& Obj)
    {
        Initialize();
        this->Vx = Obj.Vx;
        this->Vy = Obj.Vy;
        this->Vtheta = Obj.Vtheta;
        this->SteerAngle = Obj.SteerAngle;
        this->HeadingAngle = Obj.HeadingAngle;
        this->GyroAngle = Obj.GyroAngle;
        this->lGyroTime = Obj.lGyroTime;
        this->lRawTime = Obj.lRawTime;
        this->coordBase = Obj.coordBase;
        this->lRecvTime = Obj.lRecvTime;
        return *this;
    }

    CVelocity ToGeometryVel() const
    {
        return CVelocity(Vx/1000.0, Vy/1000.0, Vtheta/1000.0);
    }
};

class COdometryFrame
{
public:
    unsigned long long time_stamp;
    CPosture pose;

public:
    void Initialize()
    {
        time_stamp = 0;
        pose.SetPnt(0.0, 0.0);
        pose.SetAngle(0.0);
    }

    COdometryFrame()
    {
        Initialize();
    }

    ~COdometryFrame()
    {
    }

    COdometryFrame(const COdometryFrame& other)
    {
        this->time_stamp = other.time_stamp;
        this->pose = other.pose;
    }

    COdometryFrame& operator = (const COdometryFrame& Obj)
    {
        this->time_stamp = Obj.time_stamp;
        this->pose = Obj.pose;
        return *this;
    }
};

class CBaseOdometry
{
private:
    std::deque<StVelocity*> m_Velocity;
    std::deque<COdometryFrame> m_OdomFrames;
    std::mutex vel_mtx;
    std::mutex odom_data_mtx;
    std::mutex odom_frame_mtx;
    unsigned int m_nVelCount;
    unsigned int m_nFrameCount;

    CPosture m_EstiOdomPst;
    CPosture m_CurOdomPst;
    CPosture m_LastOdomPst;
    CPosture m_LocalOdomTrans;
    CVelocity m_CurVel;
    double m_TotalVel;
    std::atomic<long> m_AccumuOdom;
    atomic_ullong m_CurTimeStamp;
    atomic_ullong m_PreTimeStamp;
    bool m_bFirstTime;

public:
    bool m_bGyroEnable;     // 是否启用陀螺仪
    bool m_bGyroRemote;     // 陀螺仪数据是远端还是本地, true:remote;false:local.

private:
    CBaseOdometry();
    ~CBaseOdometry();

    bool LinearInterpo(unsigned long long lCurTime, unsigned long long lFrontTime,
                       unsigned long long lBackTime, double& ratioFront, double& ratioBack);

    bool LookUpInterVelocity(unsigned long long lCurTime, StVelocity& frontVel, StVelocity& backVel);

    bool LookUpInterVelocitySync(unsigned long long lCurTime, long long lSyncOffset, StVelocity& frontVel, StVelocity& backVel);

    bool LookUpInterGyroAngleSync(unsigned long long lCurTime, long long lSyncOffset, StVelocity& frontAngle, StVelocity& backAngle);

    friend MagicSingleton<CBaseOdometry>;

private:
    StVelocity* CloneVelocity(short Vx, short Vy, short Vtheta, short SteerAngle, short HeadingAngle,
                              int GyroAngle, unsigned long long lGyroTime, unsigned long long lRawTime,
                              unsigned char coordBase, unsigned long long lRecvTime);

    bool GetEstimateOdomStep(unsigned long long lStartTime, unsigned long long lEndTime,
                              const CPosture& curOdom, CPosture& odomStep);

    bool AddOdomFrame(const COdometryFrame& frame);

    bool LookUpInterOdomFrame(unsigned long long lCurTime, COdometryFrame& frontFrame, COdometryFrame& backFrame);

    bool GetRemoteGyroAngle(unsigned long long cur_time, CFloatDataInfo& sync_data);

    bool SolveOdomAngleStep(unsigned long long start_time, unsigned long long end_time, CVelocity& velo, float& angle_step);

public:
    bool Initialize();

    void Clear();

    bool UpdateVelocity(short Vx, short Vy, short Vtheta, short SteerAngle, short HeadingAngle,
                        int GyroAngle, unsigned long long lGyroTime, unsigned long long lRawTime,
                        unsigned char coordBase, unsigned long long lRecvTime);

    bool UpdateEstimateOdom();

    bool GetLocalOdomTrans(unsigned long long lStartTime, unsigned long long lEndTime, CPosture& odomTrans);

    bool GetOdomData(sensor::COdometryData& data);

    unsigned int GetOdomFlag();

    bool GetOdomPosture(unsigned long long odom_time, CPosture& odom_pst);

    void SetAccumuOdom(long accuOdom);
    long GetAccumuOdom();

    StVelocity GetBaseVelocity();

};

} // namespace mapping

using BaseOdomSingleton = MagicSingleton<mapping::CBaseOdometry>;

//auto pA = Singleton<A>::GetInstance();
//auto pA = LaserOdomSingleton::GetInstance();

