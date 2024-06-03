//
//   The interface of class "CBaseOdometry".
//

#include "BaseOdometry.h"
#include "Tools.h"
#include "CanMan.h"
#include "blackboxhelper.hpp"
#include "ParameterObject.h"
#include "RoboLocClnt.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define MAX_VELOCITY_MEM_COUNT      200
#define MAX_ODOM_FRAME_COUNT        200
#define CTRL_CYCLE                  50
#define USE_LINEAR_INTERPO

//FIXME:机器人通讯超时为600毫秒,通信超时后开始按照减速加速度停车,后续应该在通讯中断后按照理想速度规划更新里程计位姿.
#define VELO_DATA_VALID_TIME        3000
#define GYRO_DATA_VALID_TIME        600
#define ODOM_DATA_VALID_TIME        1000
#define ODOMETRY_FLAG               6 //4
#define GYRO_DISABLE_TIME           5000 //5000ms
#define GYRO_OFFSET_TIME            60 //60ms
#define GYRO_TOLERANCE_RATIO        10
#define ROBOT_MAX_LINEAR_VELO       5.0 //5.0m/s
#define ROBOT_MAX_ANGULAR_VELO      5.0 //5.0rad/s

#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CLaserOdometry".
namespace mapping {

CBaseOdometry::CBaseOdometry()
{
    m_nVelCount = 0;
    m_nFrameCount = 0;
    m_Velocity.clear();
    m_OdomFrames.clear();
    m_CurVel.Initialize();
    m_EstiOdomPst.SetPnt(0.0, 0.0);
    m_EstiOdomPst.SetAngle(0.0);
    m_TotalVel = 0.0;
    m_AccumuOdom = 0;
    m_CurTimeStamp = 0;
    m_PreTimeStamp = 0;
    m_bFirstTime = true;

    m_bGyroEnable = false;
    m_bGyroRemote = false;
}

CBaseOdometry::~CBaseOdometry()
{
    Clear();
}

bool CBaseOdometry::Initialize()
{
    m_bGyroEnable = false;
    m_bGyroRemote = false;

    auto pParameterObject = ParameterObjectSingleton::GetInstance();
    pParameterObject->GetParameterValue("Gyro_Enable", m_bGyroEnable);
    pParameterObject->GetParameterValue("Gyro_Remote", m_bGyroRemote);

    return true;
}

void CBaseOdometry::Clear()
{
    for (unsigned int i = 0; i < m_Velocity.size(); i++)
    {
        if (m_Velocity[i] != NULL) {
            delete m_Velocity[i];
            m_Velocity[i] = NULL;
        }
    }
    m_Velocity.clear();
    m_OdomFrames.clear();
}

bool CBaseOdometry::UpdateVelocity(short Vx, short Vy, short Vtheta, short SteerAngle, short HeadingAngle,
                                   int GyroAngle, unsigned long long lGyroTime, unsigned long long lRawTime,
                                   unsigned char coordBase, unsigned long long lRecvTime)
{
    {
        std::lock_guard<std::mutex> lock(vel_mtx);

        // 接收到的速度值或者陀螺仪值更新，就加入到缓存
        if(!m_Velocity.empty()) {
            if(m_Velocity.back()->lRawTime >= lRawTime && m_Velocity.back()->lGyroTime >= lGyroTime) {
                return false;
            }
        }

        StVelocity* stVel = CloneVelocity(Vx, Vy, Vtheta, SteerAngle, HeadingAngle, GyroAngle, lGyroTime,
                                          lRawTime, coordBase, lRecvTime);
        if (stVel == NULL)
            return false;

        if(m_nVelCount >= MAX_VELOCITY_MEM_COUNT)
        {
            if(m_Velocity.front() != NULL){
                delete m_Velocity.front();
            }
            m_Velocity.pop_front();

            m_Velocity.push_back(stVel);
            m_nVelCount = MAX_VELOCITY_MEM_COUNT;
        }
        else
        {
            m_Velocity.push_back(stVel);
            m_nVelCount += 1;
        }
    }

    return true;
}

StVelocity* CBaseOdometry::CloneVelocity(short Vx, short Vy, short Vtheta, short SteerAngle, short HeadingAngle,
                                         int GyroAngle, unsigned long long lGyroTime, unsigned long long lRawTime,
                                         unsigned char coordBase, unsigned long long lRecvTime)
{
    StVelocity *ret = new StVelocity();
    if(ret == NULL) {
        return NULL;
    }

    ret->Vx = Vx;
    ret->Vy = Vy;
    ret->Vtheta = Vtheta;
    ret->SteerAngle = SteerAngle;
    ret->HeadingAngle = HeadingAngle;
    ret->GyroAngle = GyroAngle;
    ret->lGyroTime = lGyroTime;
    ret->lRawTime = lRawTime;
    ret->coordBase = coordBase;
    ret->lRecvTime = lRecvTime;
    return ret;
}

bool CBaseOdometry::GetEstimateOdomStep(unsigned long long lStartTime, unsigned long long lEndTime,
                                        const CPosture& curOdom, CPosture& odomStep)
{
    bool bRet = false;
    long long lTimeSpan = static_cast<long long>(lEndTime - lStartTime);
    odomStep.SetPosture(0.0, 0.0, 0.0);
    if(lTimeSpan <= 0) {
        return false;
    }

    //　同步时间机制
    auto pRoboClnt = RoboClntSingleton::GetInstance();
    STSyncTime sync_time_data;
    bool sync_ok = false;
    if(pRoboClnt){
        sync_ok = pRoboClnt->GetSyncTimeData(sync_time_data);
    }

    StVelocity frontVel;
    StVelocity backVel;
    if(sync_ok){
        bRet = LookUpInterVelocitySync(lStartTime, sync_time_data.lTimeOffset, frontVel, backVel);
    }
    else {
        bRet = LookUpInterVelocity(lStartTime, frontVel, backVel);
    }
    if(!bRet) {
        return false;
    }

    double fVx = 0.0;
    double fVy = 0.0;
    double fVthita = 0.0;
    double steer_angle = 0.0;
    double heading_angle = 0.0;
    double speedAngle = 0;
#ifdef USE_LINEAR_INTERPO
    double ratioFront = 0.0;
    double ratioBack = 0.0;
    if(sync_ok){
        unsigned long long sync_start_time = static_cast<unsigned long long>(static_cast<long long>(lStartTime) - sync_time_data.lTimeOffset);
        bRet = LinearInterpo(sync_start_time, frontVel.lRawTime, backVel.lRawTime, ratioFront, ratioBack);
    }
    else {
        bRet = LinearInterpo(lStartTime, frontVel.lRecvTime, backVel.lRecvTime, ratioFront, ratioBack);
    }
    if(!bRet) {
        ratioFront = 1.0;
        ratioBack = 0.0;
    }

    // 速度线性插值计算
    fVx = static_cast<double>(frontVel.Vx) / 1000.0 * ratioFront + static_cast<double>(backVel.Vx) / 1000.0 * ratioBack;
    fVy = static_cast<double>(frontVel.Vy) / 1000.0 * ratioFront + static_cast<double>(backVel.Vy) / 1000.0 * ratioBack;
    fVthita = static_cast<double>(frontVel.Vtheta) / 1000.0 * ratioFront + static_cast<double>(backVel.Vtheta) / 1000.0 * ratioBack;

    // 舵角和方向角不进行线性插值计算,侧移发送实际舵角和车头方向角，其他轨迹发送0.
    if(ratioFront > ratioBack){
        steer_angle = static_cast<double>(frontVel.SteerAngle) / 1000.0;
        heading_angle = static_cast<double>(frontVel.HeadingAngle) / 1000.0;
    }
    else {
        steer_angle = static_cast<double>(backVel.SteerAngle) / 1000.0;
        heading_angle = static_cast<double>(backVel.HeadingAngle) / 1000.0;
    }
    // 舵角范围:[-PI, PI].
    steer_angle = CAngle::NormAngle2(steer_angle);
    // 车头方向角范围:[0, 2*PI]
    heading_angle = CAngle::NormAngle(heading_angle);

#else
    fVx = static_cast<double>(frontVel.Vx) / 1000.0;
    fVy = static_cast<double>(frontVel.Vy) / 1000.0;
    fVthita = static_cast<double>(frontVel.Vtheta) / 1000.0;
    steer_angle = static_cast<double>(frontVel.SteerAngle) / 1000.0;
    heading_angle = static_cast<double>(frontVel.HeadingAngle) / 1000.0;
#endif

    // 当有线速度时,计算距离;当只有角速度时,计算弧度;
    m_TotalVel = sqrt(fVy * fVy + fVx * fVx);
    if(fabs(m_TotalVel) > 0.0001){
        m_AccumuOdom += static_cast<long>(fabs(m_TotalVel) * lTimeSpan * 100.0);
    }
    else if (fabs(fVthita) > 0.0001) {
        m_AccumuOdom += static_cast<long>(fabs(fVthita) * lTimeSpan * 100.0);
    }
    else {
    }

#if 1
    // 支持全方位车型里程计估计,如直线、圆弧、自旋、侧移轨迹.
    speedAngle = CAngle::NormAngle(atan2(fVy, fVx) + steer_angle + curOdom.fThita); /*+ PI / 2.0*/ // 车体中心坐标系和激光传感器坐标系相差90度.
#else
    speedAngle = CAngle::NormAngle(atan2(fVy, fVx) + pstScanner.fThita /*+ PI / 2*/);
#endif

    // 保存速度向量
    m_CurVel.fXLinear = m_TotalVel * cos(speedAngle);
    m_CurVel.fYLinear = m_TotalVel * sin(speedAngle);
    m_CurVel.fAngular = fVthita;

    double x_step = 0.0;
    double y_step = 0.0;
    float angle_step = 0;
    // 计算位移变化量
    x_step = m_CurVel.fXLinear * lTimeSpan / 1000.0;
    y_step = m_CurVel.fYLinear * lTimeSpan / 1000.0;
    // 计算角度变化量
    if(m_bGyroEnable){
        SolveOdomAngleStep(lStartTime, lEndTime, m_CurVel, angle_step);
    }
    else {
        angle_step = static_cast<float>(m_CurVel.fAngular * lTimeSpan / 1000.0);
    }

    double x_step_limit = 0.0;
    double y_step_limit = 0.0;
    float angle_step_limit = 0;
    // 对计算的位姿变化量进行一致性检测，需要满足最大线速度和角速度
    angle_step = CAngle::NormAngle2(angle_step);
    angle_step_limit = static_cast<float>(ROBOT_MAX_ANGULAR_VELO * lTimeSpan / 1000.0);
    if(fabs(angle_step) > fabs(angle_step_limit)){
        angle_step = angle_step / fabs(angle_step) * fabs(angle_step_limit);
    }

    x_step_limit = ROBOT_MAX_LINEAR_VELO * lTimeSpan / 1000.0;
    if(fabs(x_step) > fabs(x_step_limit)){
        x_step = x_step / fabs(x_step) * fabs(x_step_limit);
    }

    y_step_limit = ROBOT_MAX_LINEAR_VELO * lTimeSpan / 1000.0;
    if(fabs(y_step) > fabs(y_step_limit)){
        y_step = y_step / fabs(y_step) * fabs(y_step_limit);
    }

    odomStep.x = x_step;
    odomStep.y = y_step;
    odomStep.fThita = static_cast<double>(angle_step);
    return true;
}

bool CBaseOdometry::AddOdomFrame(const COdometryFrame& frame)
{
    {
        std::lock_guard<std::mutex> lock(odom_frame_mtx);
        if(!m_OdomFrames.empty()) {
            if(m_OdomFrames.back().time_stamp >= frame.time_stamp) {
                return false;
            }
        }

        if(m_nFrameCount >= MAX_ODOM_FRAME_COUNT){
            m_OdomFrames.pop_front();
            m_OdomFrames.push_back(frame);
            m_nFrameCount = MAX_ODOM_FRAME_COUNT;
        }
        else{
            m_OdomFrames.push_back(frame);
            m_nFrameCount += 1;
        }
    }

    return true;
}

bool CBaseOdometry::LookUpInterOdomFrame(unsigned long long lCurTime, COdometryFrame& frontFrame, COdometryFrame& backFrame)
{
    bool bRet = false;
    if(m_OdomFrames.empty()) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(odom_frame_mtx);

        if(lCurTime >= m_OdomFrames.back().time_stamp) {
            CPosture odom_step(0.0, 0.0, 0.0);
            COdometryFrame& back_frame = m_OdomFrames.back();
            GetEstimateOdomStep(back_frame.time_stamp, lCurTime, back_frame.pose, odom_step);
            frontFrame.pose.x = back_frame.pose.x + odom_step.x;
            frontFrame.pose.y = back_frame.pose.y + odom_step.y;
            frontFrame.pose.fThita = back_frame.pose.fThita + odom_step.fThita;
            frontFrame.pose.fThita = CAngle::NormAngle(frontFrame.pose.fThita);
            frontFrame.time_stamp = lCurTime;
            backFrame = frontFrame;
            if((lCurTime - m_OdomFrames.back().time_stamp) < ODOM_DATA_VALID_TIME) {
                return true;
            }
            else {
                return false;
            }
        }

        if(lCurTime <= m_OdomFrames.front().time_stamp) {
            frontFrame = m_OdomFrames.front();
            backFrame = m_OdomFrames.front();
            if((m_OdomFrames.front().time_stamp - lCurTime) < ODOM_DATA_VALID_TIME) {
                return true;
            }
            else {
                return false;
            }
        }

        // 里程计数据按时间序列排列，在里程计数据中为同步的时间点找到合适的时间位置
        // 即找到与同步时间相邻的左右两个数据
        // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做插值
        long lSize = static_cast<long>(m_OdomFrames.size());
        for (long i = lSize - 1; i >= 0; --i){
            if(lCurTime < m_OdomFrames[i].time_stamp){
                continue;
            }
            if(lCurTime == m_OdomFrames[i].time_stamp){
                frontFrame = m_OdomFrames[i];
                backFrame = m_OdomFrames[i];
                bRet = true;
                break;
            }
            if((lCurTime - m_OdomFrames[i].time_stamp) > ODOM_DATA_VALID_TIME){
                bRet = false;
                break;
            }

            if((i + 1) < lSize){
                if(lCurTime > m_OdomFrames[i + 1].time_stamp
                        || (m_OdomFrames[i + 1].time_stamp - lCurTime) > ODOM_DATA_VALID_TIME){
                    bRet = false;
                    break;
                }
                frontFrame = m_OdomFrames[i + 1];
                backFrame = m_OdomFrames[i];
                bRet = true;
                break;
            }
        }

        if(!bRet) {
            frontFrame = m_OdomFrames.back();
            backFrame = m_OdomFrames.back();
        }
    }

    return bRet;
}

bool CBaseOdometry::GetRemoteGyroAngle(unsigned long long cur_time, CFloatDataInfo& sync_data)
{
    bool bRet = false;
    StVelocity front_velo;
    StVelocity back_velo;
    sync_data.clear();

    //　同步时间机制
    auto pRoboClnt = RoboClntSingleton::GetInstance();
    STSyncTime sync_time_data;
    bool sync_ok = false;
    if(pRoboClnt){
        sync_ok = pRoboClnt->GetSyncTimeData(sync_time_data);
    }

    if(sync_ok){
        bRet = LookUpInterGyroAngleSync(cur_time, sync_time_data.lTimeOffset, front_velo, back_velo);
    }
    else {
        bRet = LookUpInterVelocity(cur_time, front_velo, back_velo);
    }
    if(!bRet) {
        return false;
    }

    double gyro_angle = 0.0;
    unsigned long long gyro_angle_time = 0;
#ifdef USE_LINEAR_INTERPO
    double ratioFront = 0.0;
    double ratioBack = 0.0;
    unsigned long long sync_cur_time = static_cast<unsigned long long>(static_cast<long long>(cur_time) - sync_time_data.lTimeOffset);
    if(sync_ok){
        bRet = LinearInterpo(sync_cur_time, front_velo.lGyroTime, back_velo.lGyroTime, ratioFront, ratioBack);
    }
    else {
        bRet = LinearInterpo(cur_time, front_velo.lRecvTime, back_velo.lRecvTime, ratioFront, ratioBack);
    }
    if(!bRet) {
        ratioFront = 1.0;
        ratioBack = 0.0;
        if(sync_ok){
            gyro_angle_time = static_cast<unsigned long long>(static_cast<long long>(front_velo.lGyroTime) + sync_time_data.lTimeOffset);
        }
        else {
            gyro_angle_time = front_velo.lRecvTime;
        }
    }
    else {
        gyro_angle_time = cur_time;
    }

    // 线性插值,角度需要标准化
    double angle_front = static_cast<double>(front_velo.GyroAngle)/1000.0;
    double angle_back = static_cast<double>(back_velo.GyroAngle)/1000.0;
    double norm_angle_front = 0.0;
    double norm_angle_back = 0.0;
    norm_angle_back = CAngle::NormalizeAngleDifference(angle_back, angle_front);
    norm_angle_front = angle_front;
    gyro_angle = norm_angle_front*ratioFront + norm_angle_back*ratioBack;
#else
    gyro_angle_time = front_velo.lRecvTime;
    gyro_angle = static_cast<double>(front_velo.GyroAngle) / 1000.0;
#endif

    sync_data.m_dwTimeStamp = gyro_angle_time;
    sync_data.m_fData = static_cast<float>(gyro_angle);
    return true;
}

bool CBaseOdometry::SolveOdomAngleStep(unsigned long long start_time, unsigned long long end_time,
                                       CVelocity& velo, float& angle_step)
{
    bool start_ret = false;
    bool end_ret = false;
    CFloatDataInfo start_angle;
    CFloatDataInfo end_angle;
    start_angle.clear();
    end_angle.clear();
    angle_step = 0;

    float angle_by_velo = 0;
    float angle_by_gyro = 0;
    long long time_span = static_cast<long long>(end_time - start_time);
    if(time_span <= 0) {
        return false;
    }
    angle_by_velo = static_cast<float>(velo.fAngular * time_span / 1000.0);

    unsigned long long start_time_offset = 0;
    auto pCanManager = CanManagerSingleton::GetInstance();
    // 远端主控制器传递的陀螺仪数据
    if(m_bGyroRemote){
        end_ret = GetRemoteGyroAngle(end_time, end_angle);

        //由于陀螺仪角度滞后，所以使用前一个周期的陀螺仪角度计算角速度
        start_time_offset = end_angle.m_dwTimeStamp - GYRO_OFFSET_TIME;
        if(start_time_offset < start_time){
            start_ret = GetRemoteGyroAngle(start_time_offset, start_angle);
        }
        else{
            start_ret = GetRemoteGyroAngle(start_time, start_angle);
        }
    }
    //　本地CAN口获取的陀螺仪数据
    else {
        if(pCanManager){
            end_ret = pCanManager->GetGyroAngle(end_time, end_angle);

            //由于陀螺仪角度滞后，所以使用前一个周期的陀螺仪角度计算角速度
            start_time_offset = end_angle.m_dwTimeStamp - GYRO_OFFSET_TIME;
            if(start_time_offset < start_time){
                start_ret = pCanManager->GetGyroAngle(start_time_offset, start_angle);
            }
            else{
                start_ret = pCanManager->GetGyroAngle(start_time, start_angle);
            }
        }
    }

    if(start_ret && end_ret){
        static bool gyro_enable = false;
        static bool robot_stop = false;
        static unsigned long long robot_stop_time = GetTickCount();
        float gyro_actual_angle = 0;
        float gyro_actual_angular = 0;

        if(fabs(velo.fXLinear) > 0.0001 || fabs(velo.fYLinear) > 0.0001 || fabs(velo.fAngular) > 0.0001) {
            gyro_enable = true;
            robot_stop = false;
        }
        // 停止超过一定时间后，不使用陀螺仪计算角度，因为漂移
        else {
            if(!robot_stop){
                robot_stop_time = GetTickCount();
                robot_stop = true;
            }
            if(GetTickCount() - robot_stop_time > GYRO_DISABLE_TIME){
                gyro_enable = false;
            }
        }

        if(gyro_enable) {
            if(end_angle.m_dwTimeStamp > start_angle.m_dwTimeStamp) {
                // 角度标准化
                double norm_angle_start = 0.0;
                double norm_angle_end = 0.0;
                norm_angle_start = CAngle::NormalizeAngleDifference(start_angle.m_fData, end_angle.m_fData);
                norm_angle_end = end_angle.m_fData;

                gyro_actual_angle = norm_angle_end - norm_angle_start;
                gyro_actual_angular = gyro_actual_angle / (end_angle.m_dwTimeStamp - start_angle.m_dwTimeStamp);
                angle_by_gyro = gyro_actual_angular * time_span;

                // 当陀螺仪计算的角度变化和角速度计算的角度变化误差超限，直接使用角速度计算角度
                if(fabs(angle_by_gyro) > fabs(GYRO_TOLERANCE_RATIO * angle_by_velo) ||
                        fabs(angle_by_velo) > fabs(GYRO_TOLERANCE_RATIO * angle_by_gyro)){
                    angle_step = angle_by_velo;
                }
                else {
                    angle_step = angle_by_gyro;
                }
            }
            else {
                angle_step = angle_by_velo;
            }
        }
        else {
            angle_step = 0;
        }
    }
    else {
        // 当陀螺仪数据无效时，使用角速度计算角度
        angle_step = angle_by_velo;
    }

    // 角度需要标准化
    angle_step = CAngle::NormAngle2(angle_step);
    // 计算角度变化量应该小于最大角速度*时间
    float angle_step_limit = 0;
    angle_step_limit = static_cast<float>(ROBOT_MAX_ANGULAR_VELO * time_span / 1000.0);
    if(fabs(angle_step) > fabs(angle_step_limit)){
        angle_step = angle_step / fabs(angle_step) * fabs(angle_step_limit);
    }

    return true;
}

bool CBaseOdometry::UpdateEstimateOdom()
{
    bool bRet = false;
    unsigned long long timeNow = GetTickCount();
    long long diffTime = 0.0;
    if(m_bFirstTime){
        m_PreTimeStamp = timeNow;
        m_bFirstTime = false;
    }

    m_CurTimeStamp = timeNow;
    diffTime = static_cast<long long>(m_CurTimeStamp.load() - m_PreTimeStamp.load());
    if(diffTime <= 0) {
        return false;
    }

    long long div = diffTime / CTRL_CYCLE;
    long long mod = diffTime % CTRL_CYCLE;
    unsigned long long startTime = 0;
    unsigned long long endTime = 0;
    CPosture odom_step(0.0, 0.0, 0.0);
    COdometryFrame odom_frame;
    {
        std::lock_guard<std::mutex> lock(odom_data_mtx);

        m_CurOdomPst.x = m_LastOdomPst.x;
        m_CurOdomPst.y = m_LastOdomPst.y;
        m_CurOdomPst.fThita = m_LastOdomPst.fThita;

        for (long long i = 0; i < div; i++){
            startTime =  m_PreTimeStamp.load() + i * CTRL_CYCLE;
            endTime = startTime + CTRL_CYCLE;
            bRet = GetEstimateOdomStep(startTime, endTime, m_EstiOdomPst, odom_step);
            if(bRet) {
                m_EstiOdomPst.x += odom_step.x;
                m_EstiOdomPst.y += odom_step.y;
                m_EstiOdomPst.fThita += odom_step.fThita;

                m_CurOdomPst.x += odom_step.x;
                m_CurOdomPst.y += odom_step.y;
                m_CurOdomPst.fThita += odom_step.fThita;
            }
        }

        if(mod > 0) {
            startTime =  m_PreTimeStamp.load() + div * CTRL_CYCLE;
            endTime = startTime + mod;
            bRet = GetEstimateOdomStep(startTime, endTime, m_EstiOdomPst, odom_step);
            if(bRet) {
                m_EstiOdomPst.x += odom_step.x;
                m_EstiOdomPst.y += odom_step.y;
                m_EstiOdomPst.fThita += odom_step.fThita;

                m_CurOdomPst.x += odom_step.x;
                m_CurOdomPst.y += odom_step.y;
                m_CurOdomPst.fThita += odom_step.fThita;
            }
        }

        // 姿态角范围:[0, 2*PI].
        m_EstiOdomPst.fThita = CAngle::NormAngle(m_EstiOdomPst.fThita);

        odom_frame.time_stamp = m_CurTimeStamp.load();
        odom_frame.pose = m_EstiOdomPst;
    }
    m_PreTimeStamp = m_CurTimeStamp.load();

    //存储里程计数据帧
    AddOdomFrame(odom_frame);

    return true;
}

bool CBaseOdometry::GetLocalOdomTrans(unsigned long long lStartTime, unsigned long long lEndTime, CPosture& odomTrans)
{
    bool bRet = false;
    CPosture start_pst;
    CPosture end_pst;
    odomTrans.SetPosture(0.0, 0.0, 0.0);
    long long time_elapse = static_cast<long long>(lEndTime - lStartTime);
    if(time_elapse <= 0) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(odom_data_mtx);

        COdometryFrame frontFrame1;
        COdometryFrame backFrame1;
        bRet = LookUpInterOdomFrame(lStartTime, frontFrame1, backFrame1);
        if(!bRet) {
            return false;
        }

        COdometryFrame frontFrame2;
        COdometryFrame backFrame2;
        bRet = LookUpInterOdomFrame(lEndTime, frontFrame2, backFrame2);
        if(!bRet) {
            return false;
        }

#ifdef USE_LINEAR_INTERPO
        double ratioFront1 = 0.0;
        double ratioBack1 = 0.0;
        bRet = LinearInterpo(lStartTime, frontFrame1.time_stamp, backFrame1.time_stamp, ratioFront1, ratioBack1);
        if(!bRet) {
            ratioFront1 = 1.0;
            ratioBack1 = 0.0;
        }
        start_pst.x = frontFrame1.pose.x * ratioFront1 + backFrame1.pose.x * ratioBack1;
        start_pst.y = frontFrame1.pose.y * ratioFront1 + backFrame1.pose.y * ratioBack1;

        // 线性插值,角度需要标准化
        double norm_ori_front1 = 0.0;
        double norm_ori_back1 = 0.0;
        norm_ori_back1 = CAngle::NormalizeAngleDifference(backFrame1.pose.fThita, frontFrame1.pose.fThita);
        norm_ori_front1 = frontFrame1.pose.fThita;

        start_pst.fThita = norm_ori_front1 * ratioFront1 + norm_ori_back1 * ratioBack1;
        start_pst.fThita = CAngle::NormAngle(start_pst.fThita);

        double ratioFront2 = 0.0;
        double ratioBack2 = 0.0;
        bRet = LinearInterpo(lEndTime, frontFrame2.time_stamp, backFrame2.time_stamp, ratioFront2, ratioBack2);
        if(!bRet) {
            ratioFront2 = 1.0;
            ratioBack2 = 0.0;
        }
        end_pst.x = frontFrame2.pose.x * ratioFront2 + backFrame2.pose.x * ratioBack2;
        end_pst.y = frontFrame2.pose.y * ratioFront2 + backFrame2.pose.y * ratioBack2;

        // 线性插值,角度需要标准化
        double norm_ori_front2 = 0.0;
        double norm_ori_back2 = 0.0;
        norm_ori_back2 = CAngle::NormalizeAngleDifference(backFrame2.pose.fThita, frontFrame2.pose.fThita);
        norm_ori_front2 = frontFrame2.pose.fThita;

        end_pst.fThita = norm_ori_front2 * ratioFront2 + norm_ori_back2 * ratioBack2;
        end_pst.fThita = CAngle::NormAngle(end_pst.fThita);
#else
        start_pst = frontFrame1.pose;
        end_pst = frontFrame2.pose;
#endif

        CTransform transOdom;
        CPosture odom_trans_;
        odom_trans_.SetPosture(0.0, 0.0, 0.0);
        transOdom.Init(start_pst);
        odom_trans_ = transOdom.GetLocalPosture(end_pst);
        // [-PI, PI]
        odom_trans_.fThita = CAngle::NormAngle2(odom_trans_.fThita);
        odomTrans = odom_trans_;
        m_LocalOdomTrans = odom_trans_;
    }
    return true;
}

bool CBaseOdometry::GetOdomData(sensor::COdometryData& data)
{
    std::lock_guard<std::mutex> lock(odom_data_mtx);
    data.odom_flag = ODOMETRY_FLAG;
    data.time_stamp = m_CurTimeStamp.load();
    data.velocity = m_CurVel;
    data.local_pst = m_LocalOdomTrans;
    data.global_pst = m_EstiOdomPst;
    return true;
}

unsigned int CBaseOdometry::GetOdomFlag()
{
    unsigned int odom_flag = ODOMETRY_FLAG;
    return odom_flag;
}

void CBaseOdometry::SetAccumuOdom(long accuOdom)
{
    m_AccumuOdom = accuOdom;
}

bool CBaseOdometry::GetOdomPosture(unsigned long long odom_time, CPosture& odom_pst)
{
    bool bRet = false;
    odom_pst.SetPnt(0.0, 0.0);
    odom_pst.SetAngle(0.0);
    {
        std::lock_guard<std::mutex> lock(odom_data_mtx);

        COdometryFrame frontFrame;
        COdometryFrame backFrame;
        bRet = LookUpInterOdomFrame(odom_time, frontFrame, backFrame);
        if(!bRet) {
            return false;
        }

#ifdef USE_LINEAR_INTERPO
        double ratioFront = 0.0;
        double ratioBack = 0.0;
        bRet = LinearInterpo(odom_time, frontFrame.time_stamp, backFrame.time_stamp, ratioFront, ratioBack);
        if(!bRet) {
            ratioFront = 1.0;
            ratioBack = 0.0;
        }
        odom_pst.x = frontFrame.pose.x * ratioFront + backFrame.pose.x * ratioBack;
        odom_pst.y = frontFrame.pose.y * ratioFront + backFrame.pose.y * ratioBack;

        // 线性插值,角度需要标准化
        double norm_ori_front = 0.0;
        double norm_ori_back = 0.0;
        norm_ori_back = CAngle::NormalizeAngleDifference(backFrame.pose.fThita, frontFrame.pose.fThita);
        norm_ori_front = frontFrame.pose.fThita;

        odom_pst.fThita = norm_ori_front * ratioFront + norm_ori_back * ratioBack;
        odom_pst.fThita = CAngle::NormAngle(odom_pst.fThita);
#else
        odom_pst = frontFrame.pose;
#endif
    }
    return true;
}

long CBaseOdometry::GetAccumuOdom()
{
    return m_AccumuOdom.load();
}

bool CBaseOdometry::LinearInterpo(unsigned long long lCurTime, unsigned long long lFrontTime,
                                  unsigned long long lBackTime, double& ratioFront, double& ratioBack)
{
    if(lFrontTime <= lBackTime || lCurTime < lBackTime || lCurTime > lFrontTime) {
        ratioFront = 1.0;
        ratioBack = 0.0;
        return false;
    }

    ratioFront = static_cast<double>(lCurTime - lBackTime) / static_cast<double>(lFrontTime - lBackTime);
    ratioBack = static_cast<double>(lFrontTime - lCurTime) / static_cast<double>(lFrontTime - lBackTime);
    return true;
}

bool CBaseOdometry::LookUpInterVelocity(unsigned long long lCurTime, StVelocity& frontVel, StVelocity& backVel)
{
    bool bRet = false;
    if(m_Velocity.empty()) {
        return false;
    }

    //    for (auto rit = m_Velocity.rbegin(); rit != m_Velocity.rend(); ++rit) {
    //        *rit;
    //    }
    {
        std::lock_guard<std::mutex> lock(vel_mtx);

        if(lCurTime >= m_Velocity.back()->lRecvTime) {
            frontVel = *m_Velocity.back();
            backVel = *m_Velocity.back();
            if((lCurTime - m_Velocity.back()->lRecvTime) < VELO_DATA_VALID_TIME) {
                return true;
            }
            else {
                return false;
            }
        }

        if(lCurTime <= m_Velocity.front()->lRecvTime) {
            frontVel = *m_Velocity.front();
            backVel = *m_Velocity.front();
            if((m_Velocity.front()->lRecvTime - lCurTime) < VELO_DATA_VALID_TIME) {
                return true;
            }
            else {
                return false;
            }
        }

        // 速度矢量按时间序列排列，在速度矢量数据中为同步的时间点找到合适的时间位置
        // 即找到与同步时间相邻的左右两个数据
        // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做插值
        long lSize = static_cast<long>(m_Velocity.size());
        for (long i = lSize - 1; i >= 0; --i){
            if (m_Velocity[i] == NULL) {
                continue;
            }
            if(lCurTime < m_Velocity[i]->lRecvTime){
                continue;
            }
            if(lCurTime == m_Velocity[i]->lRecvTime){
                frontVel = *m_Velocity[i];
                backVel = *m_Velocity[i];
                bRet = true;
                break;
            }
            if((lCurTime - m_Velocity[i]->lRecvTime) > VELO_DATA_VALID_TIME){
                bRet = false;
                break;
            }

            if((i + 1) < lSize){
                if(lCurTime > m_Velocity[i + 1]->lRecvTime
                        || (m_Velocity[i + 1]->lRecvTime - lCurTime) > VELO_DATA_VALID_TIME){
                    bRet = false;
                    break;
                }
                frontVel = *m_Velocity[i + 1];
                backVel = *m_Velocity[i];
                bRet = true;
                break;
            }
        }

        if(!bRet) {
            frontVel = *m_Velocity.back();
            backVel = *m_Velocity.back();
        }
    }

    return bRet;
}

bool CBaseOdometry::LookUpInterVelocitySync(unsigned long long lCurTime, long long lSyncOffset, StVelocity& frontVel, StVelocity& backVel)
{
    bool bRet = false;
    if(m_Velocity.empty()) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(vel_mtx);

        unsigned long long sync_time = static_cast<unsigned long long>(static_cast<long long>(lCurTime) - lSyncOffset);
        if(sync_time >= m_Velocity.back()->lRawTime) {
            frontVel = *m_Velocity.back();
            backVel = *m_Velocity.back();
            if((sync_time - m_Velocity.back()->lRawTime) < VELO_DATA_VALID_TIME) {
                return true;
            }
            else {
                return false;
            }
        }

        if(sync_time <= m_Velocity.front()->lRawTime) {
            frontVel = *m_Velocity.front();
            backVel = *m_Velocity.front();
            if((m_Velocity.front()->lRawTime - sync_time) < VELO_DATA_VALID_TIME) {
                return true;
            }
            else {
                return false;
            }
        }

        // 速度矢量按时间序列排列，在速度矢量数据中为同步的时间点找到合适的时间位置
        // 即找到与同步时间相邻的左右两个数据
        // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做插值
        long lSize = static_cast<long>(m_Velocity.size());
        for (long i = lSize - 1; i >= 0; --i){
            if (m_Velocity[i] == NULL) {
                continue;
            }
            if(sync_time < m_Velocity[i]->lRawTime){
                continue;
            }
            if(sync_time == m_Velocity[i]->lRawTime){
                frontVel = *m_Velocity[i];
                backVel = *m_Velocity[i];
                bRet = true;
                break;
            }
            if((sync_time - m_Velocity[i]->lRawTime) > VELO_DATA_VALID_TIME){
                bRet = false;
                break;
            }

            if((i + 1) < lSize){
                if(sync_time > m_Velocity[i + 1]->lRawTime
                        || (m_Velocity[i + 1]->lRawTime - sync_time) > VELO_DATA_VALID_TIME){
                    bRet = false;
                    break;
                }
                frontVel = *m_Velocity[i + 1];
                backVel = *m_Velocity[i];
                bRet = true;
                break;
            }
        }

        if(!bRet) {
            frontVel = *m_Velocity.back();
            backVel = *m_Velocity.back();
        }
    }
    return bRet;
}

bool CBaseOdometry::LookUpInterGyroAngleSync(unsigned long long lCurTime, long long lSyncOffset, StVelocity& frontAngle, StVelocity& backAngle)
{
    bool bRet = false;
    if(m_Velocity.empty()) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(vel_mtx);

        unsigned long long sync_time = static_cast<unsigned long long>(static_cast<long long>(lCurTime) - lSyncOffset);
        if(sync_time >= m_Velocity.back()->lGyroTime) {
            frontAngle = *m_Velocity.back();
            backAngle = *m_Velocity.back();
            if((sync_time - m_Velocity.back()->lGyroTime) < GYRO_DATA_VALID_TIME) {
                return true;
            }
            else {
                return false;
            }
        }

        if(sync_time <= m_Velocity.front()->lGyroTime) {
            frontAngle = *m_Velocity.front();
            backAngle = *m_Velocity.front();
            if((m_Velocity.front()->lGyroTime - sync_time) < GYRO_DATA_VALID_TIME) {
                return true;
            }
            else {
                return false;
            }
        }

        // 速度矢量按时间序列排列，在速度矢量数据中为同步的时间点找到合适的时间位置
        // 即找到与同步时间相邻的左右两个数据
        // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做插值
        long lSize = static_cast<long>(m_Velocity.size());
        for (long i = lSize - 1; i >= 0; --i){
            if (m_Velocity[i] == NULL) {
                continue;
            }
            if(sync_time < m_Velocity[i]->lGyroTime){
                continue;
            }
            if(sync_time == m_Velocity[i]->lGyroTime){
                frontAngle = *m_Velocity[i];
                backAngle = *m_Velocity[i];
                bRet = true;
                break;
            }
            if((sync_time - m_Velocity[i]->lGyroTime) > GYRO_DATA_VALID_TIME){
                bRet = false;
                break;
            }

            if((i + 1) < lSize){
                if(sync_time > m_Velocity[i + 1]->lGyroTime
                        || (m_Velocity[i + 1]->lGyroTime - sync_time) > GYRO_DATA_VALID_TIME){
                    bRet = false;
                    break;
                }
                frontAngle = *m_Velocity[i + 1];
                backAngle = *m_Velocity[i];
                bRet = true;
                break;
            }
        }

        if(!bRet) {
            frontAngle = *m_Velocity.back();
            backAngle = *m_Velocity.back();
        }
    }
    return bRet;
}

StVelocity CBaseOdometry::GetBaseVelocity()
{
    if(!m_Velocity.empty()) {
        std::lock_guard<std::mutex> lock(vel_mtx);
        return *m_Velocity.back();
    }
    else {
        return StVelocity();
    }
}


} // namespace mapping

