//
//   The interface of class "CRawScan".
//

#pragma once

#include <vector>
#include <memory>

#include "Geometry.h"
#include "ScannerParam.h"


namespace sensor
{

class COdometryData
{
  public:
    unsigned int odom_flag;    // 4:有相对里程姿态数据和速度向量数据
    unsigned long long time_stamp;
    CVelocity velocity;
    CPosture local_pst;
    CPosture global_pst;

  public:
    COdometryData()
    {
        odom_flag = 4;
        time_stamp = 0;
    }

    void Clear()
    {
        odom_flag = 0;
        time_stamp = 0;
    }
    COdometryData(const COdometryData& other)
    {
        this->odom_flag = other.odom_flag;
        this->time_stamp = other.time_stamp;
        this->velocity = other.velocity;
        this->local_pst = other.local_pst;
        this->global_pst = other.global_pst;
    }

    COdometryData& operator = (const COdometryData& Obj)
    {
        this->odom_flag = Obj.odom_flag;
        this->time_stamp = Obj.time_stamp;
        this->velocity = Obj.velocity;
        this->local_pst = Obj.local_pst;
        this->global_pst = Obj.global_pst;
        return *this;
    }
};

class CRawPointCloud
{
  public:
    std::vector<unsigned short int> distance;    // 极坐标距离数据(为节省空间，使用unsigned short int类型)
    std::vector<unsigned short int> intensity;    // 激光强度数据(同上，unsigned short int类型)
    unsigned long long timestamp_raw;             // 点云数据的原始时间
    unsigned long long timestamp_sync;            // 点云数据的同步时间
    short laser_type;                             // 0 - pf, 1 - hokuyo, 2 - sick
    short laser_id;                               // 扫描器标识
    unsigned int num_points;                      // 扫描线数
    float start_angle;                            // 扫描起始角(单位：弧度)
    float end_angle;                              // 扫描终止角(单位：弧度)

  public:
    CRawPointCloud()
    {
        Clear();
        timestamp_raw = 0;
        timestamp_sync = 0;
        laser_type = 0;
        laser_id = 0;
        num_points = 0;
        start_angle = 0.0;
        end_angle = 0.0;
    }

    void Clear()
    {
        distance.clear();
        intensity.clear();
    }

    CRawPointCloud(const CRawPointCloud& other)
    {
        Clear();
        this->distance = other.distance;
        this->intensity = other.intensity;
        this->timestamp_raw = other.timestamp_raw;
        this->timestamp_sync = other.timestamp_sync;
        this->laser_type = other.laser_type;
        this->laser_id = other.laser_id;
        this->num_points = other.num_points;
        this->start_angle = other.start_angle;
        this->end_angle = other.end_angle;
    }

    CRawPointCloud(CRawPointCloud&& other)
    {
        Clear();
        this->distance = other.distance;
        this->intensity = other.intensity;
        this->timestamp_raw = other.timestamp_raw;
        this->timestamp_sync = other.timestamp_sync;
        this->laser_type = other.laser_type;
        this->laser_id = other.laser_id;
        this->num_points = other.num_points;
        this->start_angle = other.start_angle;
        this->end_angle = other.end_angle;
    }

    CRawPointCloud& operator = (const CRawPointCloud& Obj)
    {
        Clear();
        this->distance = Obj.distance;
        this->intensity = Obj.intensity;
        this->timestamp_raw = Obj.timestamp_raw;
        this->timestamp_sync = Obj.timestamp_sync;
        this->laser_type = Obj.laser_type;
        this->laser_id = Obj.laser_id;
        this->num_points = Obj.num_points;
        this->start_angle = Obj.start_angle;
        this->end_angle = Obj.end_angle;
        return *this;
    }

    CRawPointCloud& operator = (CRawPointCloud&& Obj)
    {
        Clear();
        this->distance = Obj.distance;
        this->intensity = Obj.intensity;
        this->timestamp_raw = Obj.timestamp_raw;
        this->timestamp_sync = Obj.timestamp_sync;
        this->laser_type = Obj.laser_type;
        this->laser_id = Obj.laser_id;
        this->num_points = Obj.num_points;
        this->start_angle = Obj.start_angle;
        this->end_angle = Obj.end_angle;
        return *this;
    }

    // 返回点云的角度分辨率(考虑到同一激光器可能采用不同的扫描分辨率，不能以固定参数返回)
    float AngularReso() const
    {
        return (end_angle - start_angle) / distance.size();
    }

    // 从二进制文件中读取扫描数据
    bool LoadBinary(FILE *fp, float fStartAngle, float fEndAngle, int nLineCount, int nFileVersion);

    // 将扫描数据保存到二进制文件
    bool SaveBinary(FILE *fp, int nFileVersion);
};

using RawPointCloudVect = std::vector<std::shared_ptr<CRawPointCloud>>;

class CRawScan
{
  public:
    RawPointCloudVect point_cloud;
    COdometryData odom_data;

  public:
    CRawScan()
    {
        point_cloud.clear();
        odom_data.Clear();
    }

    ~CRawScan()
    {
        Clear();
    }

    CRawScan(const CRawScan &other)
    {
        *this = other;
    }

    CRawScan &operator=(const CRawScan &Obj)
    {
        Clear();
        point_cloud = Obj.point_cloud;
        odom_data = Obj.odom_data;
        return *this;
    }

    void Clear()
    {
        for (unsigned int i = 0; i < point_cloud.size(); i++)
        {
            if (point_cloud[i])
            {
                point_cloud[i].reset();
                point_cloud[i] = nullptr;
            }
        }
        point_cloud.clear();
    }

    bool SetOdometry(COdometryData &odom)
    {
        odom_data = odom;
        return true;
    }

    bool SetPointCloud(RawPointCloudVect &pCloud)
    {
        point_cloud = pCloud;
        return true;
    }

    bool PushBackPointCloud(std::shared_ptr<CRawPointCloud> &pCloud)
    {
        point_cloud.push_back(pCloud);
        return true;
    }

    // 从二进制文件中读取扫描数据
    bool LoadBinary(FILE *fp, const CPosture &pstRobot, const CScannerGroupParam &ScannerParam, int nFileVersion);

    // 将扫描数据保存到二进制文件
    bool SaveBinary(FILE *fp, int nFileVersion);
};

}    // namespace sensor
