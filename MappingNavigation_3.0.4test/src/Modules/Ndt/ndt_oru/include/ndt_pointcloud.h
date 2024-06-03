#ifndef __NdtPointCloud
#define __NdtPointCloud

#include <vector>
#include "ZTypes.h"
#include "ndt_options.h"
#include "TimeStamp.h"

#ifdef NDT1_USE_MINI_EIGEN
#include "ME_Vector.h"
#include "ME_Transform.h"
#define Eigen MiniEigen
#else
#include <Eigen/Eigen>
#endif

class CScan;

namespace ndt_oru
{

///////////////////////////////////////////////////////////////////////////////
//   应用于NDT算法中的点云(它是下面两种点云的基类)。
#ifdef NDT1_USE_MINI_EIGEN
class DllExport CPointCloud : public std::vector<Eigen::Vector3d>
#else
class DllExport CPointCloud : public std::vector<Eigen::Vector3d>//, Eigen::aligned_allocator<Eigen::Vector3d>>
#endif
{
  public:
    CPointCloud(const CScan &Scan);
    CPointCloud() {}

    void operator += (const CPointCloud &other);

    void CreateFromScan(const CScan &Scan);
    void ToScan(CScan &Scan);
    bool ToScan(CScan *Scan)const;
    int  ToScanOnlyHighIntensity(CScan *Scan)const;
    std::vector<int> vecIntensity; //sfe1012 for feature match
};

///////////////////////////////////////////////////////////////////////////////
//   定义具有时间戳的点云。
class DllExport CStampedPointCloud : public CPointCloud, public CTimeStamp
{
  public:
    CStampedPointCloud(const CStampedPointCloud &another);
    CStampedPointCloud();

    CStampedPointCloud &operator=(const CStampedPointCloud &another);
    void operator+=(const CStampedPointCloud &another);

    void Clear();

    CStampedPointCloud &GetStampedPointCloud() { return *this; }

  private:
    void CopyCloud(const CPointCloud &cloud);
};

///////////////////////////////////////////////////////////////////////////////
//   标记好传感器编号的点云。
class DllExport CLabeledPointCloud : public CStampedPointCloud
{
  public:
    CLabeledPointCloud(int nScannerId)
    {
        m_nScannerId = nScannerId;
        Clear();
    }

    CLabeledPointCloud(const CLabeledPointCloud &other)
    {
        Clear();
        m_nScannerId = other.m_nScannerId;
        GetStampedPointCloud() = ((CLabeledPointCloud&)other).GetStampedPointCloud();
    }

  public:
    unsigned int m_nScannerId;    // 激光扫描器编号
};

}    // namespace ndt_oru

#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif

#endif
