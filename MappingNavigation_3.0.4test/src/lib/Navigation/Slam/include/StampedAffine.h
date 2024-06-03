#pragma once

#include "ZTypes.h"
#include "TimeStamp.h"
#include "Eigen/Eigen"

namespace ndt_oru
{

//
//   定义具有时间戳的姿态。
//
class DllExport CStampedAffine : public Eigen::Affine3d, public CTimeStamp
{
#ifndef NDT1_USE_MINI_EIGEN
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

  public:
    CStampedAffine(Eigen::Affine3d &affine, CTimeStamp &stamp) :
        Eigen::Affine3d(affine),
        CTimeStamp(stamp)
    {
    }

    CStampedAffine(Eigen::Affine3d &affine, unsigned int uTime) :
        Eigen::Affine3d(affine),
        CTimeStamp(uTime)
    {
    }

    CStampedAffine(Eigen::Affine3d &affine) : Eigen::Affine3d(affine) {}

    CStampedAffine() {}

    Eigen::Affine3d &GetAffine3dObject()
    {
        Eigen::Affine3d *p = static_cast<Eigen::Affine3d *>(this);
        return *p;
    }

    Eigen::Affine3d operator*(const Eigen::Affine3d &af)
    {
        Eigen::Affine3d *p = static_cast<Eigen::Affine3d *>(this);
        Eigen::Affine3d r = *p * af;
        return r;
    }
};

}    // namespace ndt_oru
