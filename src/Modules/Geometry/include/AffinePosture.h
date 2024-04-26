#ifndef __AffinePosture
#define __AffinePosture

#include "Geometry.h"
//#include "ndt_options.h"

#ifdef NDT1_USE_MINI_EIGEN
#include "ME_Transform.h"
#define Eigen MiniEigen
#else
#include <Eigen/Eigen>
#include <Eigen/Dense>
#endif

DllExport Eigen::Affine3d PostureToAffine(double x, double y, double yaw);
DllExport Eigen::Affine3d PostureToAffine(const CPosture& pst);
DllExport CPosture AffineToPosture(const Eigen::Affine3d& affine);

#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif

#endif
