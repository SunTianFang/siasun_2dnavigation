#include "AffinePosture.h"

#ifdef NDT1_USE_MINI_EIGEN
#define Eigen MiniEigen
#endif

///////////////////////////////////////////////////////////////////////////////

Eigen::Affine3d PostureToAffine(double x, double y, double yaw)
{
#ifdef NDT1_USE_MINI_EIGEN
    Eigen::Affine3d T(x, y, yaw);
#else
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Translation3d v(x, y, 0);
    Eigen::Affine3d T = v * m;

#endif

    return T;
}

Eigen::Affine3d PostureToAffine(const CPosture &pst)
{
    return PostureToAffine(pst.x, pst.y, pst.fThita);
}

CPosture AffineToPosture(const Eigen::Affine3d &affine)
{
    CPosture pst(0,0,0);
	
#ifdef NDT1_USE_MINI_EIGEN
    pst.x = (float)affine.getx();
    pst.y = (float)affine.gety();
    pst.fThita = (float)affine.getth();
#else
    pst.x = (float)affine.translation().x();
    pst.y = (float)affine.translation().y();
    pst.fThita = (float)affine.rotation().eulerAngles(0, 1, 2)(2);
#endif

    return pst;
}
