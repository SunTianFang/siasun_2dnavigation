namespace ndt_oru
{

template <typename PointT>
void transformPointCloudInPlace(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> &Tr,
                                std::vector<Eigen::Vector3d> &pc)
{
    Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> T = Tr.cast<float>();
    for (int i = 0; i < (int)pc.size(); i++)
    {
#ifdef NDT1_USE_MINI_EIGEN
        PointT &thisPoint = pc.points[i];

        Eigen::Vector3f pt(thisPoint.x, thisPoint.y, thisPoint.z);

        pt = T * pt;

        thisPoint.x = pt(0);
        thisPoint.y = pt(1);
        thisPoint.z = pt(2);
#else
        Eigen::Map<Eigen::Vector3f> pt((float *)&pc.points[i], 3);
        pt = T * pt;
#endif
    }
}

template <typename PointT>
double geomDist(PointT p1, PointT p2)
{
    Eigen::Vector3d v;
    v << p1.x - p2.x, p1.y - p2.y, p1.z - p2.z;
    return v.norm();
}

}    // namespace perception_oru
