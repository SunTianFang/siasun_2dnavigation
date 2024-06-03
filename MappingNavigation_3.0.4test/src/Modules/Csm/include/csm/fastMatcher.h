#ifndef FASTMatcher
#define FASTMatcher

#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/kdtree/io.h>
#include <vector>

#include <time.h>
#include "Scan.h"
#include "csm/CsmScan.h"

#define CORRESPONDENCE_XY_RATE  0.75        // 对应点对x,y方向的投影比例阈值

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 角度直方图
struct AngleHistogram
{
    // 直方图
    map<int, float> angleCounts;            // 角度范围编号,eg:360°分12个子空间，即30°一个范围，编号为1~12该角度范围的点的个数
    float mean = 0.f;                       // 平均值
    float var = 0.f;                        // 方差
    float sd = 0.f;                         // 标准差
    std::string covHistogramID;             // 计算相关性的histogram id
    float cov = 0.f;                        // 协方差 with covHistogramID
    float correlation_coefficient = 0.f;    // 相关性系数

};

// 储存x,y投影点数
struct XYProjection
{
//    vector<int> xCount;
//    vector<int> yCount;
    int xN;
    int yN;
    float rate;
};

class fastMatcher
{
public:
    PointCloud::Ptr sourcePC_;
    PointCloud::Ptr targetPC_;

    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals_;   // source点云法向量
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals_;   // target点云法向量

    PointCloud::Ptr csmCorrespondenceInSource;   // csm匹配中计算的对应点对（source点中）
    PointCloud::Ptr overlapTarget_;   // target点云中与source点云重合的部分点云
    PointCloud::Ptr overlapSource_;   // source点云中与target点云重合的部分点云
    int overlapSize;                  // target与source重合的点对数量（对应点对数）

//    XYProjection xyProjection_;

public:

    fastMatcher();

    // CsmScan -> pcl
    PointCloud::Ptr csmScan2pointCloud(CCsmScan* csmScan);

    // pcl -> CsmScan
    void pointCloud2CCsmScan(PointCloud::Ptr in, CCsmScan* csmScan );

    // 点云降采样
    void downSample(PointCloud::Ptr in, float radSearch = 0.1, int neighborNum = 2);
#if 0
    // 计算法线, pcl的法线对2d情况失效，都是“垂直”每一个点
    pcl::PointCloud<pcl::Normal>::Ptr normalEstimation(PointCloud::Ptr in);

    // 计算FPFH
    ///param@输入点云，法线特征，输出fpfh特征
    bool computeFPFH(PointCloud::Ptr in, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs);
#endif

#if 0
    // 粗匹配(2维情况失效）
    bool SACMatcher(PointCloud::Ptr targetPC, PointCloud::Ptr sourcePC);
    bool SACMatcher(CCsmScan* targetCS, CCsmScan* sourceCS);
#endif
    // 计算target与source点云重叠部分
    bool CorrespondenceEstimation(PointCloud::Ptr targetPC, PointCloud::Ptr sourcePC,
                                  PointCloud::Ptr overlapTarget, PointCloud::Ptr overlapSource, int &overlapSize);
    bool CorrespondenceEstimation(CCsmScan* targetCS, CCsmScan* sourceCS, CPosture poseIn);

    // 点云坐标变换
    PointCloud::Ptr transformPointCloud(PointCloud::Ptr cloudIn, CPosture poseIn);

    // 计算并比较（target，source）点云方向直方图（target or map坐标系）
    double computeDirectionalHistogram(CCsmScan* targetCS, CCsmScan* sourceCS, CPosture poseIn);
    double computeDirectionalHistogram(PointCloud::Ptr targetPC, PointCloud::Ptr sourcePC);

    // 2分法查找角度区间
    int binarySearch2(float array[], int n, float target);

    // 保存dx数据->pcd个数点云数据
//    bool save2pcd(PointCloud::Ptr targetPC, PointCloud::Ptr sourcePC);

    // 欧几里得聚类，返回点云的聚类个数
    /// 求点云投影方向的点的个数
    /// 用聚类方式为了减小直线垂直投影导致的多点聚集现象
    int euclideanCluster(PointCloud::Ptr inPC, std::vector<pcl::PointIndices> &cluster_indices,
                         double clusterRange = 0.005, int minClusterSize = 1, int maxClusterSize = 1000);

    PointCloud::Ptr get_overlapTarget()
    {
        return overlapTarget_;
    }

    PointCloud::Ptr get_overlapSource()
    {
        return overlapSource_;
    }

#if 0
    float CorrespondenceDirectionCompute(CCsmScan* targetCS, CCsmScan* sourceCS, CPosture poseIn);

    void clearXYProjection();
#endif

};

#endif
