#include "fastMatcher.h"
#include "AffinePosture.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

fastMatcher::fastMatcher()
{
    sourcePC_.reset(new PointCloud());
    targetPC_.reset(new PointCloud());

    csmCorrespondenceInSource.reset(new PointCloud);

    sourceNormals_.reset(new pcl::PointCloud< pcl::Normal>);
    targetNormals_.reset(new pcl::PointCloud< pcl::Normal>);

    overlapTarget_.reset(new PointCloud());
    overlapSource_.reset(new PointCloud());

}

// 点云降采样
void fastMatcher::downSample(PointCloud::Ptr in, float radSearch, int neighborNum)
{

    if (!in->empty())
    {
        // 半径滤波
        pcl::RadiusOutlierRemoval<PointT> sor;
        sor.setInputCloud(in);
        sor.setRadiusSearch(radSearch);              // 设置滤波半径(m)
        sor.setMinNeighborsInRadius(neighborNum);    // 设置查询点的临近点集小于5的删除
        sor.setNegative(false);                      // 默认false:保存内点；true:保存滤掉的外点
        sor.filter(*in);
    }
}
#if 0
// 计算表面法线
pcl::PointCloud<pcl::Normal>::Ptr fastMatcher::normalEstimation(PointCloud::Ptr in)
{
    //去除NAN点
//    std::vector<int> indices_src;  // 存储去除的点的索引
//    pcl::removeNaNFromPointCloud(*in, *in, indices_src);

    // 计算法线（PCA）
    pcl::NormalEstimation<PointT,pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(in);
    pcl::search::KdTree< PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    normalEstimation.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud< pcl::Normal>);  // 法线对象
    // 每个点采用半径2cm的近邻搜索
    normalEstimation.setRadiusSearch(0.02);
    normalEstimation.compute(*normals);

    return normals;
}

// 计算FPFH
bool fastMatcher::computeFPFH(PointCloud::Ptr in,
                                          pcl::PointCloud<pcl::Normal>::Ptr normals,
                                          pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs)
{
    pcl::FPFHEstimation<PointT,pcl::Normal,pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(in);
    fpfh.setInputNormals(normals);
    pcl::search::KdTree<PointT>::Ptr tree_fpfh (new pcl::search::KdTree<PointT>);
    fpfh.setSearchMethod(tree_fpfh);
    fpfh.setRadiusSearch(0.05);
    fpfh.compute(*fpfhs);
//    std::cout<<"compute  fpfh"<< std::endl;

    return true;

}
#endif

#if 0
// SAC粗匹配
bool fastMatcher::SACMatcher(CCsmScan* targetCS, CCsmScan* sourceCS)
{
    PointCloud::Ptr targetPC(new PointCloud);
    PointCloud::Ptr sourcePC(new PointCloud);
    targetPC = csmScan2pointCloud(targetCS);
    sourcePC = csmScan2pointCloud(sourceCS);

    return SACMatcher(targetPC, sourcePC);
}

bool fastMatcher::SACMatcher(PointCloud::Ptr targetPC, PointCloud::Ptr sourcePC)
{
    clock_t start_time = clock();

    targetPC_ = targetPC;
    sourcePC_ = sourcePC;

    downSample(targetPC_);
    downSample(sourcePC_);
    if (targetPC_->empty() || sourcePC_->empty())
        return false;
    // 计算法线
    sourceNormals_ = normalEstimation(sourcePC_);
    targetNormals_ = normalEstimation(targetPC_);

//    cout << "---------------------------------------" << endl;
//    cout << "normal size : " << sourceNormals_->size() << endl;
//    for (int i = 0; i < sourceNormals_->size(); i++)
//    {
//        cout << "points: " << sourceNormals_->points[i] << endl;
//        cout << "curvate: " << sourceNormals_->points[i]._Normal::curvature << endl;
//    }

    // 计算FPFH   
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());

    computeFPFH(sourcePC_, sourceNormals_, fpfhs_src);
    computeFPFH(targetPC_, targetNormals_, fpfhs_tgt);
    // SAC粗匹配
    pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> scia;
    scia.setInputSource(sourcePC_);
    scia.setInputTarget(targetPC_);
    scia.setSourceFeatures(fpfhs_src);
    scia.setTargetFeatures(fpfhs_tgt);
    //scia.setMinSampleDistance(1);
    //scia.setNumberOfSamples(2);
    //scia.setCorrespondenceRandomness(20);
    PointCloud::Ptr sac_result (new PointCloud);
    scia.align(*sac_result);

    Eigen::Matrix4f sac_trans;
    sac_trans=scia.getFinalTransformation();

    Eigen::Affine3f sac_res;
    sac_res = sac_trans;
    double x = sac_res.translation().x();
    double y = sac_res.translation().y();
    double fThita = sac_res.rotation().eulerAngles(0, 1, 2)(2);
//    std::cout  <<"sac has converged:"<<scia.hasConverged()<<"  score: "<<scia.getFitnessScore()<< std::endl;
    std::cout << "sac: " << x << " " << y << " " << fThita << std::endl;

    clock_t end_time = clock();
    std::cout << "sac time: " << (double)(end_time - start_time)/(double)CLOCKS_PER_SEC<<" s"<< std::endl;

    return true;
}
#endif

PointCloud::Ptr fastMatcher::csmScan2pointCloud(CCsmScan *csmScan)
{
    PointCloud::Ptr pc(new PointCloud);

    for (int i =0; i < csmScan->m_nRays; i++)
    {
        PointT tp;
        tp.x = csmScan->m_pPoints[i].points.p[0];
        tp.y = csmScan->m_pPoints[i].points.p[1];
        tp.z = 0.f;
        if (isnan(tp.x) || isnan(tp.y) || isinf(tp.x) || isinf(tp.y))
            continue;
        pc->push_back(tp);
    }

    return pc;

}

void fastMatcher::pointCloud2CCsmScan(PointCloud::Ptr in, CCsmScan *csmScan)
{
    /// 用overlap的点重新做csm匹配，因此只改变CCsmScan中的m_pPoints的值
    /// 其他的值如：estimate,odometry 另外在最外层赋值，此函数只改变点云
    csmScan->Clear();

    csmScan->Alloc(in->points.size());

    double min_reading = 0;
    double max_reading = 30;

    for (int i = 0; i < (int)in->points.size(); i++)
    {
        CPnt pt;
        pt.x = in->points[i].x;
        pt.y = in->points[i].y;
        pt.UpdatePolar();
        float reading = pt.r;

        csmScan->m_pPoints[i].valid = (reading > min_reading) && (reading < max_reading);
        csmScan->m_pPoints[i].readings =csmScan->m_pPoints[i].valid ? reading : NAN;
        csmScan->m_pPoints[i].theta = pt.a;
/*
        if (i >= 5 & i < nCount - 6)
        {
              // 计算曲率值, 通过当前点前后5个点距离值的偏差程度来代表曲率
            float diff_range = Scan.m_pPoints[j - 5].r + Scan.m_pPoints[j - 4].r +
                               Scan.m_pPoints[j - 3].r + Scan.m_pPoints[j - 2].r +
                               Scan.m_pPoints[j - 1].r - Scan.m_pPoints[j].r * 10 +
                               Scan.m_pPoints[j + 1].r + Scan.m_pPoints[j + 2].r +
                               Scan.m_pPoints[j + 3].r + Scan.m_pPoints[j + 4].r +
                               Scan.m_pPoints[j + 5].r;
            // diffX * diffX + diffY * diffY
            m_pPoints[i].curvature = diff_range * diff_range;
        }
        else
            m_pPoints[i].curvature = -1;  // 前后5个点曲率给-1
*/
    }
}

bool fastMatcher::CorrespondenceEstimation(PointCloud::Ptr targetPC, PointCloud::Ptr sourcePC,
                                                       PointCloud::Ptr overlapTarget, PointCloud::Ptr overlapSource, int &overlapSize)
{
        pcl::registration::CorrespondenceEstimation<PointT, PointT>core;
        core.setInputSource(sourcePC);
        core.setInputTarget(targetPC);

        for (int i = 0; i < (int)sourcePC->points.size(); i++)
        {

           float x = sourcePC->points[i].x;
           float y = sourcePC->points[i].y;
           float z = sourcePC->points[i].z;

            if (is_nan(x) || is_nan(y) || is_nan(z))
                return false;

            if((isinf(x)) || (isinf(y)) || (isinf(z)))
                return false;

        }

        for (int i = 0; i < (int)targetPC->points.size(); i++)
        {

           float x = targetPC->points[i].x;
           float y = targetPC->points[i].y;
           float z = targetPC->points[i].z;

            if (is_nan(x) || is_nan(y) || is_nan(z))
                return false;

            if((isinf(x)) || (isinf(y)) || (isinf(z)))
                return false;

        }


        boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);   //共享所有权的智能指针，以kdtree做索引

        core.determineReciprocalCorrespondences(*cor, 0.01);   //点之间的最大距离0.01m,cor对应索引

        overlapSize = cor->size();     // 对应点对大小

        //构造重叠点云
        /// pcl::correspondences类里面有query和match两个成员，分别是sourc和target点云上对应点对的索引
        overlapTarget->width = cor->size();
        overlapTarget->height = 1;
        overlapTarget->is_dense = false;
        overlapTarget->points.resize(overlapTarget->width * overlapTarget->height);

        overlapSource->width = cor->size();
        overlapSource->height = 1;
        overlapSource->is_dense = false;
        overlapSource->points.resize(overlapSource->width * overlapSource->height);

        for (size_t i = 0; i < cor->size(); i++)
        {
            overlapTarget->points[i].x = targetPC->points[cor->at(i).index_query].x;
            overlapTarget->points[i].y = targetPC->points[cor->at(i).index_query].y;
            overlapTarget->points[i].z = targetPC->points[cor->at(i).index_query].z;

            overlapSource->points[i].x = sourcePC->points[cor->at(i).index_match].x;
            overlapSource->points[i].y = sourcePC->points[cor->at(i).index_match].y;
            overlapSource->points[i].z = sourcePC->points[cor->at(i).index_match].z;
        }
        return true;

}

bool fastMatcher::CorrespondenceEstimation(CCsmScan* targetCS, CCsmScan* sourceCS, CPosture poseIn)
{
    PointCloud::Ptr targetPC(new PointCloud);
    PointCloud::Ptr sourcePC(new PointCloud);
    targetPC = csmScan2pointCloud(targetCS);
    sourcePC = csmScan2pointCloud(sourceCS);

    sourcePC = transformPointCloud(sourcePC, poseIn);

    overlapTarget_->clear();
    overlapSource_->clear();

    return CorrespondenceEstimation(targetPC, sourcePC, overlapTarget_, overlapSource_, overlapSize);
}

#if 0
/// 即保证x,y方向上的匹配点对数量有保证（单一方向的匹配认为是不对的）
/// 返回值: x,y方向投影的比率
/// （弃用）
float fastMatcher::CorrespondenceDirectionCompute(CCsmScan *targetCS, CCsmScan *sourceCS, CPosture poseIn)
{
    /// @abstract
    /// 1.对target，source点云的覆盖部分（匹配点对）进行去除离群点滤波，再聚类。（聚类簇适当多一些）
    /// 2.对每个聚类簇进行投影，并计算x,y方向的投影数量
    /// 3.判断每个点簇的x,y方向投影的比例，如果比例接近，则认为是方向贡献无效的，一般可能为斜线（45°）情况，该点簇不作为方向点的统计
    /// 4.如果x,y比例差距较大，说明此点簇对方向性的贡献较大，保留x,y方向的投影点个数
    /// 5.最后统计x,y方向的投影点个数，最为质量评估的判断
    /// 6.如果x,y方向点都接近0，说明是长廊的斜线，此时方向性判断失效

    // 1.求csm的匹配点对
    csmCorrespondenceInSource->clear();
    for(int i = 0; i < sourceCS->m_nRays; i++)
    {
        if(sourceCS->m_pPoints[i].corr.valid)
        {
            PointT tp;
            tp.x = sourceCS->m_pPoints[i].points_w.p[0];
            tp.y = sourceCS->m_pPoints[i].points_w.p[1];
            tp.z = 0.0;
            csmCorrespondenceInSource->push_back(tp);
        }
    }
    cout << "source m_nRays: " << sourceCS->m_nRays << " " << "csmCorrespondenceInSource: " << csmCorrespondenceInSource->points.size() << endl;

    // 对匹配点对进行聚类
    if (csmCorrespondenceInSource->empty())
        return false;
    std::vector<pcl::PointIndices> cluster_indices;
    if (!euclideanCluster(csmCorrespondenceInSource, cluster_indices, 0.50, 10, 60))
        return false;
    // 如果聚类不为0,则对每个聚类簇求x,y投影比例
    cout << "cluster_indices size: " << cluster_indices.size() << endl;

    clearXYProjection();  // 清空xy投影数量

    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
        PointCloud::Ptr tpClusterPointCloud(new PointCloud);
        // 获取每个点簇的点云
        for (auto pit = cluster_indices[i].indices.begin(); pit != cluster_indices[i].indices.end(); ++pit)
        {
            PointT p;
            p.x = csmCorrespondenceInSource->points[*pit].x;
            p.y = csmCorrespondenceInSource->points[*pit].y;
            p.z = 0.0;
            tpClusterPointCloud->points.push_back(p);
        }
        // 计算每个点簇的点云x,y方向投影
        PointCloud::Ptr sourceX(new PointCloud);
        PointCloud::Ptr sourceY(new PointCloud);
        int xNum = 0;        // x方向投影点个数
        int yNum = 0;        // y方向投影点个数
        float xyRate = 0.0;  // x,y点数投影比例
        for (PointCloud::iterator it = tpClusterPointCloud->begin(); it != tpClusterPointCloud->end(); it++)
        {
            if(isnan(it->x) || isnan(it->y))
                continue;

            PointT pt;
            // x方向投影
            pt.x = it->x;
            pt.y = 0.0;
            pt.z = 0.0;
            sourceX->push_back(pt);

            // y方向投影
            pt.x = 0.0;
            pt.y = it->y;
            pt.z = 0.0;
            sourceY->push_back(pt);
        }

        // 聚类求投影点的个数
        std::vector<pcl::PointIndices> xIndices, yIndices;
        xNum = euclideanCluster(sourceX, xIndices);                 // 求x,y方向投影时，聚类使用默认参数即可
        yNum = euclideanCluster(sourceY, yIndices);
        // xNum,yNum都不为0，表示该聚类点簇不是垂直坐标轴方向的分布，需要判断x,y的分配比例
        // 如果分配比例近似相等，则认为此点簇是近似45°倾斜的，也就是该点簇点对分配没有作用，不记录个数
        if (xNum != 0 && yNum !=0)
        {
            if (xNum <= yNum)
                xyRate = (float)xNum / (float)yNum;
            else
                xyRate = (float)yNum / (float)xNum;

            if (xyRate > CORRESPONDENCE_XY_RATE)       // 计算每个点簇x,y方向的投影点数比例
            {
                // 如果x,y比例 > 0.75时，认为比例是不一样的，此时的数量可以作为参考
                xyProjection_.xN += xNum;
                xyProjection_.yN += yNum;
            }
        }
        else
        {
            // xNum,yNum有一个为0，则表示该点簇是单一方向的，记录个数
            xyProjection_.xN += xNum;
            xyProjection_.yN += yNum;
        }
    }
    // x,y方向投影点都不为0时，说明匹配点对x,y方向都有分配，质量评估初步可信，具体分配点数的比例阈值默认10%，
    // 即x或y方向占另一个方向点数超过10%即认为方向分配可用，否则认为单一方向点数过多，匹配结果不可信
    // 如果 xyProjection_.xN == 0 || xyProjection_.yN ==0，则认为有一个方向投影全是0，匹配结果不可信，类似长走廊环境
    if (xyProjection_.xN != 0 && xyProjection_.yN !=0)
    {
        if (xyProjection_.xN >= xyProjection_.yN)
            xyProjection_.rate = (float)xyProjection_.yN / (float)xyProjection_.xN;
        else
            xyProjection_.rate = (float)xyProjection_.xN / (float)xyProjection_.yN;

        return xyProjection_.rate;
    }

    return 0;
}
#endif

// pcl欧几里得聚类
/// @param
/// 参数：输入的待聚类点云，聚类索引vector(聚类结果保存在其中),聚类半径，满足一个聚类的最小点数，满足一个聚类的最大点数
/// 返回聚类个数
int fastMatcher::euclideanCluster(PointCloud::Ptr inPC, std::vector<pcl::PointIndices> &cluster_indices,
                                  double clusterRange, int minClusterSize, int maxClusterSize)
{
    if (inPC->empty())
        return 0;

    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(inPC);
//    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;  // 欧几里得聚类
    ec.setClusterTolerance(clusterRange);        // 设置近邻搜索半径为0.005m，即5mm
    ec.setMinClusterSize(minClusterSize);        // 聚类需要的最少点数为1
    ec.setMaxClusterSize(maxClusterSize);        // 聚类需要的最大点数
    ec.setSearchMethod(tree);                    // 设置点云的搜索机制
    ec.setInputCloud(inPC);
    ec.extract(cluster_indices);                 // 提取聚类

    return cluster_indices.size();
}

// 点云坐标变换
PointCloud::Ptr fastMatcher::transformPointCloud(PointCloud::Ptr cloudIn, CPosture poseIn)
{
    pcl::PointCloud<PointT>::Ptr cloudOut(new pcl::PointCloud<PointT>());
    Eigen::Affine3d transPose;

//    transPose = PostureToAffine(poseIn);

    PointT pointTo;
    double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
    Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);

    Eigen::Vector3d eulerAngle(poseIn.fThita, 0.f, 0.f);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ()));
    q_w_curr= yawAngle * pitchAngle * rollAngle;

    t_w_curr.x() = poseIn.x;
    t_w_curr.y() = poseIn.y;
    t_w_curr.z() = 0.f;

    for (int i = 0; i < cloudSize; i++)
    {
        Eigen::Vector3d point_curr(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z);
        Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;

        pointTo.x = point_w.x();
        pointTo.y = point_w.y();
        pointTo.z = point_w.z();
//        pointTo.intensity = cloudIn->points[i].intensity;

        cloudOut->points[i] = pointTo;
    }
    return cloudOut;
}

// 二分查找
/*
非递归的二分查找
arrat：数组 ， n:数组的大小;  target:查找的数据； 返回target所在数组的下标
*/
int fastMatcher::binarySearch2(float array[], int n, float target)
{
    int low = 0, high = n, middle = 0;
    while(low < high)
    {
        middle = (low + high)/2;
        if (target > array[middle])
        {
            low = middle;
            if (low + 1 == high)
            {
                // 落在区间[low,high]中，对应区号为low
                return low;
            }
        }
        else if (target < array[middle])
        {
            high = middle;
            if (high - 1 == low)
            {
                return low;
            }
        }
        else if (target == array[middle])
        {
            if (middle == 0)
                return middle;
            else
                return middle - 1;
        }
    }
    return -1;
}

// 计算点云的方向直方图
double fastMatcher::computeDirectionalHistogram(CCsmScan *targetCS, CCsmScan *sourceCS, CPosture poseIn)
{
    PointCloud::Ptr targetPC(new PointCloud);
    PointCloud::Ptr sourcePC(new PointCloud);
    targetPC = csmScan2pointCloud(targetCS);
    sourcePC = csmScan2pointCloud(sourceCS);
    // 将sourcePC转换至targetPC坐标系下，poseIn是csm的匹配结果（△）
    sourcePC = transformPointCloud(sourcePC, poseIn);

#if 0
    // 测试用，保存两帧点云数据为pcd格式
    if (!save2pcd(targetPC, sourcePC))
        cout << "save pcd fail" << endl;
#endif

    return computeDirectionalHistogram(targetPC, sourcePC);
}

// 计算点云方向直方图，并计算target，source点云直方图的相关性
double fastMatcher::computeDirectionalHistogram(PointCloud::Ptr targetPC, PointCloud::Ptr sourcePC)
{
    if (targetPC->empty() || sourcePC->empty())
        return 0;
    // down sample
    downSample(targetPC, 0.05, 10);
    downSample(sourcePC, 0.05, 10);
    // 30 60 90 120 150 180 210 240 270 300 330 360,12个区间转换为rad（弧度）生成angleRange数组
    float angleRange[] = {0, 0.52, 1.05, 1.57, 2.09, 2.62, 3.14, 3.67, 4.19, 4.71, 5.24, 5.76, 6.28};
    int angleRangeN = 12;
    AngleHistogram targetAngleH, sourceAngleH;
    // 初始化直方图
    for (int i = 0; i < angleRangeN; i++)
    {
        targetAngleH.angleCounts[i] = 0.f;
        sourceAngleH.angleCounts[i] = 0.f;
    }
    // compute direction histogram
    /// 计算统一至target坐标系下的两个点云的方向直方图
    for (PointCloud::iterator it = targetPC->points.begin(); it != targetPC->points.end(); it++)
    {
        double angle = CAngle::NormAngle(atan2(it->y, it->x));
        int angleNum = binarySearch2(angleRange, angleRangeN, angle);
        if (angleNum >= 0)
        {
            targetAngleH.angleCounts[angleNum]++;
        }
    }

    for (PointCloud::iterator it = sourcePC->points.begin(); it != sourcePC->points.end(); it++)
    {
        double angle = CAngle::NormAngle(atan2(it->y, it->x));
        int angleNum = binarySearch2(angleRange, angleRangeN, angle);
        if (angleNum >= 0)
        {
            sourceAngleH.angleCounts[angleNum]++;
        }
    }

    // 直方图归一化至区间[0,1]
    // 公式:
    float maxT = 0;
    float maxS = 0;
    for (int i = 0; i < angleRangeN; i++)
    {
        // target
        if (targetAngleH.angleCounts[i] > maxT)
        {
            maxT = targetAngleH.angleCounts[i];
        }
        // source
        if (sourceAngleH.angleCounts[i] > maxS)
        {
            maxS = sourceAngleH.angleCounts[i];
        }
    }

    for (int i = 0; i < angleRangeN; i++)
    {
        // target
        float &tp = targetAngleH.angleCounts[i];
        if (tp < 0)
        {
            tp = 0;
        }
        else if (tp >= 0 && tp < maxT)
        {
            tp = tp * (1.f / maxT);
        }
        else
        {
            tp = 1;
        }

        //source
        float &sp = sourceAngleH.angleCounts[i];
        if (sp < 0)
        {
            sp = 0;
        }
        else if (sp >= 0 && sp < maxS)
        {
            sp = sp * (1.f / maxS);
        }
        else
        {
            sp = 1;
        }
    }

    // 打印每个bar的数据
/*
    for (int i = 0; i < (int)targetAngleH.angleCounts.size(); i++)
    {
        std::cout << "targetAngleH.angleCounts[" << i << "]" << targetAngleH.angleCounts[i] << std::endl;
        std::cout << "sourceAngleH.angleCounts[" << i << "]" << sourceAngleH.angleCounts[i] << std::endl;
    }
*/
    // 计算相关性系数
    // 公式:
    ///
    /// d(H1,H2) = ∑[(H1(i) - average(H1)) × (H2(i) - average(H2))] / sqrt[∑(H1(i) - average(H1))² × ∑(H2(i) - average(H2))²]
    /// 越接近1越相关
    double d = 0;          // 返回相关性系数
    for (int i = 0; i < angleRangeN; i++)
    {
        targetAngleH.mean += targetAngleH.angleCounts[i];
        sourceAngleH.mean += sourceAngleH.angleCounts[i];
    }

    targetAngleH.mean /= angleRangeN;
    sourceAngleH.mean /= angleRangeN;

    // 计算相关性公式中分子，分母的三个和
    float s11 = 0, s12 = 0, s22 = 0;
    for (int i = 0; i < angleRangeN; i++)
    {
        s11 += (targetAngleH.angleCounts[i] - targetAngleH.mean) * (targetAngleH.angleCounts[i] - targetAngleH.mean);
        s12 += (targetAngleH.angleCounts[i] - targetAngleH.mean) * (sourceAngleH.angleCounts[i] - sourceAngleH.mean);  // 分子
        s22 += (sourceAngleH.angleCounts[i] - sourceAngleH.mean) * (sourceAngleH.angleCounts[i] - sourceAngleH.mean);
    }
    double s11xs22 = s11 * s22;
    if (std::abs(s11xs22) > DBL_EPSILON)
    {
        d = s12 / std::sqrt(s11xs22);
    }
    else
    {
        d = 1.f;
    }

    return d;

}

// 保存dx数据为pcd点云数据
//bool fastMatcher::save2pcd(PointCloud::Ptr targetPC, PointCloud::Ptr sourcePC)
//{
//    if (targetPC->empty() && sourcePC->empty())
//        return false;

//    if (pcl::io::savePCDFileASCII("target.pcd", *targetPC) == -1)
//        return false;

//    if (pcl::io::savePCDFileASCII("source.pcd", *sourcePC) == -1)
//        return false;

//    return true;
//}

#if 0
void fastMatcher::clearXYProjection()
{
//    vector<int>().swap(xyProjection_.xN);
//    vector<int>().swap(xyProjection_.yN);
    xyProjection_.xN = 0;
    xyProjection_.yN = 0;
    xyProjection_.rate = 0.0;
}
#endif
