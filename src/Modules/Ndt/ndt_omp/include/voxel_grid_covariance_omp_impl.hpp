#pragma once

#include <pcl/common/common.h>
#include <pcl/filters/boost.h>
#include "voxel_grid_covariance_omp.h"
#include <Eigen/Dense>
#include <Eigen/Cholesky>

//////////////////////////////////////////////////////////////////////////////////////////

//
//   取得对应于本单元的概率圆。
//
template <typename PointT>
bool ndt_omp::Leaf<PointT>::getEllipse(CEllipse &ellipse)
{
    Eigen::Vector3d mean = getMean();
    Eigen::Matrix2d cov(cov_.block(0, 0, 2, 2));

    CPnt ptMean(mean(0), mean(1));
    CEllipse *ell = CreateEllipseFromMeanCov(ptMean, cov, 1.5 * 3);

    // 生成失败，返回false
    if (ell == NULL)
        return false;

    ellipse = *ell;
    delete ell;

    return true;
}

//
//   根据给定的均值和协方差矩阵生成一个椭圆对象。
//
template <typename PointT>
CEllipse *ndt_omp::Leaf<PointT>::CreateEllipseFromMeanCov(const CPnt &ptMean, const Eigen::Matrix2d &cov, double chi2)
{
    Eigen::Vector2d evals;
    Eigen::Matrix2d evecs;

    const double d = cov.determinant();
    if (d <= 0 || d != d)    // Note: "d!=d" is a great test for invalid numbers, don't remove!
        return NULL;
    else
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver;
        eigensolver.compute(cov);
        evals = eigensolver.eigenvalues();
        evecs = eigensolver.eigenvectors();
    }

    // double angle = atan2(evecs(0, 0), evecs(1, 0));

    double angle = atan2(evecs(0, 0), evecs(0, 1)) - PI;

    double halfmajoraxissize = chi2 * evals(1);
    double halfminoraxissize = chi2 * evals(0);

    // 如果半长轴小于半短轴，需要在此进行对换
    if (halfmajoraxissize < halfminoraxissize)
    {
        double temp = halfmajoraxissize;
        halfmajoraxissize = halfminoraxissize;
        halfminoraxissize = temp;
        angle += PI / 2;
    }

    return new CEllipse(ptMean, halfmajoraxissize, halfminoraxissize, angle);
}

// 转换到ndt_oru的NDT单元
template <typename PointT>
void ndt_omp::Leaf<PointT>::ToOruCell(ndt_oru::NDTCell &oruCell)
{
    oruCell.setCenter(center_);
    oruCell.setMean(mean_);
    oruCell.setCov(cov_, true);
    oruCell.setN(nr_points);
    oruCell.setOccupancy(0.5);
    oruCell.hasGaussian_ = true;
    //  oruCell.canUpdate_ = false;
}

#ifdef QT_VERSION
template <typename PointT>
void ndt_omp::Leaf<PointT>::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrLine, QColor clrFill)
{
    CEllipse ell;
    if (!getEllipse(ell))
        return;

    ell.Draw(ScrnRef, pPainter, clrLine, 1, clrFill, true);
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////

//
//   对先前已设置过的输入点云进行处理，生成相关的NDT单元数据，并把所有NDT单元的重心点保存到output中。
//
template <typename PointT>
void ndt_omp::VoxelGridCovariance<PointT>::applyFilter(PointCloud &output)
{
    voxel_centroids_leaf_indices_.clear();

    // 核实输入点云是否已设置过
    if (!input_)
    {
        PCL_WARN("[pcl::%s::applyFilter] No input dataset given!\n", getClassName().c_str());
        output.width = output.height = 0;
        output.points.clear();
        return;
    }

    // Copy the header (and thus the frame_id) + allocate enough space for points
    output.height = 1;         // downsampling breaks the organized structure
    output.is_dense = true;    // we filter out invalid points
    output.points.clear();

    // 下面取得输入点云的坐标(最大/最小)边界值
    Eigen::Vector4f min_p, max_p;

    // 如果只希望处理点云中的一部分
    if (!filter_field_name_.empty())
        pcl::getMinMax3D<PointT>(input_, filter_field_name_, static_cast<float>(filter_limit_min_),
                                 static_cast<float>(filter_limit_max_), min_p, max_p, filter_limit_negative_);
    // 否则，处理整个点云
    else
        pcl::getMinMax3D<PointT>(*input_, min_p, max_p);

    // 根据上面计算得到的点云的范围，判断给定的NDT单元大小是否可用(不会太小)
    int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
    int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
    int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

    // 如果点云所占用的单元数超过了上界，则说明NDT单元大小(leaf size)设置得过小
    if ((dx * dy * dz) > std::numeric_limits<int32_t>::max())
    {
        PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.",
                 getClassName().c_str());
        output.clear();
        return;
    }

    // Compute the minimum and maximum bounding box values
    // 计算NDT图在X/Y/Z方向上的栅格边界值
    min_b_[0] = static_cast<int>(floor(min_p[0] * inverse_leaf_size_[0]));    // X-最小
    max_b_[0] = static_cast<int>(floor(max_p[0] * inverse_leaf_size_[0]));    // X-最大
    min_b_[1] = static_cast<int>(floor(min_p[1] * inverse_leaf_size_[1]));    // Y-最小
    max_b_[1] = static_cast<int>(floor(max_p[1] * inverse_leaf_size_[1]));    // Y-最大
    min_b_[2] = static_cast<int>(floor(min_p[2] * inverse_leaf_size_[2]));    // Z-最小
    max_b_[2] = static_cast<int>(floor(max_p[2] * inverse_leaf_size_[2]));    // Z-最大

    // Compute the number of divisions needed along all axis
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
    div_b_[3] = 0;

    // Clear the leaves
    leaves_.clear();

    // Set up the division multiplier
    divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

    int centroid_size = 4;

    // ---[ RGB special case
    std::vector<pcl::PCLPointField> fields;
    int rgba_index = -1;
    rgba_index = pcl::getFieldIndex(*input_, "rgb", fields);
    if (rgba_index == -1)
        rgba_index = pcl::getFieldIndex(*input_, "rgba", fields);
    if (rgba_index >= 0)
    {
        rgba_index = fields[rgba_index].offset;
        centroid_size += 3;
    }

    // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
    if (!filter_field_name_.empty())
    {
        // Get the distance field index
        std::vector<pcl::PCLPointField> fields;
        int distance_idx = pcl::getFieldIndex(*input_, filter_field_name_, fields);
        if (distance_idx == -1)
            PCL_WARN("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName().c_str(),
                     distance_idx);

        // First pass: go over all points and insert them into the right leaf
        for (size_t cp = 0; cp < input_->points.size(); ++cp)
        {
            if (!input_->is_dense)
                // Check if the point is invalid
                if (!pcl_isfinite(input_->points[cp].x) || !pcl_isfinite(input_->points[cp].y) ||
                    !pcl_isfinite(input_->points[cp].z))
                    continue;

            // Get the distance value
            const uint8_t *pt_data = reinterpret_cast<const uint8_t *>(&input_->points[cp]);
            float distance_value = 0;
            memcpy(&distance_value, pt_data + fields[distance_idx].offset, sizeof(float));

            if (filter_limit_negative_)
            {
                // Use a threshold for cutting out points which inside the interval
                if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
                    continue;
            }
            else
            {
                // Use a threshold for cutting out points which are too close/far away
                if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
                    continue;
            }

            int ijk0 =
                static_cast<int>(floor(input_->points[cp].x * inverse_leaf_size_[0]) - static_cast<float>(min_b_[0]));
            int ijk1 =
                static_cast<int>(floor(input_->points[cp].y * inverse_leaf_size_[1]) - static_cast<float>(min_b_[1]));
            int ijk2 =
                static_cast<int>(floor(input_->points[cp].z * inverse_leaf_size_[2]) - static_cast<float>(min_b_[2]));

            // Compute the centroid leaf index
            int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

            Leaf<PointT> &leaf = leaves_[idx];
            if (leaf.nr_points == 0)
            {
                leaf.centroid.resize(centroid_size);
                leaf.centroid.setZero();
            }

            Eigen::Vector3d pt3d(input_->points[cp].x, input_->points[cp].y, input_->points[cp].z);

            // Accumulate point sum for centroid calculation
            leaf.mean_ += pt3d;

            // Accumulate x*xT for single pass covariance calculation
            leaf.cov_ += pt3d * pt3d.transpose();

            Eigen::Vector4f pt(input_->points[cp].x, input_->points[cp].y, input_->points[cp].z, 0);
            leaf.centroid.template head<4>() += pt;

            //            leaf.points_.push_back(input_->points[cp]);

            ++leaf.nr_points;
        }
    }

    // 没有距离过滤的情况，对所有数据进行处理
    else
    {
        // 第一轮：将输入点云中所有的点插入到正确的NDT单元中
        for (size_t cp = 0; cp < input_->points.size(); ++cp)
        {
            // 如果还没有对无效点进行过滤除处理，则在此滤除那些无效点
            if (!input_->is_dense)
                if (!pcl_isfinite(input_->points[cp].x) || !pcl_isfinite(input_->points[cp].y) ||
                    !pcl_isfinite(input_->points[cp].z))
                    continue;

            // 计算该点所对应的栅格索引值
            int ijk0 =
                static_cast<int>(floor(input_->points[cp].x * inverse_leaf_size_[0]) - static_cast<float>(min_b_[0]));
            int ijk1 =
                static_cast<int>(floor(input_->points[cp].y * inverse_leaf_size_[1]) - static_cast<float>(min_b_[1]));
            int ijk2 =
                static_cast<int>(floor(input_->points[cp].z * inverse_leaf_size_[2]) - static_cast<float>(min_b_[2]));

            // Compute the centroid leaf index
            int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

            // 获取对应的NDT单元的引用
            Leaf<PointT> &leaf = leaves_[idx];
            if (leaf.nr_points == 0)
            {
                leaf.centroid.resize(centroid_size);
                leaf.centroid.setZero();
            }

            Eigen::Vector3d pt3d(input_->points[cp].x, input_->points[cp].y, input_->points[cp].z);

            // 累加点的坐标以便后面计算重心点坐标
            leaf.mean_ += pt3d;

            // 对应于该NDT单元的协方差矩阵的单轮计算(由当前点带来的变化)
            leaf.cov_ += pt3d * pt3d.transpose();

            Eigen::Vector4f pt(input_->points[cp].x, input_->points[cp].y, input_->points[cp].z, 0);
            leaf.centroid.template head<4>() += pt;

            leaf.points_.push_back(input_->points[cp]);

            ++leaf.nr_points;
        }
    }

    // 第二轮处理：依次计算所有NDT单元的重心和协方差矩阵
    output.points.reserve(leaves_.size());

    if (searchable_)
        voxel_centroids_leaf_indices_.reserve(leaves_.size());

    int cp = 0;
    if (save_leaf_layout_)
        leaf_layout_.resize(div_b_[0] * div_b_[1] * div_b_[2], -1);

    // 为防止协方差矩阵奇异化，下面对特征值和特征向量进行调整
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
    Eigen::Matrix3d eigen_val;
    Eigen::Vector3d pt_sum;

    // 对于那些小于最大特征值给定比例门限的特征值，对其进行放大处理
    double min_covar_eigvalue;

    // 依次对各个单元进行处理
    for (auto it = leaves_.begin(); it != leaves_.end(); ++it)
    {
        // 取得对应的NDT单元的引用
        Leaf<PointT> &leaf = it->second;

        // 根据落入单元内的点的数量，对单元的重心点进行规格化处理
        leaf.centroid /= static_cast<float>(leaf.nr_points);

        // Point sum used for single pass covariance calculation
        pt_sum = leaf.mean_;

        // 计算该单元的均值
        leaf.mean_ /= leaf.nr_points;

        // 如果落入单元内的点数超过了规定的门限值，说明这是一个有效NDT单元，可以对其计算协方差矩阵(点数不够的单元不能成为NDT单元)
        if (leaf.nr_points >= min_points_per_voxel_)
        {
            if (save_leaf_layout_)
                leaf_layout_[it->first] = cp++;

            // 将该有效单元的重心点加入到output点云中
            output.push_back(PointT());
            output.points.back().x = leaf.centroid[0];
            output.points.back().y = leaf.centroid[1];
            output.points.back().z = leaf.centroid[2];

            // 保存单元的索引以便将来快速检索
            if (searchable_)
                voxel_centroids_leaf_indices_.push_back(static_cast<int>(it->first));

            // 对该单元的协方差矩阵进行调整
            leaf.cov_ = (leaf.cov_ - 2 * (pt_sum * leaf.mean_.transpose())) / leaf.nr_points +
                        leaf.mean_ * leaf.mean_.transpose();
            leaf.cov_ *= (leaf.nr_points - 1.0) / leaf.nr_points;

            // 用ndt_oru方法计算协方差
            Eigen::MatrixXd mp;
            mp.resize(leaf.points_.size(), 3);

            // 先计算各点相对于中心点的位置差
            for (unsigned int i = 0; i < leaf.points_.size(); i++)
            {
                mp(i, 0) = leaf.points_[i].x - leaf.mean_(0);
                mp(i, 1) = leaf.points_[i].y - leaf.mean_(1);
                mp(i, 2) = leaf.points_[i].z - leaf.mean_(2);
            }

            // 计算协方差矩阵
            Eigen::Matrix3d covSum_ = mp.transpose() * mp;
            leaf.oru_cov_ = covSum_ / (leaf.points_.size() - 1);
            leaf.points_.clear();

            // 计算协方差矩阵的特征值和特征向量
            eigensolver.compute(leaf.cov_);
            eigen_val = eigensolver.eigenvalues().asDiagonal();
            leaf.evecs_ = eigensolver.eigenvectors();

            // 特征值均不得小于0
            if (eigen_val(0, 0) < 0 || eigen_val(1, 1) < 0 || eigen_val(2, 2) <= 0)
            {
                leaf.nr_points = -1;
                continue;
            }

            // 为避免出现奇异矩阵(eq 6.11)[Magnusson 2009]，对特征值矩阵进行调整处理(使得最大/最小之比小于100)
            min_covar_eigvalue = min_covar_eigvalue_mult_ * eigen_val(2, 2);
            if (eigen_val(0, 0) < min_covar_eigvalue)
            {
                eigen_val(0, 0) = min_covar_eigvalue;

                if (eigen_val(1, 1) < min_covar_eigvalue)
                {
                    eigen_val(1, 1) = min_covar_eigvalue;
                }

                leaf.cov_ = leaf.evecs_ * eigen_val * leaf.evecs_.inverse();
            }
            leaf.evals_ = eigen_val.diagonal();

            // 计算逆协方差矩阵
            leaf.icov_ = leaf.cov_.inverse();
            if (leaf.icov_.maxCoeff() == std::numeric_limits<float>::infinity() ||
                leaf.icov_.minCoeff() == -std::numeric_limits<float>::infinity())
            {
                leaf.nr_points = -1;
            }
        }
    }

    output.width = static_cast<uint32_t>(output.points.size());
}

//////////////////////////////////////////////////////////////////////////////////////////

//
//   取得给定点reference_point周围的所有的有效NDT单元(不包括reference_point所处的单元)
//   说明：
//      1. 由relative_coordinates给出需要考查的所有邻居单元
//      2. 只计算周围那些有效(落入点数足够)的NDT单元
//      3. 结果保存在neightbors中，函数返回周围有效的NDT单元数
//
template <typename PointT>
int ndt_omp::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint(const Eigen::MatrixXi &relative_coordinates,
                                                                 const PointT &reference_point,
                                                                 std::vector<LeafConstPtr> &neighbors) const
{
    neighbors.clear();

    // Find displacement coordinates
    // 计算给定点所在单元的位置索引
    Eigen::Vector4i ijk(static_cast<int>(floor(reference_point.x / leaf_size_[0])),
                        static_cast<int>(floor(reference_point.y / leaf_size_[1])),
                        static_cast<int>(floor(reference_point.z / leaf_size_[2])), 0);

    Eigen::Array4i diff2min = min_b_ - ijk;
    Eigen::Array4i diff2max = max_b_ - ijk;

    neighbors.reserve(relative_coordinates.cols());

    // 依次考查每个邻居单元，看它们是否含有足够的点数(由于需要计算9+9+8=26个索引位置，该方法比半径搜索法要慢)
    for (int ni = 0; ni < relative_coordinates.cols(); ni++)
    {
        Eigen::Vector4i displacement = (Eigen::Vector4i() << relative_coordinates.col(ni), 0).finished();

        // Checking if the specified cell is in the grid
        if ((diff2min <= displacement.array()).all() && (diff2max >= displacement.array()).all())
        {
            auto leaf_iter = leaves_.find(((ijk + displacement - min_b_).dot(divb_mul_)));

            // 如果找到了单元，且它含有的点数足够，则把该单元加入到结果数据中
            if (leaf_iter != leaves_.end() && leaf_iter->second.nr_points >= min_points_per_voxel_)
            {
                LeafConstPtr leaf = &(leaf_iter->second);
                neighbors.push_back(leaf);
            }
        }
    }

    return (static_cast<int>(neighbors.size()));
}

//////////////////////////////////////////////////////////////////////////////////////////

//
//   取得给定点reference_point周围的所有的有效NDT单元(不包括reference_point所处的单元)
//   说明：
//      与上一个函数不同，该函数会自动计算所有需要考查的邻居单元，无需由参数给出。
//
template <typename PointT>
int ndt_omp::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint(const PointT &reference_point,
                                                                 std::vector<LeafConstPtr> &neighbors) const
{
    neighbors.clear();

    // 自动计算所有邻居单元的索引坐标
    Eigen::MatrixXi relative_coordinates = pcl::getAllNeighborCellIndices();
    return getNeighborhoodAtPoint(relative_coordinates, reference_point, neighbors);
}

//////////////////////////////////////////////////////////////////////////////////////////

//
//   只计算X-Y平面周围7个邻居单元，取得给定点reference_point周围的所有的有效NDT单元(不包括reference_point所处的单元)
//
template <typename PointT>
int ndt_omp::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint7(const PointT &reference_point,
                                                                  std::vector<LeafConstPtr> &neighbors) const
{
    neighbors.clear();

    Eigen::MatrixXi relative_coordinates(3, 7);
    relative_coordinates.setZero();
    relative_coordinates(0, 1) = 1;
    relative_coordinates(0, 2) = -1;
    relative_coordinates(1, 3) = 1;
    relative_coordinates(1, 4) = -1;
    relative_coordinates(2, 5) = 1;
    relative_coordinates(2, 6) = -1;

    return getNeighborhoodAtPoint(relative_coordinates, reference_point, neighbors);
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
int ndt_omp::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint1(const PointT &reference_point,
                                                                  std::vector<LeafConstPtr> &neighbors) const
{
    neighbors.clear();
    return getNeighborhoodAtPoint(Eigen::MatrixXi::Zero(3, 1), reference_point, neighbors);
}

// 转换为ndt_oru类型的NDT图
template <typename PointT>
void ndt_omp::VoxelGridCovariance<PointT>::ToOruMap(ndt_oru::NDTMap &oruMap)
{
    oruMap.Clear();

    // 设置NDT单元大小
    Eigen::Vector3f leafSize = pcl::VoxelGrid<PointT>::getLeafSize();
    oruMap.setCellSize(leafSize[0], leafSize[1], leafSize[2]);

    // 为NDT图分配空间
    Eigen::Vector3d center(0, 0, 0);
    if (!oruMap.initialize(center, 500, 500, leafSize[2]))
        return;

    oruMap.setCellType(new ndt_oru::NDTCell(leafSize[0], leafSize[1], leafSize[2]));

    // 依次加入各单元
    for (auto it = leaves_.begin(); it != leaves_.end(); ++it)
    {
        // 取得对应的NDT单元的引用
        Leaf<PointT> &leaf = it->second;
        if (leaf.nr_points < min_points_per_voxel_)
            continue;

        ndt_oru::NDTCell *oruCell = oruMap.getCellForPoint(leaf.mean_, true);
        if (oruCell == NULL)
            continue;

        leaf.center_ = oruCell->getCenter();
        leaf.ToOruCell(*oruCell);
    }
}

#ifdef QT_VERSION
template <typename PointT>
void ndt_omp::VoxelGridCovariance<PointT>::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrLine,
                                                QColor clrFill)
{
    for (auto it = leaves_.begin(); it != leaves_.end(); ++it)
    {

        // 取得对应的NDT单元的引用
        Leaf<PointT> &leaf = it->second;
        if (leaf.nr_points < min_points_per_voxel_)
            continue;

        leaf.Plot(ScrnRef, pPainter, clrLine, clrFill);
    }
}
#endif

#define PCL_INSTANTIATE_VoxelGridCovariance(T) template class PCL_EXPORTS pcl::VoxelGridCovariance<T>;
