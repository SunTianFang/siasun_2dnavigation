#pragma once

#include <pcl/filters/boost.h>
#include <pcl/filters/voxel_grid.h>
#include <map>
#include <unordered_map>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Eigen>
#include "ndt_map_oru.h"

#include <iostream>
#include "Ellipse.h"
#include "ScrnRef.h"

#ifdef QT_VERSION
#include <QColor>
#include <QPainter>
#endif

namespace ndt_omp
{

// 用于存储NDT单元的重心、协方差和点数的数据结构(逆协方差和特征值、特征向量也要进行预先计算)
template <typename PointT>
class Leaf
{
  public:
    Leaf()
        : nr_points(0),
          mean_(Eigen::Vector3d::Zero()),
          centroid(),
          cov_(Eigen::Matrix3d::Identity()),
          icov_(Eigen::Matrix3d::Zero()),
          evecs_(Eigen::Matrix3d::Identity()),
          evals_(Eigen::Vector3d::Zero())
    {
    }

    // 取得协方差矩阵
    Eigen::Matrix3d getCov() const { return (cov_); }

    // 取得协方差矩阵的逆矩阵
    Eigen::Matrix3d getInverseCov() const { return (icov_); }

    // 取得单元的均值(即重心点)
    Eigen::Vector3d getMean() const { return (mean_); }

    // 取得协方差矩阵的特征向量(顺序与getEvals()相同)
    Eigen::Matrix3d getEvecs() const { return (evecs_); }

    // 取得协方差矩阵的特征值(顺序与getEvecs()相同)
    Eigen::Vector3d getEvals() const { return (evals_); }

    // 取得单元中点的数量
    int getPointCount() const { return (nr_points); }

    // 取得对应于本单元的概率圆
    bool getEllipse(CEllipse &ellipse);

    // 根据给定的均值和协方差矩阵生成一个椭圆对象
    CEllipse *CreateEllipseFromMeanCov(const CPnt &ptMean, const Eigen::Matrix2d &cov, double chi2);

    // 转换到ndt_oru的NDT单元
    void ToOruCell(ndt_oru::NDTCell &oruCell);

#ifdef QT_VERSION
    void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrLine, QColor clrFill);
#endif

  public:
    int nr_points;               // NDT单元内落入的点数
    Eigen::Vector3d center_;     // 该单元的中心点
    Eigen::Vector3d mean_;       // 单元的重心点
    Eigen::VectorXf centroid;    // 单元的重心点(当有颜色信息加入时才与mean_不同)
    Eigen::Matrix3d cov_;        // 协方差矩阵
    Eigen::Matrix3d icov_;       // 逆协方差矩阵
    Eigen::Matrix3d evecs_;      // 协方差矩阵的特征向量
    Eigen::Vector3d evals_;      // 协方差矩阵的特征值

    Eigen::Matrix3d oru_cov_;
    std::vector<PointT> points_;
};

/** \brief A searchable voxel strucure containing the mean and covariance of the data.
    * \note For more information please see
    * <b>Magnusson, M. (2009). The Three-Dimensional Normal-Distributions Transform —
    * an Efﬁcient Representation for Registration, Surface Analysis, and Loop Detection.
    * PhD thesis, Orebro University. Orebro Studies in Technology 36</b>
    * \author Brian Okorn (Space and Naval Warfare Systems Center Pacific)
    */
template <typename PointT>
class VoxelGridCovariance : public pcl::VoxelGrid<PointT>
{
  protected:
    using pcl::VoxelGrid<PointT>::filter_name_;
    using pcl::VoxelGrid<PointT>::getClassName;
    using pcl::VoxelGrid<PointT>::input_;
    using pcl::VoxelGrid<PointT>::indices_;
    using pcl::VoxelGrid<PointT>::filter_limit_negative_;
    using pcl::VoxelGrid<PointT>::filter_limit_min_;
    using pcl::VoxelGrid<PointT>::filter_limit_max_;
    using pcl::VoxelGrid<PointT>::filter_field_name_;

    using pcl::VoxelGrid<PointT>::leaf_layout_;
    using pcl::VoxelGrid<PointT>::save_leaf_layout_;
    using pcl::VoxelGrid<PointT>::leaf_size_;
    using pcl::VoxelGrid<PointT>::min_b_;
    using pcl::VoxelGrid<PointT>::max_b_;
    using pcl::VoxelGrid<PointT>::inverse_leaf_size_;
    using pcl::VoxelGrid<PointT>::div_b_;
    using pcl::VoxelGrid<PointT>::divb_mul_;

    typedef typename pcl::traits::fieldList<PointT>::type FieldList;
    typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  public:
    typedef boost::shared_ptr<pcl::VoxelGrid<PointT>> Ptr;
    typedef boost::shared_ptr<const pcl::VoxelGrid<PointT>> ConstPtr;

    // 定义类型LeafPtr-指向NDT单元的指针
    typedef Leaf<PointT> *LeafPtr;

    // 定义类型LeafConstPtr-指向NDT单元的常数指针
    typedef const Leaf<PointT> *LeafConstPtr;

    // 定义类型Map-由索引号到NDT单元的映射
    typedef std::map<size_t, Leaf<PointT>> Map;

  public:
    VoxelGridCovariance()
        : searchable_(true),
          min_points_per_voxel_(6),
          min_covar_eigvalue_mult_(0.01),
          leaves_(),
          voxel_centroids_(),
          voxel_centroids_leaf_indices_(),
          kdtree_()
    {
        save_leaf_layout_ = false;
        leaf_size_.setZero();
        min_b_.setZero();
        max_b_.setZero();
        filter_name_ = "VoxelGridCovariance";
    }

    // 设置有效NDT单元所应含有的最少点数(不应小于3)
    inline void setMinPointPerVoxel(int min_points_per_voxel)
    {
        if (min_points_per_voxel > 2)
        {
            min_points_per_voxel_ = min_points_per_voxel;
        }
        else
        {
            PCL_WARN("%s: Covariance calculation requires at least 3 points, setting Min Point per Voxel to 3 ",
                     this->getClassName().c_str());
            min_points_per_voxel_ = 3;
        }
    }

    // 取得有效NDT单元所应含有的最少点数
    inline int getMinPointPerVoxel() { return min_points_per_voxel_; }

    // 设置协方差矩阵特征值的“膨胀因子”(即最小特征值与最大特征值的最小允许比值)
    inline void setCovEigValueInflationRatio(double min_covar_eigvalue_mult)
    {
        min_covar_eigvalue_mult_ = min_covar_eigvalue_mult;
    }

    // 取得协方差矩阵特征值的“膨胀因子”(即最小特征值与最大特征值的最小允许比值)
    inline double getCovEigValueInflationRatio() { return min_covar_eigvalue_mult_; }

    // 对给定点云进行处理，生成相关的NDT图，并将生成的NDT图中所有有效单元的重心点存储到output中
    // 如果searchable为true, 则建立KD树以便进行快速搜索
    inline void filter(PointCloud &output, bool searchable = false)
    {
        searchable_ = searchable;
        applyFilter(output);

        voxel_centroids_ = PointCloudPtr(new PointCloud(output));

        if (searchable_ && voxel_centroids_->size() > 0)
        {
            // Initiates kdtree of the centroids of voxels containing a sufficient number of points
            kdtree_.setInputCloud(voxel_centroids_);
        }
    }

    // 初始化NDT图结构
    // 如果searchable为true, 则建立KD树以便进行快速搜索
    inline void filter(bool searchable = false)
    {
        searchable_ = searchable;
        voxel_centroids_ = PointCloudPtr(new PointCloud);
        applyFilter(*voxel_centroids_);

        if (searchable_ && voxel_centroids_->size() > 0)
        {
            // Initiates kdtree of the centroids of voxels containing a sufficient number of points
            kdtree_.setInputCloud(voxel_centroids_);
        }
    }

    // 根据给定的索引号，取得对应的NDT单元的指针
    inline LeafConstPtr getLeaf(int index)
    {
        auto leaf_iter = leaves_.find(index);
        if (leaf_iter != leaves_.end())
        {
            LeafConstPtr ret(&(leaf_iter->second));
            return ret;
        }
        else
            return NULL;
    }

    // 根据给定的点，取得对应的NDT单元的指针
    inline LeafConstPtr getLeaf(PointT &p)
    {
        // 计算关于p点的X/Y/Z单元索引
        int ijk0 = static_cast<int>(floor(p.x * inverse_leaf_size_[0]) - min_b_[0]);
        int ijk1 = static_cast<int>(floor(p.y * inverse_leaf_size_[1]) - min_b_[1]);
        int ijk2 = static_cast<int>(floor(p.z * inverse_leaf_size_[2]) - min_b_[2]);

        // 计算相关的NDT单元序号
        int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

        // 根据映射关系找到对应的单元
        auto leaf_iter = leaves_.find(idx);
        if (leaf_iter != leaves_.end())
        {
            // 找到了，返回指向该NDT单元的指针
            LeafConstPtr ret(&(leaf_iter->second));
            return ret;
        }
        else
            return NULL;    // 没找到，返回NULL
    }

    // 根据给定的点(以Eigen向量形式提供)，取得对应的NDT单元的指针
    inline LeafConstPtr getLeaf(Eigen::Vector3f &p)
    {
        // 计算关于p点的X/Y/Z单元索引
        int ijk0 = static_cast<int>(floor(p[0] * inverse_leaf_size_[0]) - min_b_[0]);
        int ijk1 = static_cast<int>(floor(p[1] * inverse_leaf_size_[1]) - min_b_[1]);
        int ijk2 = static_cast<int>(floor(p[2] * inverse_leaf_size_[2]) - min_b_[2]);

        // 计算相关的NDT单元序号
        int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

        // 根据映射关系找到对应的单元
        auto leaf_iter = leaves_.find(idx);
        if (leaf_iter != leaves_.end())
        {
            // 找到了，返回指向该NDT单元的指针
            LeafConstPtr ret(&(leaf_iter->second));
            return ret;
        }
        else
            return NULL;    // 没找到，返回NULL
    }

    // 取得给定点reference_point周围的所有的有效NDT单元(不包括reference_point所处的单元)
    // 说明：只计算周围那些有效(落入点数足够)的NDT单元，结果保存在neightbors中，返回周围有效的NDT单元数。
    int getNeighborhoodAtPoint(const Eigen::MatrixXi &, const PointT &reference_point,
                               std::vector<LeafConstPtr> &neighbors) const;

    // 取得给定点reference_point周围的所有的有效NDT单元(不包括reference_point所处的单元)
    // 说明：与上一个函数不同，该函数会自动计算所有需要考查的邻居单元，无需由参数给出。
    int getNeighborhoodAtPoint(const PointT &reference_point, std::vector<LeafConstPtr> &neighbors) const;

    // 只计算X-Y平面周围7个邻居单元，取得给定点reference_point周围的所有的有效NDT单元(不包括reference_point所处的单元)
    int getNeighborhoodAtPoint7(const PointT &reference_point, std::vector<LeafConstPtr> &neighbors) const;

    int getNeighborhoodAtPoint1(const PointT &reference_point, std::vector<LeafConstPtr> &neighbors) const;

    // 取得NDT图的映射
    inline const Map &getLeaves() { return leaves_; }

    // 取得对应于所有有效NDT单元重心点的点云
    inline PointCloudPtr getCentroids() { return voxel_centroids_; }

    /** \brief Search for the k-nearest occupied voxels for the given query point.
       * \note Only voxels containing a sufficient number of points are used.
       * \param[in] point the given query point
       * \param[in] k the number of neighbors to search for
       * \param[out] k_leaves the resultant leaves of the neighboring points
       * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
       * \return number of neighbors found
       */
    int nearestKSearch(const PointT &point, int k, std::vector<LeafConstPtr> &k_leaves,
                       std::vector<float> &k_sqr_distances)
    {
        k_leaves.clear();

        // Check if kdtree has been built
        if (!searchable_)
        {
            PCL_WARN("%s: Not Searchable", this->getClassName().c_str());
            return 0;
        }

        // Find k-nearest neighbors in the occupied voxel centroid cloud
        std::vector<int> k_indices;
        k = kdtree_.nearestKSearch(point, k, k_indices, k_sqr_distances);

        // Find leaves corresponding to neighbors
        k_leaves.reserve(k);
        for (std::vector<int>::iterator iter = k_indices.begin(); iter != k_indices.end(); iter++)
        {
            k_leaves.push_back(&leaves_[voxel_centroids_leaf_indices_[*iter]]);
        }
        return k;
    }

    /** \brief Search for the k-nearest occupied voxels for the given query point.
       * \note Only voxels containing a sufficient number of points are used.
       * \param[in] cloud the given query point
       * \param[in] index the index
       * \param[in] k the number of neighbors to search for
       * \param[out] k_leaves the resultant leaves of the neighboring points
       * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
       * \return number of neighbors found
       */
    inline int nearestKSearch(const PointCloud &cloud, int index, int k, std::vector<LeafConstPtr> &k_leaves,
                              std::vector<float> &k_sqr_distances)
    {
        if (index >= static_cast<int>(cloud.points.size()) || index < 0)
            return (0);
        return (nearestKSearch(cloud.points[index], k, k_leaves, k_sqr_distances));
    }

    /** \brief Search for all the nearest occupied voxels of the query point in a given radius.
       * \note Only voxels containing a sufficient number of points are used.
       * \param[in] point the given query point
       * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
       * \param[out] k_leaves the resultant leaves of the neighboring points
       * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
       * \param[in] max_nn
       * \return number of neighbors found
       */
    int radiusSearch(const PointT &point, double radius, std::vector<LeafConstPtr> &k_leaves,
                     std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const
    {
        k_leaves.clear();

        // Check if kdtree has been built
        if (!searchable_)
        {
            PCL_WARN("%s: Not Searchable", this->getClassName().c_str());
            return 0;
        }

        // Find neighbors within radius in the occupied voxel centroid cloud
        std::vector<int> k_indices;
        int k = kdtree_.radiusSearch(point, radius, k_indices, k_sqr_distances, max_nn);

        // Find leaves corresponding to neighbors
        k_leaves.reserve(k);
        for (std::vector<int>::iterator iter = k_indices.begin(); iter != k_indices.end(); iter++)
        {
            auto leaf = leaves_.find(voxel_centroids_leaf_indices_[*iter]);
            if (leaf == leaves_.end())
            {
                std::cerr << "error : could not find the leaf corresponding to the voxel" << std::endl;
                std::cin.ignore(1);
            }
            k_leaves.push_back(&(leaf->second));
        }
        return k;
    }

    /** \brief Search for all the nearest occupied voxels of the query point in a given radius.
       * \note Only voxels containing a sufficient number of points are used.
       * \param[in] cloud the given query point
       * \param[in] index a valid index in cloud representing a valid (i.e., finite) query point
       * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
       * \param[out] k_leaves the resultant leaves of the neighboring points
       * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
       * \param[in] max_nn
       * \return number of neighbors found
       */
    inline int radiusSearch(const PointCloud &cloud, int index, double radius, std::vector<LeafConstPtr> &k_leaves,
                            std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const
    {
        if (index >= static_cast<int>(cloud.points.size()) || index < 0)
            return (0);
        return (radiusSearch(cloud.points[index], radius, k_leaves, k_sqr_distances, max_nn));
    }

    // 转换为ndt_oru类型的NDT图
    void ToOruMap(ndt_oru::NDTMap &oruMap);

#ifdef QT_VERSION
    void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrLine, QColor clrFill);
#endif

  protected:
    // 对先前已设置过的输入点云进行处理，生成相关的NDT单元数据，并把所有NDT单元的重心点保存到output中
    void applyFilter(PointCloud &output);

  protected:
    bool searchable_;                                  // 标志此单元是否可搜索
    int min_points_per_voxel_;                         // 每个可用的NDT单元内所应含有的最少的点数
    double min_covar_eigvalue_mult_;                   // 协方差矩阵特征值之间所应具有的最小比值
    Map leaves_;                                       // 存储所有单元(包括那些点数不够的单元)的结构
    PointCloudPtr voxel_centroids_;                    // 含有所有有效NDT单元重心点的点云
    std::vector<int> voxel_centroids_leaf_indices_;    // 可由voxel_centroids_点找到对应NDT单元的索引向量
    pcl::KdTreeFLANN<PointT> kdtree_;                  // 用voxel_centroids_建立起来的KD树
};
}    // namespace ndt_omp
