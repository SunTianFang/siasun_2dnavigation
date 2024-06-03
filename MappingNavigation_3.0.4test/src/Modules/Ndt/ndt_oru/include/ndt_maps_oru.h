#pragma once

#include "ndt_map_oru.h"

#ifdef NDT1_USE_MINI_EIGEN
#define Eigen MiniEigen
#endif

namespace ndt_oru
{

///////////////////////////////////////////////////////////////////////////////
//   定义可以存储多个子图的NDT图。
//   注：除了正常的子图外，该类中还包含有一个临时子图，用来适应在动态环境下的SLAM。
//
class DllExport NDTMaps : public std::vector<NDTMap *>
{
#ifndef NDT1_USE_MINI_EIGEN
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

  protected:
    NDTMap tempSubmap_;    // 临时子图，用来处理动态环境下的SLAM

  protected:
    virtual NDTMap *CreateSubmap() { return new NDTMap; }

  public:
    NDTMaps() {Clear();}
    virtual ~NDTMaps();

    // 清除所有子图
    void Clear();

    // By Sam: 取得选中子图的数量。
    int CountSelectedSubmaps();

    // By Sam: 删除所有选中的子图。
    void DeleteAllSelected();

    //By yu. reurn Selected_Map_id
    void GetAllSelected(std::vector<int> &Index);

    // 清除所有子图的应用区域
    void ClearAppArea();

    // 向指定的子图中增加一个矩形应用区域

    // 生成一个新的NDT子图
    virtual NDTMap *AddSubmap(const Eigen::Affine3d &homePose, const CSubmapParam &submapConfig);

    // 根据给定的NDT图生成一个新的NDT子图
    NDTMap *AddSubmap(const NDTMap *map);

    // 重载操作符"+="
    NDTMaps &operator+=(const NDTMap &another);

    // 重载操作符"+="
    NDTMaps &operator+=(const NDTMaps &another);

    // 清除临时子图
    void ClearTempSubmap();

    // 对全图进行坐标变换
    void Transform(Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr);

    // 从文件中装入NDT图
    virtual bool LoadBinary(FILE *fp);

    // 将NDT图保存到文件
    virtual bool SaveBinary(FILE *fp);
};
}    // namespace ndt_oru

#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif
