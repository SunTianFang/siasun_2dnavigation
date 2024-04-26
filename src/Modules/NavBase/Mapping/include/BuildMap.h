#pragma once

#include "LocalizationManager.h"
#include "ndt_maps_oru.h"
#include "LaserAutoMapping.h"
#include "NdtMethod.h"
///////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////
//   实现由优化后的点云生成ndt地图、概率栅格、和分支限界地图功能
//   Author: lishen
//   Date:   2022. 6.
///////////////////////////////////////////////


namespace mapping {


class  CBuildMap : public CLocalizationManager
{
  private:


    ndt_oru::CNdtExtLocalization *pLocation;

     CScannerGroupParam m_scannerParam;

  public:
    CBuildMap();
   // ~CBuildMap();

    virtual bool Create();

    // 定位过程函数
    //virtual CMatchInfo *Localize(Eigen::Affine3d &estimatePose);

    bool BuildNdtMap( map<int,mapping::CStepData> *pStepData);

    bool SetNdtSubmapParam(ndt_oru::CSubmapParam *param);

    bool TransformScanToCloud(const CScan &scan,
                                            ndt_oru::CStampedPointCloud* cloud_trans,bool bAddFarPoint);
    void SetScansParam(CScannerGroupParam *param);

    bool SaveNdtMap();

    int  SaveProbGridMap(const string filename,map<int,mapping::CStepData> *pStepData,CPosture &ptLeftBottom,CPosture &ptRightTop,int frozenNodeNum);

};
}
