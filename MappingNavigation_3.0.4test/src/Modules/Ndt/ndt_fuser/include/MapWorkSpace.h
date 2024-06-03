#ifndef __CMapWorkSpace
#define __CMapWorkSpace

#include "SlamDataSet.h"
#include "ndt_map_oru.h"
#include "VectPose.h"


class CMapWorkSpace
{
public:
	CSlamDataSet  m_dataset;                // 数据集
	perception_oru::NDTMap m_ndtMap;		    // NDT图
	CVectPose m_vectPose;                   // 姿态记录

public:
	CMapWorkSpace() {}

	bool Load(FILE* fp);
	bool Save(FILE* fp, int nFileVersion = 251);

#ifdef _MFC_VER

	// 绘制模型图。
	void PlotMap(CDC* pDC, CScreenReference& ScrnRef, bool bShowPoses = false);
#endif

};
#endif
