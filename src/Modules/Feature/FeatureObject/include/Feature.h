#ifndef __CFeature
#define __CFeature

#include "Geometry.h"
#include "ScrnRef.h"

// 聚类后未提特征的点集合,(中间产物)
#define FEATURE_UNCLASSIFIED_NORM	-2
#define FEATURE_UNCLASSIFIED_HIGH	-1
#define FEATURE_UNCLASSIFIED_RECT   -3   //by zjc 矩形特征聚类，后续还会根据需求添加
// 特征类型
#define GENERIC_POINT_FEATURE      0        // 一般特征
#define FLAT_REFLECTOR_FEATURE     1        // 平面反光板特征
#define CYLINDER_FEATURE           2        // 圆柱特征
#define CORNER_FEATURE             3        // 角点
#define SHORT_LINE_FEATURE         4        // 短直线特征
#define EDGE_FEATURE               5        // 边缘特征
#define IRREGULAR_PILLAR_FEATURE   6
#define RECT_FEATURE               7

#define FEATURE_DIR_FRONT_SIDE_ONLY           1        // 只使用直线正面
#define FEATURE_DIR_BACK_SIDE_ONLY            2        // 只使用直线反面
#define FEATURE_DIR_BOTH_SIDES                3        // 使用直线正反两面

#define SIASUN_MATCHER_ANGLE_WINDOW           (5*PI/180)   // +/-5度开放角
#define SIASUN_MATCHER_DIST_WINDOW            0.3//(300)         // +/-300mm距离
#define SIASUN_MATCHER_SHORTDIST_WINDOW       0.2//(200)

/******************点云分割需要的参数***********************/
// 点距离范围阈值(单位m)
#define POINTS_DISTANCE_THRESH_NEAR     0.2//200
#define POINTS_DISTANCE_THRESH_FAR      35//35000
// 区分反光板的强度阈值,线性插值
#define HIGH_INTENSITY_THRESH_NEAR      2540
#define HIGH_INTENSITY_THRESH_FAR       2540
#define INTENSITY_DIV(DIST) 2540        //by zjc 由于已实现各激光头标准化处理，因此强度门限为固定值
/*HIGH_INTENSITY_THRESH_NEAR + DIST*							\
                                                        (HIGH_INTENSITY_THRESH_NEAR - HIGH_INTENSITY_THRESH_FAR)/  \
                                                        (POINTS_DISTANCE_THRESH_NEAR - POINTS_DISTANCE_THRESH_FAR)*/
// 点云截断阈值(单位m),线性插值
#define POINTS_CONNECTION_THRESH_NEAR   0.05//50
#define POINTS_CONNECTION_THRESH_FAR    0.1//100
#define CONNECTION_DIV(DIST) POINTS_CONNECTION_THRESH_NEAR + DIST*							\
                                                        (POINTS_CONNECTION_THRESH_NEAR - POINTS_CONNECTION_THRESH_FAR)/  \
                                                        (POINTS_DISTANCE_THRESH_NEAR - POINTS_DISTANCE_THRESH_FAR)
/***********************************************************/
//////////////////////////////////////////////////////////////////////////////
//   定义“特征”基类。
class DllExport CFeature
{
public:
	int      m_nType;               // 特征的类型编号
	int      m_nID;                 // 此特征的ID号

public:
	CFeature() 
	{
		m_nType = 0;
		m_nID = 0;
	}

    // 设置特征的类型
    void SetType(int nType) { m_nType = nType; }

    // 取得特征的类型
    int GetType() const { return m_nType; }

    virtual bool LoadText(FILE* fp) { return true; }
    virtual bool SaveText(FILE* fp) { return true; }
    virtual bool LoadBinary(FILE* fp) { return true; }
    virtual bool SaveBinary(FILE* fp) { return true; }
};
#endif
