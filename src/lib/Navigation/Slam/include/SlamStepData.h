#ifndef __CSlamStepData
#define __CSlamStepData

#include <vector>
#include "Geometry.h"
#include "Scan.h"
#include "PointFeatureSet.h"
#include "Frame.h"
#include "StampedPosture.h"
#include "CsmScan.h"
#include "CsmMatcher.h"
#include "LiveObjects.h"

#ifdef QT_VERSION
#include <QColor>
#endif

using namespace std;

typedef vector<CScan> CVectScan;

///////////////////////////////////////////////////////////////////////////////
//   关于每个SLAM步的数据及相关处理类。
class CSlamStepData
{
  public:
    CStampedPosture m_pstMoveEst;       // 机器人的估测姿态变化量
    CStampedPosture m_pst;              // 由数据集记录的机器人的绝对姿态(仅供参考)
    CPosture m_pstRobot;                // 机器人的绝对姿态(实际计算使用)
    CPosture m_vel;                     // 速度向量，借用CPosture结构来表示
    CVectScan m_scanLocal;              // 相对于扫描姿态的局部点云
    CVectScan m_scanGlobal;             // 全局激光扫描数据
    bool m_bCorrOk;                     // 点云匹配是否成功
    CCorrList m_corrList;               // 散点匹配表
    sm_result m_smResult;               // 散点匹配结果
    CFrame m_frmToLastStep;             // 到上一步的匹配变换
    bool m_bMatchLast;


     //lishen
     bool m_display;
     bool m_bCartoNode;
     CPosture m_pstRobotBeforeOpt;

  public:
    CSlamStepData() { m_display = true; m_bCartoNode = false;Clear(); }

    // 拷贝构造函数
    CSlamStepData(const CSlamStepData &Obj);

    // 从扫描点云数据生成
    bool CreateFromScan(const CScan &Scan);

    // 二进制文件读取原始扫描点数据
    bool LoadRawScanBinary(FILE *fp, const CScannerGroupParam &ScannerParam, int nFileVersion, bool bFirstStep = false);

    // 将原始扫描点数据写入二进制文件
    bool SaveRawScanBinary(FILE *fp, int nFileVersion, bool bSaveGlobalPosture = false);

    // 清除所有数据
    void Clear();

#if 1
    // 在给定的矩形区域中收集本步数据内的所有高亮度点
    void CollectHighIntensPoints(const CRectangle &r, CPointFeatureSet &highIntensPoints, int nScannerId = -1);
#endif

#if 1
     // Jzz: 在给定的矩形区域中收集本步数据内的所有点
    void CollectInRectPoints(const CRectangle &r, CPointFeatureSet &inRectPoints, int nScannerId = -1);
#endif

    // 设置步内的各个扫描点云是否在屏幕上可见
    void SetScansVisible(bool *pVisible);

    // 根据给定的原始坐标系，将数据转换到世界坐标系下
    void CreateGlobalData(CPosture &pstInit, const CScannerGroupParam &ScannerGroupParam);

    // 将本步数据与给定的步(一般是上一步)进行点云匹配
    bool PointCloudMatch(const CSlamStepData &another, CCorrList *pList = NULL, sm_result *pResult = NULL);

    // 取得指定的世界散点
    CScanPoint *GetWorldRawPoint(int nScannerId, int nIdxPoint);

    // 将本步的数据进行指定的坐标变换
    void Transform(const CFrame &frame);

    // 将本步的数据进行指定的坐标逆变换
    void InvTransform(const CFrame &frame);

    // 将全部点绕激光器中心点进行旋转
    void Rotate(float angle);

    // 将全部的点进行平动
    void Move(float dx, float dy);

    // 启用关于扫描角度范围的约束
    void ApplyScanAngleRule(int nScannerId, float fMinAngle, float fMaxAngle);

    // 启用关于扫描距离的约束
    void ApplyScanDistRule(int nScannerId, float fMinDist, float fMaxDist);

    // 判断指定的屏幕点是否触本步数据中的某个原始点。
    int PointHitRawPoint(const CPnt &pt, float fDistGate);

    // 判断指定的屏幕点是否触本步数据中的机器人位姿
    int PointHitPose(const CPnt &pt, float fDistGate);

    // 对本步中的指定传感器扫描数据进行扫描匹配
    bool MatchInternalScans(int nScanId1, int nScanId2, CPosture &result);

    // 将本步与另一个给定的数据步进行匹配
    bool Match(const CSlamStepData &another, sm_result &result);

    // 将本步坐标变换到与给定的数据步相匹配
    bool AlignTo(const CSlamStepData &another, CPosture *ppstMove = NULL);

    // 将本步坐标变换，使其激光位姿等于给定的位姿
    bool TransformToRobotPose(const CPosture& pstScanner);

    // 施加虚拟环境
    void ApplyVirtObjects(const CLiveObjects &objs, int stepId);

#ifdef _MFC_VER
    // 在屏幕上显示
    virtual void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF clrRawPoint, COLORREF clrHightLightRawPoint, bool bShowPose = false,
                      COLORREF clrPose = 0, bool bShowFeature = false, COLORREF clrFeature = 0);

#elif defined QT_VERSION
    // 在屏幕上显示
    virtual void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrRawPoint, QColor clrHightLightRawPoint, bool bShowPose = false,
                      QColor clrPose = QColor(255, 0, 255), bool bShowFeature = false, QColor clrFeature = Qt::black);

    // 显示该步中的高亮点
    void PlotHighIntensPoints(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr);
#endif
};
#endif
