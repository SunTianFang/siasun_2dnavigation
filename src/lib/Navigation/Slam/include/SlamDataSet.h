#ifndef __CSlamDataSet
#define __CSlamDataSet

#include "ndt_options.h"
#include "SlamStepData.h"
#include "LiveObjects.h"


#define CERES_OPTIMIZATION

#ifdef CERES_OPTIMIZATION
#include "OptimizationConditions.h"
#endif

#ifdef NDT1_USE_MINI_EIGEN
#include "ME_Transform.h"
#define Eigen MiniEigen
#endif


#include "navigation.h"
#include "probability_grid.h"

// 对扫描数据进行滤波处理的规则
class CScanFilterRule
{
  public:
    int m_nScannerId;     // 激光器编号
    int m_nType;          // 0-无规则; 1-角度规则; 2-距离规则
    int m_nStartId;       // 开始启用的ID
    int m_nEndId;         // 结束启用的ID
    float m_fParam[2];    // 规则参数

  public:
    CScanFilterRule()
    {
        m_nScannerId = 0;
        m_nType = 0;
    }

    // 角度规则
    CScanFilterRule(int nScannerId, int nType, int nStartId, int nEndId, float fParam1, float fParam2)
    {
        m_nScannerId = nScannerId;
        m_nType = nType;
        m_nStartId = nStartId;
        m_nEndId = nEndId;
        m_fParam[0] = fParam1;
        m_fParam[1] = fParam2;
    }
};

typedef vector<CScanFilterRule> CScanFilterRules;

///////////////////////////////////////////////////////////////////////////////
class CSlamDataSet : public vector<CSlamStepData>
{
  private:
    CPosture m_pstCur;
    int m_nFileVersion;           // 数据文件的版本号
    int m_nScannerCount;          // 激光扫描器的数量
    unsigned int m_uStartTime;    // 数据集的起始时间
    CCorrList m_corrList;         // 用于存储帧间比较匹配数据的工作空间
    sm_result m_result;           // 数据步匹配结果数据
    int m_nCorrCurStep;           // 匹配表的当前步
    int m_nCorrRefStep;           // 匹配表的参考步

    CCartoSlam *m_pCartoSlam;     //


    bool    m_bExpandMap;

    bool    m_isCartoSlaming;

    int lastCartoOptNodeId;
    map<int,CScan*> m_nodeScans;

  public:
    CSlamStepData m_TransformedStep;
    CScannerGroupParam m_ScannerParam;      // 扫描器参数
    CPointFeatureSet m_reflectors;          // 反光板集合

#ifdef CERES_OPTIMIZATION
    COptimizationConditions m_Conditions;         // 优化条件
    vector<CConstraint> m_DetectedConstraints;    // 保存检测到的约束关系的向量
#endif

    int m_nStepCache;    // 当前对应于约束关系的步数

  public:
    CSlamDataSet()
    {
        m_nFileVersion = 1;
        m_nScannerCount = 1;
        m_uStartTime = 0;
        m_nCorrCurStep = -1;    // 匹配表的当前步
        m_nCorrRefStep = -1;    // 匹配表的参考步
        m_nStepCache = -1;
        m_reflectors.EnableDistanceCache(false);

        /////////////////////add by lishen
        m_pCartoSlam = nullptr;
        m_isCartoSlaming = false;
        m_bExpandMap = false;

    }
    ~CSlamDataSet(){
        if(m_pCartoSlam!=nullptr)
            m_pCartoSlam->DesInstance();
    }

    // 清除数据集
    void Clear();

    // 设置步内的各个扫描点云是否在屏幕上可见
    void UpdateScansVisibility();

    // 判断数据集是否为空
    bool IsEmpty() { return size() == 0; }

    // 装入二进制原始扫描点数据文件
    int LoadRawScanBinary(FILE *fp);

    // 将原始扫描点数据写入二进制文件
    bool SaveRawScanBinary(FILE *fp, int nFileVersion, int nFrom = 0, int nTo = -1, bool bReverseOrder = false,
                           bool bSaveGlobalPosture = true);

    // 启用新的激光器参数
    bool SetScannerParam(const CScannerGroupParam &Param);

    // 取得指定的世界散点
    CScanPoint *GetWorldRawPoint(int nStepId, int nScannerId, int nIdxPoint);

    // 更新数据集内的高亮度点集合
    void CollectHighIntensPoints(const CRectangle &r, CPointFeatureSet& points, int nScannerId = -1);

    // jzz :更新数据集内落在给定矩形区域内的点集合
    void CollectInRectPoints(const CRectangle &r, CPointFeatureSet& points, int nScannerId = -1);

    // 将两个数据进行合并
    void operator+=(const CSlamDataSet &other);

    // 删除指定的步
    void DeleteStep(int nStepId);

    // 删除指定的段
    void DeleteSteps(int nStarStepId, int nEndStepId);

    // 对数据集应用滤波规则
    void ApplyFilterRules(const CScanFilterRules &Rules);

    // 对数据集进行指定的坐标变换
    void Transform(const CFrame &frame);

    // 对数据集进行指定的坐标逆变换
    void InvTransform(const CFrame &frame);

    // 判断在某一步时，指定的屏幕点是否触及某个原始点
    int PointHitRawPoint(int nStepId, const CPnt &pt, float fDistGate);

    // 判断指定的屏幕点是否触及某一步的某个原始点
    int PointHitRawPoint(const CPnt &pt, float fDistGate, int &nStepId, int nStepInterval = 1);

    // 判断指定的屏幕点是否触及某个点特征
    int PointHitPointFeature(const CPnt &pt, float fDistGate);

    // 对指定的两个数据步进行匹配，并将结果保存到result中
    bool MatchSteps(int nStepId1, int nStepId2, sm_result &result);

    // 移动数据步nStepId，使其与数据步nRefStepId对齐
    bool AlignSingleStepWith(int nStepId, int nRefStepId, CPosture *ppstMove = NULL);

    // 对从第nStepId开始的所有的步按给定的姿态变化进行调整
    void AlignStepsWith(int nStepId, int nRefStepId);

    // 对从第nStepId开始的所有的步按给定的姿态变化进行调整(需要叠加给定的坐标变换)
    void AlignStepsWith(int nStepId, int nRefStepId,
                        Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> &trans);

    bool UpdateEstMove(int nFromStep = 0, int nToStep = 0);

    // 添加从nFromStepId到nToStepId的观测约束
    bool AddConstraint(int nFromStepId, int nToStepId);

    //
    bool AddConstraints();


    bool NewAddConstraints(int fromId, int toId);


    // 更新关于给定步的建议约束
    void UpdateSuggestedConstraints(int nStep, int nInterval);

    // 利用约束条件优化数据集
    bool Optimize();

    void SecondOptimization(void);

    void GetCartoConstraints(vector<CConstraint> &vtConstraints);

#ifdef CERES_OPTIMIZATION
    // 取得给定步的所有可用约束步
    vector<CConstraint> GetConstraintSteps(int nStep, int nInterval);
#endif

    // 利用“激光里程计”技术提高数据集准确度
    void ApplyLaserOdometry();

    // 根据局部数据生成全局数据
    void CreateGlobalData();

    // 施加虚拟环境
    void ApplyVirtObjects(CLiveObjects &objs);

    // 判断在某一步时，指定的屏幕点是否触及某个机器人位姿
    int PointHitPose(const CPnt &pt, float fDistGate);

    // 判断指定的屏幕点是否触及某个建议约束
    int PointHitSuggestedConstraint(const CPnt &pt, float fDistGate);

    // 判断指定的屏幕点是否触及某个确认约束
    int PointHitConstraint(const CPnt &pt, float fDistGate);

    void ClearAllConstraints();
    void DelSuggestedConstraint(int i);
    void DeleteConstraint(int index);

    void ClearSuggestedConstraints();

    void BuildMapOver();

    bool IsCartoSlaming(){return m_isCartoSlaming;}

    // 全局优化程序
    bool GlobalOptimizeProc(int nCurStep, int nInterval, int oldStepNum,bool bExpandMap,bool bFrozen);

    void StopMapping(void);

    void CartoReset();

    static void GridSlamCallback(slam_result *slam, std::vector<opt_result> *opt);

    static void BuildMapOverCallback();

    static void OptOverCallback(std::vector<opt_result> *opt);

    void UpdataSetData(std::vector<opt_result> *opt);

    void GridSlamUpdatePose(slam_result *slam, std::vector<opt_result> *opt);

    void SaveGridMap(std::string filename,std::unique_ptr<mapping::ProbabilityGrid> pmap);

    int GetFirstOptFinishedStepNum();

    int GetSecondOptFinishedStepNum();

    bool LoadExpandedDx(FILE *fp);

    void SetExpandedMap(std::unique_ptr<mapping::ProbabilityGrid> pmap);

    bool RotateMap(double angle);

#ifdef _MFC_VER

    // 显示某一步
    virtual void Plot(int nStepId, CScreenReference &ScrnRef, CDC *pDC, COLORREF clrRawPoint, bool bShowPose = false,
                      COLORREF crScanner = 0, bool bShowFeature = false, COLORREF clrFeature = 0);

    // 显示全图
    virtual void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF clrRawPoint, COLORREF clrPointFeature,
                      int nShowRawPoints = SHOW_RAW_POINTS, bool bShowPointFeature = true);

    // 画出两个步之间的点云的对应关系
    void PlotCorr(CScreenReference &ScrnRef, CDC *pDC, COLORREF clr, int nRefStep, int nCurStep,
                  bool bTransformScan = false);

#elif defined QT_VERSION
    // 显示某一步及其对比步
    virtual void Plot(int nStepId, CScreenReference &ScrnRef, QPainter *pPainter, QColor clrRawPoint,
                      bool bShowPose = false, QColor clrPose = QColor(255, 0, 255), int nCompStep = -1,
                      bool bTransformScan = false, QColor clrCompRawPoint = QColor(0, 0, 0),
                      QColor clrCompPose = QColor(0, 0, 0));

    // 显示全图
    virtual void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrRawPoint, bool bShowPose, QColor clrPose,
                      int interval = 10, int nMaxStep = -1);

    // 显示位姿
    void PlotPose(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrPose, int nStartStep = 0, int nEndStep = -1);

    // 显示点云配准的统计数据
    void ShowCorrStatus(QPainter *pPainter, int nCurStep);

    // 画出两个步之间的点云的对应关系
    void PlotCorr(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr, int nRefStep, int nCurStep,
                  bool bTransformScan = false, bool bForceCompare = false);

    // 画出所有的姿态约束
    void PlotConstraints(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr, int nMaxStep = -1);

    // 画出所有被固定的位姿
    void PlotFixedPoses(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr, int nMax = -1);

    void PlotSuggestedConstraints(CScreenReference &ScrnRef, QPainter *pPainter, int nStepId, int nInterval,
                                  QColor clr);
#endif
};

#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif

#endif
