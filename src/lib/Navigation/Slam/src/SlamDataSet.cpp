#include "stdafx.h"
#include "SlamDataSet.h"
#include "AffinePosture.h"

#include "probability_grid.h"
#include "probability_grid_range_data_inserter_2d.h"

#include "fast_correlative_scan_matcher_2d.h"
//#include "make_unique.h"

#include <map>

#ifdef CERES_OPTIMIZATION
#include "angle_local_parameterization.h"
#include "ceres/ceres.h"
#include "pose_graph_2d_error_term.h"
#include "types.h"
#include "hh.h"
#include "OptimizationProblem.h"
#endif

#ifdef NDT1_USE_MINI_EIGEN
#define Eigen MiniEigen
#endif

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   清除数据集。
//

   
using namespace mapping;

CSlamDataSet *pslamDataSetObj;



void CSlamDataSet::Clear()
{
    vector<CSlamStepData>::clear();

    m_ScannerParam.clear();

    m_bExpandMap = false;

#ifdef CERES_OPTIMIZATION
    m_Conditions.Clear();
    ClearSuggestedConstraints();
#endif
}

//
//   设置帧内的各个扫描点云是否在屏幕上可见。
//
void CSlamDataSet::UpdateScansVisibility()
{
    bool *pVisible = new bool[m_ScannerParam.size()];
    for (int i = 0; i < m_ScannerParam.size(); i++)
        pVisible[i] = m_ScannerParam[i].m_bVisible;

    for (int j = 0; j < (int)size(); j++)
        at(j).SetScansVisible(pVisible);

    delete []pVisible;
}

//   装入二进制原始扫描点数据文件。
//   返回值：
//   < 0 - 出错
//     0 - 判断此文件为未经综合的原始数据集文件，且目前已读完
//     1 - 尚未读完数据集的原始部分，状态正常
//     2 - 判断此文件为综合后的数据集文件，且目前原始数据已读完
//
#if 0
int CSlamDataSet::LoadRawScanBinary(FILE *fp)
{
    Clear();

    if (fread(&m_nFileVersion, sizeof(int), 1, fp) != 1)
        return -1;

    unsigned int uTimeStamp;
    if (m_nFileVersion >= 210)
    {
        if (fread(&uTimeStamp, sizeof(unsigned int), 1, fp) != 1)
            return false;

        m_uStartTime = uTimeStamp;
    }

    if (!m_ScannerParam.LoadBinary(fp, m_nFileVersion))
        return -1;

    clear();

    CSlamStepData StepData;

    // 如果版本在V2.00以上，接下来将直接读入总的帧数
    if (m_nFileVersion >= 200)
    {
        int nStepsCount = 0;
        if (fread(&nStepsCount, sizeof(int), 1, fp) != 1)
            return -1;

        nStepsCount--;
        bool bEof = false;
        // 根据帧数读入全部的局部数据
        for (int i = 0; i < nStepsCount; i++)
        {
            if (!StepData.LoadRawScanBinary(fp, m_ScannerParam, m_nFileVersion, (i == 0)))
            {
                bEof = true;
                break;
            }
            push_back(StepData);
        }

        // 试读下一个标志数据，看看文件是否结束(要求标志字符需要为'O')
        if (!bEof)
        {
            unsigned char ch;
            if (fread(&ch, sizeof(unsigned char), 1, fp) != 1 || ch != 'O')
            {
                // 已到文件尾，说明这是一个原始数据集文件，不含帧间匹配数据
            }
            // 应在此读入用户优化数据
            else
            {
#ifdef CERES_OPTIMIZATION
                if (!m_Conditions.LoadBinary(fp))
                    return false;
#endif
            }
        }
    }
    // 如果文件版本在V2.00以下，需要以文件尾作为结束判断条件
    else
    {
        // 读入全部的局部数据
        bool bFirstStep = true;
        while (StepData.LoadRawScanBinary(fp, m_ScannerParam, m_nFileVersion, bFirstStep))
        {
            push_back(StepData);
            bFirstStep = false;
        }
    }

    // 生成全部的全局数据
    CreateGlobalData();

    return true;
}

//
//   将原始扫描点数据写入二进制文件。
//
bool CSlamDataSet::SaveRawScanBinary(FILE *fp, int nFileVersion, int nFrom, int nTo, bool bReverseOrder,
                                     bool bSaveGlobalPosture)
{
    if (fwrite(&nFileVersion, sizeof(int), 1, fp) != 1)
        return false;

    // 依次写入各激光扫描器的参数
    if (!m_ScannerParam.SaveBinary(fp, nFileVersion))
        return false;

    if (nTo < 0)
        nTo = (int)size();

    // 如果版本在V2.00以上，接下来将直接写入总的帧数
    if (nFileVersion >= 200)
    {
        int nCount = nTo - nFrom + 1;
        if (fwrite(&nCount, sizeof(int), 1, fp) != 1)
            return false;
    }

    // 写入全部的局部数据
    // 如果是正序输出
    if (!bReverseOrder)
    {
        // 因为各帧数据有可能被更新过，所以保存数据集之前需要重新计算帧间姿态变化量
        UpdateEstMove(nFrom, nTo - 1);

        for (int i = nFrom; i < nTo; i++)
            at(i).SaveRawScanBinary(fp, nFileVersion, bSaveGlobalPosture);
    }
    // 如果是反序输出
    else
    {
        for (int i = nTo - 1; i >= nFrom; i--)
        {
            CSlamStepData Step = at(i);

            // 第一帧的移动里程一定为(0, 0, 0)
            if (i == nTo - 1)
                Step.m_pstMoveEst.SetPosture(0, 0, 0);

            // 其它帧的移动里程为其后一帧移动里程的逆
            else
            {
                CTransform trans(at(i + 1).m_pstMoveEst);
                trans = trans.Inv();
                Step.m_pstMoveEst.GetPostureObject() = trans;
            }
            Step.SaveRawScanBinary(fp, nFileVersion, bSaveGlobalPosture);
        }
    }

    // 写入全局优化用户设置数据
    if (nFileVersion >= 200)
    {
        unsigned char ch = 'O';
        if (fwrite(&ch, sizeof(unsigned char), 1, fp) != 1)
            return false;

#ifdef CERES_OPTIMIZATION
        if (!m_Conditions.SaveBinary(fp))
            return false;
#endif
    }

    return true;
}
#endif
int CSlamDataSet::LoadRawScanBinary(FILE *fp)
{
    Clear();

    if (fread(&m_nFileVersion, sizeof(int), 1, fp) != 1)
        return -1;

    unsigned int uTimeStamp;
    if (m_nFileVersion >= 210)
    {
        if (fread(&uTimeStamp, sizeof(unsigned int), 1, fp) != 1)
            return false;

        m_uStartTime = uTimeStamp;
    }

    if (!m_ScannerParam.LoadBinary(fp, m_nFileVersion))
        return -1;



    clear();

    CSlamStepData StepData;

    // 如果版本在V2.00以上，接下来将直接读入总的帧数
    if (m_nFileVersion >= 200)
    {
        int nStepsCount = 0;
        if (fread(&nStepsCount, sizeof(int), 1, fp) != 1)
            return -1;
        std::cout<<"nStepsCount"<<nStepsCount<<std::endl;

       // nStepsCount--;   ///????????
        bool bEof = false;
        // 根据帧数读入全部的局部数据
        for (int i = 0; i < nStepsCount; i++)
        {
            if (!StepData.LoadRawScanBinary(fp, m_ScannerParam, m_nFileVersion, (i == 0)))
            {
                bEof = true;
                break;
            }
            push_back(StepData);
        }

        // 试读下一个标志数据，看看文件是否结束(要求标志字符需要为'O')
       /* if (!bEof)
        {
            unsigned char ch;
            if (fread(&ch, sizeof(unsigned char), 1, fp) != 1 || ch != 'O')
            {
                // 已到文件尾，说明这是一个原始数据集文件，不含帧间匹配数据
            }
            // 应在此读入用户优化数据
            else
            {
#ifdef CERES_OPTIMIZATION
                if (!m_Conditions.LoadBinary(fp))
                    return false;
#endif
            }
        }*/
    }
    // 如果文件版本在V2.00以下，需要以文件尾作为结束判断条件
    else
    {
        // 读入全部的局部数据
        bool bFirstStep = true;
        while (StepData.LoadRawScanBinary(fp, m_ScannerParam, m_nFileVersion, bFirstStep))
        {
            push_back(StepData);
            bFirstStep = false;
        }
    }

    // 生成全部的全局数据
    CreateGlobalData();

    return true;
}

//
//   将原始扫描点数据写入二进制文件。
//
bool CSlamDataSet::SaveRawScanBinary(FILE *fp, int nFileVersion, int nFrom, int nTo, bool bReverseOrder,
                                     bool bSaveGlobalPosture)
{
    if (fwrite(&nFileVersion, sizeof(int), 1, fp) != 1)
        return false;

    m_nFileVersion = nFileVersion;

    if (m_nFileVersion >= 210)
    {
        if (fwrite(&m_uStartTime, sizeof(unsigned int), 1, fp) != 1)
            return false;

    }


    // 依次写入各激光扫描器的参数
    if (!m_ScannerParam.SaveBinary(fp, nFileVersion))
        return false;

    if (nTo < 0)
        nTo = (int)size();

    // 如果版本在V2.00以上，接下来将直接写入总的帧数
    if (nFileVersion >= 200)
    {
        //int nCount = nTo - nFrom + 1;  //lishen???
        int nCount = nTo - nFrom ;
        if (fwrite(&nCount, sizeof(int), 1, fp) != 1)
            return false;
    }

    // 写入全部的局部数据
    // 如果是正序输出
    if (!bReverseOrder)
    {
        // 因为各帧数据有可能被更新过，所以保存数据集之前需要重新计算帧间姿态变化量
        UpdateEstMove(nFrom, nTo - 1);

        for (int i = nFrom; i < nTo; i++)
            at(i).SaveRawScanBinary(fp, nFileVersion, bSaveGlobalPosture);
    }
    // 如果是反序输出
    else
    {
        for (int i = nTo - 1; i >= nFrom; i--)
        {
            CSlamStepData Step = at(i);

            // 第一帧的移动里程一定为(0, 0, 0)
            if (i == nTo - 1)
                Step.m_pstMoveEst.SetPosture(0, 0, 0);

            // 其它帧的移动里程为其后一帧移动里程的逆
            else
            {
                CTransform trans(at(i + 1).m_pstMoveEst);
                trans = trans.Inv();
                Step.m_pstMoveEst.GetPostureObject() = trans;
            }
            Step.SaveRawScanBinary(fp, nFileVersion, bSaveGlobalPosture);
        }
    }

    ////////////////??????????????
    // 写入全局优化用户设置数据
 /*   if (nFileVersion >= 200)
    {
        unsigned char ch = 'O';
        if (fwrite(&ch, sizeof(unsigned char), 1, fp) != 1)
            return false;

#ifdef CERES_OPTIMIZATION
        if (!m_Conditions.SaveBinary(fp))
            return false;
#endif
    }*/

    return true;
}

//
//   重新计算帧间姿态变化量。
//
bool CSlamDataSet::UpdateEstMove(int nFromStep, int nToStep)
{
    if (nToStep < nFromStep || nToStep >= (int)size() || nFromStep >= (int)size() || nFromStep < 0 || nToStep < 0)
        return false;
    else if (nFromStep == 0 && nToStep == 0)
        nToStep = (int)size() - 1;

    for (int i = nFromStep; i <= nToStep; i++)
    {
        CPosture &pstMoveEst = at(i).m_pstMoveEst;

        // 第一帧的移动里程一定为(0, 0, 0)
        if (i == 0)
            pstMoveEst.SetPosture(0, 0, 0);
        else
        {
            // 取得上一帧的激光器姿态，将其作为局部坐标系frm
            CFrame frm(at(i - 1).m_pstRobot);

            // 取得当前帧激光器姿态在frm中的姿态
            CPosture pstMove = at(i).m_pstRobot;
            pstMove.Transform(frm);

            pstMoveEst = pstMove;
        }
    }
}

//
//   启用新的激光器参数。
//
bool CSlamDataSet::SetScannerParam(const CScannerGroupParam &Param)
{
    m_ScannerParam = Param;

    // 重新生成全局数据
    CreateGlobalData();

    return true;
}

//
//   取得指定的世界散点。
//
CScanPoint *CSlamDataSet::GetWorldRawPoint(int nStepId, int nScannerId, int nIdxPoint)
{
    return at(nStepId).GetWorldRawPoint(nScannerId, nIdxPoint);
}

//
//   更新数据集内的高亮度点集合。
//
void CSlamDataSet::CollectHighIntensPoints(const CRectangle &r, CPointFeatureSet& points, int nScannerId)
{
    points.Clear();

    CPointFeatureSet stepHighPoints;
    stepHighPoints.EnableDistanceCache(false);

    for (int i = 0; i < (int)size(); i++)
    {
        at(i).CollectHighIntensPoints(r, stepHighPoints, nScannerId);
        points += stepHighPoints;
    }
}

//
//   jzz: 更新数据集在给定矩形区域内的点集合。
//
void CSlamDataSet::CollectInRectPoints(const CRectangle &r, CPointFeatureSet& points, int nScannerId)
{
    points.Clear();

    CPointFeatureSet stepInRectPoints;
    stepInRectPoints.EnableDistanceCache(false);

    for (int i = 0; i < (int)size(); i++)
    {
        at(i).CollectInRectPoints(r, stepInRectPoints, nScannerId);
        points += stepInRectPoints;
    }
}

// 将两个数据进行合并
void CSlamDataSet::operator+=(const CSlamDataSet &other)
{
    for (int i = 0; i < (int)other.size(); i++)
        push_back(other[i]);

    // 更新帧间相对位
    UpdateEstMove();
}

//
//   删除指定的帧。
//
void CSlamDataSet::DeleteStep(int nStepId)
{
    erase(begin() + nStepId);
    UpdateEstMove();
}

//
//   删除指定的段。
//
void CSlamDataSet::DeleteSteps(int nStarStepId, int nEndStepId)
{
    for (int i = nEndStepId; i >= nStarStepId; i--)
        erase(begin() + i);
    UpdateEstMove();
}

//
//   对数据集应用滤波规则。
//
void CSlamDataSet::ApplyFilterRules(const CScanFilterRules &Rules)
{
    // 依次处理各条规则
    for (int i = 0; i < (int)Rules.size(); i++)
    {
        const CScanFilterRule &Rule = Rules[i];

        // 依次处理各帧
        for (int j = 0; j < (int)size(); j++)
        {
            // 如果帧编号处于规则范围之内，则启用规则
            if (j >= Rule.m_nStartId && j <= Rule.m_nEndId)
            {
                // 如果是角度规则
                if (Rule.m_nType == 1)
                    at(j).ApplyScanAngleRule(Rule.m_nScannerId, Rule.m_fParam[0], Rule.m_fParam[1]);

                // 如果是距离规则
                else if (Rule.m_nType == 2)
                    at(j).ApplyScanDistRule(Rule.m_nScannerId, Rule.m_fParam[0], Rule.m_fParam[1]);
            }
        }
    }
}

//
//   对数据集进行指定的坐标变换。
//
void CSlamDataSet::Transform(const CFrame &frame)
{
    for (int i = 0; i < (int)size(); i++)
        at(i).Transform(frame);

    UpdateEstMove();
}

//
//   对数据集进行指定的坐标逆变换。
//
void CSlamDataSet::InvTransform(const CFrame &frame)
{
    for (int i = 0; i < (int)size(); i++)
        at(i).InvTransform(frame);

    UpdateEstMove();
}

//
//   判断在某一帧时，指定的屏幕点是否触及某个原始点。
//
int CSlamDataSet::PointHitRawPoint(int nStepId, const CPnt &pt, float fDistGate)
{
    return at(nStepId).PointHitRawPoint(pt, fDistGate);
}

//
//   判断指定的屏幕点是否触及某一帧的某个原始点。
//
int CSlamDataSet::PointHitRawPoint(const CPnt &pt, float fDistGate, int &nStepId, int nStepInterval)
{
    for (int i = 0; i < (int)size(); i++)
    {
        // 如果设定了跳帧间隔，则只筛选那些符合间隔要求的帧
        if ((i % nStepInterval) == 0)
        {
            int nHitRawPoint = PointHitRawPoint(i, pt, fDistGate);

            // 如果触碰到点，返回帧号和返回类型
            if (nHitRawPoint >= 0)
            {
                nStepId = i;
                return nHitRawPoint;
            }
        }
    }

    return -1;    // 未触碰到
}

//
//   判断在某一步时，指定的屏幕点是否触及某个点特征。
//
int CSlamDataSet::PointHitPointFeature(const CPnt &pt, float fDistGate)
{
    for (int i = 0; i < (int)m_reflectors.size(); i++)
    {
        if (m_reflectors[i]->PointHit(pt, fDistGate))
            return i;    //　触碰到一点，返回其编号
    }
    return -1;    // 未触碰到，近回-1
}

//
//   对指定的两个数据帧进行匹配，并将结果保存到result中。
//
bool CSlamDataSet::MatchSteps(int nStepId1, int nStepId2, sm_result &result)
{
    return at(nStepId1).Match(at(nStepId2), result);
}

//
//   移动数据帧nStepId，使其与数据帧nRefStepId对齐。
//
bool CSlamDataSet::AlignSingleStepWith(int nStepId, int nRefStepId, CPosture *ppstMove)
{
    return at(nStepId).AlignTo(at(nRefStepId), ppstMove);
}

#ifdef CERES_OPTIMIZATION
//
//   添加从nFromStepId到nToStepId的观测约束。
//   注意：如果准备添加的约束与现已存在的约束具有相同的节点对，则在此应修改约束数据，而不进行重复加入。
//
bool CSlamDataSet::AddConstraint(int nFromStepId, int nToStepId)
{
    sm_result result;
    if (MatchSteps(nFromStepId, nToStepId, result))
    {
        // 上面匹配得到的姿态差是相对于激光器的，需要转换计算出机器人的位姿差

        // 取得激光器相对于机器人的安装姿态
        CPosture pstScannerRelative = m_ScannerParam[0].m_pst;

        // 计算目标激光器的位姿
        CPosture pst(result.x[0], result.x[1], result.x[2]);
        CFrame frm1(at(nFromStepId).m_scanGlobal[0].m_pstScanner);   // jzz: target位姿
        pst.InvTransform(frm1);                                      // source 位姿（匹配后的）  【不是车体中心，是激光头在map下的位姿】

        // 计算目标机器人的位姿
        CPosture pstRobot(0, 0, 0);
        pstRobot.Transform(pstScannerRelative);
        CFrame frm2(pst);
        pstRobot.InvTransform(frm2);

        // 计算目标机器人与源机器人的位姿差
        pstRobot.Transform(at(nFromStepId).m_pstRobot);   // m_pstRobot是机器人（车中心）的位姿

        // pstRobot即map坐标系下nToStepId到nFromStepId的位姿差,是△量，也即，从nFromStepId观测到nToStepId的姿态（source相对targetg的△）
        m_Conditions.AddConstraint(nFromStepId, nToStepId, pstRobot);
        return true;
    }
    else
        return false;
}

//
//   添加由m_vectCache定义的约束。
//
bool CSlamDataSet::AddConstraints()
{
    if (m_nStepCache >= 0)
    {
        for (int i = 0; i < (int)m_DetectedConstraints.size(); i++)
        {
            int nStepId1 = m_DetectedConstraints[i].nStepId1;
            int nStepId2 = m_DetectedConstraints[i].nStepId2;
            AddConstraint(nStepId1, nStepId2);
        }
    }

    return true;
}

//
//   更新关于给定帧的建议约束。
//
void CSlamDataSet::UpdateSuggestedConstraints(int nStepId, int nInterval)
{
    if (m_nStepCache != nStepId)
    {
        m_DetectedConstraints = GetConstraintSteps(nStepId, nInterval);
        m_nStepCache = nStepId;
    }
}

//
//   利用约束条件优化数据集。
//
bool CSlamDataSet::Optimize()
{
#if 0
    COptimizationProblem Problem;
    Problem.Build(this);

    return Problem.Solve();
#else
    std::map<int, ceres::examples::Pose2d> poses;
    std::vector<ceres::examples::Constraint2d> constraints;

    poses.clear();
    constraints.clear();

    // 先加入所有位姿
    for (int i = 0; i < (int)size(); i++)
    {
        CSlamStepData &Step = at(i);
        CPosture pst = at(i).m_pstRobot;
        ceres::examples::Pose2d pose = {pst.x, pst.y, CAngle::NormAngle2(pst.fThita)};
        poses[i] = pose;
    }

    for (int i = 0; i < (int)size() - 1; i++)
    {
        CPosture pst = at(i + 1).m_pstMoveEst;

        ceres::examples::Constraint2d constraint;
        constraint.id_begin = i;
        constraint.id_end = i + 1;
        constraint.x = pst.x;
        constraint.y = pst.y;
        constraint.yaw_radians = CAngle::NormAngle2(pst.fThita);
        constraint.information(0, 0) = 44.72135955;
        constraint.information(0, 1) = 0;
        constraint.information(0, 2) = 0;
        constraint.information(1, 1) = 44.72135955;
        constraint.information(1, 2) = 0;
        constraint.information(2, 2) = 44.72135955;
        constraint.information(1, 0) = 0;
        constraint.information(2, 0) = 0;
        constraint.information(2, 1) = 0;

        constraints.push_back(constraint);
    }

    // 再加入所有约束
    for (int i = 0; i < (int)m_Conditions.m_Constraints.size(); i++)
    {
        CConstraint c = m_Conditions.m_Constraints[i];
        CPosture pst = c.pstObserved;

        ceres::examples::Constraint2d constraint;
        constraint.id_begin = c.nStepId1;
        constraint.id_end = c.nStepId2;
        constraint.x = pst.x;
        constraint.y = pst.y;
        constraint.yaw_radians = CAngle::NormAngle2(pst.fThita);
        constraint.information(0, 0) = 44.72135955;
        constraint.information(0, 1) = 0;
        constraint.information(0, 2) = 0;
        constraint.information(1, 1) = 44.72135955;
        constraint.information(1, 2) = 0;
        constraint.information(2, 2) = 44.72135955;
        constraint.information(1, 0) = 0;
        constraint.information(2, 0) = 0;
        constraint.information(2, 1) = 0;

        constraints.push_back(constraint);
    }

    ceres::Problem problem;

    ceres::examples::BuildOptimizationProblem1(constraints, &poses, &problem, m_Conditions.m_FixedPoses);

    bool ok = ceres::examples::SolveOptimizationProblem1(&problem);

    for (int i = 0; i < (int)size(); i++)
    {
        ceres::examples::Pose2d pose = poses[i];

        CPosture pst(pose.x, pose.y, pose.yaw_radians);
        at(i).TransformToRobotPose(pst);
    }
#endif

    UpdateEstMove();

    return true;
}

//
//   取得给定帧的所有可用约束帧。
//
vector<CConstraint> CSlamDataSet::GetConstraintSteps(int nStepId, int nInterval)
{
    vector<CConstraint> vect;

    bool flag = false;
    for (int i = 0; i < nStepId; i++)
    {

        if ((i % nInterval) != 0)
            continue;


        // 过滤掉相距太远的位姿(>25米)，减少ur匹配时间
        CPosture &pst1 = at(nStepId).m_pstRobot;
        CPosture &pst2 = at(i).m_pstRobot;

        float  dis = 5;

        if (pst1.DistanceTo(pst2) > dis)
            continue;

        float theta = common::NormalizeAngleDifference(pst1.fThita - pst2.fThita);

        std::cout<<" i=  "<<i<<" nStepId =  "<<nStepId<<std::endl;

       //             std::cout<<"nStepId      = "<<nStepId<<" "<<pst1.x<<"  "<<pst1.y<<"  "<<common::NormalizeAngleDifference(pst1.fThita)<<std::endl;
       //             std::cout<<"i = "<<i<<" "<<pst2.x<<"  "<<pst2.y<<"  "<<common::NormalizeAngleDifference(pst2.fThita)<<std::endl;

       //             std::cout<<"theta = "<<theta<<std::endl;


        sm_result result;
        bool csmok = false;


         csmok =  (MatchSteps(nStepId, i, result));

        if (csmok)
        {
            if (result.valid_percent < 0.3 || result.pstMove.DistanceTo(CPnt(0, 0)) > 0.1)
            {
                csmok = false;

                int nFromStepId = nStepId;
                int nToStepId = i;
                // 取得激光器相对于机器人的安装姿态
                CPosture pstScannerRelative = m_ScannerParam[0].m_pst;


                std::cout<<" at(nFromStepId) "<<at(nFromStepId).m_scanGlobal[0].m_pstScanner.x<<"  "<<at(nFromStepId).m_scanGlobal[0].m_pstScanner.y<<"  "<<at(nFromStepId).m_scanGlobal[0].m_pstScanner.fThita<<std::endl;
                std::cout<<" at(nToStepId  ) "<<at(nToStepId).m_scanGlobal[0].m_pstScanner.x<<"  "<<at(nToStepId).m_scanGlobal[0].m_pstScanner.y<<"  "<<at(nToStepId).m_scanGlobal[0].m_pstScanner.fThita<<std::endl;

                std::cout<<" result "<<result.x[0]<<"  "<<result.x[1]<<"  "<<result.x[2]<<std::endl;
                std::cout<<" result result"<<result.valid_percent<<"  "<<result.pstMove.DistanceTo(CPnt(0, 0))<<std::endl;


                // 计算目标激光器的位姿
                CPosture pst(result.x[0], result.x[1], result.x[2]);
                CFrame frm1(at(nFromStepId).m_scanGlobal[0].m_pstScanner);   // jzz: target位姿
                pst.InvTransform(frm1);                                      // source 位姿（匹配后的）  【不是车体中心，是激光头在map下的位姿】

                // 计算目标机器人的位姿
                CPosture pstRobot(0, 0, 0);
                pstRobot.Transform(pstScannerRelative);
                CFrame frm2(pst);
                pstRobot.InvTransform(frm2);


                std::cout<<" pstRobot "<<pstRobot.x<<"  "<<pstRobot.y<<"  "<<pstRobot.fThita<<std::endl;



            }
            else
            {
                   flag = true;
                   vect.push_back(CConstraint(i, nStepId));

                   int nFromStepId = nStepId;
                   int nToStepId = i;
                   // 取得激光器相对于机器人的安装姿态
                   CPosture pstScannerRelative = m_ScannerParam[0].m_pst;

                  // std::cout<<" pstScannerRelative "<<pstScannerRelative.x<<"  "<<pstScannerRelative.y<<"  "<<pstScannerRelative.fThita<<std::endl;

                   std::cout<<" at(nFromStepId) "<<at(nFromStepId).m_scanGlobal[0].m_pstScanner.x<<"  "<<at(nFromStepId).m_scanGlobal[0].m_pstScanner.y<<"  "<<at(nFromStepId).m_scanGlobal[0].m_pstScanner.fThita<<std::endl;
                   std::cout<<" at(nToStepId  ) "<<at(nToStepId).m_scanGlobal[0].m_pstScanner.x<<"  "<<at(nToStepId).m_scanGlobal[0].m_pstScanner.y<<"  "<<at(nToStepId).m_scanGlobal[0].m_pstScanner.fThita<<std::endl;

                  std::cout<<" result "<<result.x[0]<<"  "<<result.x[1]<<"  "<<result.x[2]<<std::endl;
                    std::cout<<"AddConstraint "<<std::endl;

                   // 计算目标激光器的位姿
                   CPosture pst(result.x[0], result.x[1], result.x[2]);
                   CFrame frm1(at(nFromStepId).m_scanGlobal[0].m_pstScanner);   // jzz: target位姿
                   pst.InvTransform(frm1);                                      // source 位姿（匹配后的）  【不是车体中心，是激光头在map下的位姿】

                   // 计算目标机器人的位姿
                   CPosture pstRobot(0, 0, 0);
                   pstRobot.Transform(pstScannerRelative);
                   CFrame frm2(pst);
                   pstRobot.InvTransform(frm2);

                   // 计算目标机器人与源机器人的位姿差
                   pstRobot.Transform(at(nFromStepId).m_pstRobot);   // m_pstRobot是机器人（车中心）的位姿

                   // pstRobot即map坐标系下nToStepId到nFromStepId的位姿差,是△量，也即，从nFromStepId观测到nToStepId的姿态（source相对targetg的△）


                   m_Conditions.AddConstraint(nFromStepId, nToStepId, pstRobot);

            }



        /*    std::cout<<" i=  "<<i<<" nStepId =  "<<nStepId<<std::endl;
            CPosture pos1 = at(i).m_scanLocal.at(0).m_pstScanner;
            CPosture pos2 = at(nStepId).m_scanLocal.at(0).m_pstScanner;
            std::cout<<"i       = "<<i<<" "<<pos1.x<<"  "<<pos1.y<<"  "<<pos1.fThita<<std::endl;
            std::cout<<"nStepId = "<<nStepId<<" "<<pos2.x<<"  "<<pos2.y<<"  "<<pos2.fThita<<std::endl;

           std::cout<<"sm result "<<result.x[0]<<" "<< result.x[1]<<" "<< result.x[2]<<std::endl;*/


        }


    }


#if 0
    if(flag == false)
    {
        int begin = max(0,nStepId-13);

        for (int i = begin; i < nStepId; i++)
        {
            if(i!=nStepId-1)
            {
                if ((i % nInterval) != 0)
                    continue;
            }
            CPosture ipos;

                ipos.x = 1;

            bool ok = at(nStepId).FastMatch(at(i),ipos);
            if(ok)
            {
                CPosture bakpos = at(i).m_scanLocal.at(0).m_pstScanner;
                if(ipos.fThita<0)
                    ipos.fThita = ipos.fThita +2*3.14159;

                at(i).m_scanLocal.at(0).m_pstScanner = ipos;
                        sm_result result;

                if (MatchSteps(nStepId, i, result))
                {
                    if (result.valid_percent < 0.3 || result.pstMove.DistanceTo(CPnt(0, 0)) > 0.3)
                        continue;

                    break;

                 //   std::cout<<" second i=  "<<i<<" nStepId =  "<<nStepId<<std::endl;
                    CPosture pos1 = at(i).m_scanLocal.at(0).m_pstScanner;
                    CPosture pos2 = at(nStepId).m_scanLocal.at(0).m_pstScanner;
                 //   std::cout<<"i       = "<<i<<" "<<pos1.x<<"  "<<pos1.y<<"  "<<pos1.fThita<<std::endl;
                 //   std::cout<<"nStepId = "<<nStepId<<" "<<pos2.x<<"  "<<pos2.y<<"  "<<pos2.fThita<<std::endl;

                 //  std::cout<<"sm result "<<result.x[0]<<" "<< result.x[1]<<" "<< result.x[2]<<std::endl;

                    vect.push_back(CConstraint(i, nStepId));

                  //     std::cout<< "****** csm match ok"<<std::endl;
                    int nFromStepId = nStepId;
                    int nToStepId = i;
                    // 取得激光器相对于机器人的安装姿态
                    CPosture pstScannerRelative = m_ScannerParam[0].m_pst;

                    // 计算目标激光器的位姿
                    CPosture pst(result.x[0], result.x[1], result.x[2]);
                    CFrame frm1(at(nFromStepId).m_scanGlobal[0].m_pstScanner);   // jzz: target位姿
                    pst.InvTransform(frm1);                                      // source 位姿（匹配后的）  【不是车体中心，是激光头在map下的位姿】

                    // 计算目标机器人的位姿
                    CPosture pstRobot(0, 0, 0);
                    pstRobot.Transform(pstScannerRelative);
                    CFrame frm2(pst);
                    pstRobot.InvTransform(frm2);

                       std::cout<<" pstRobot "<<pstRobot.x<<"  "<<pstRobot.y<<"  "<<pstRobot.fThita<<std::endl;

                    // 计算目标机器人与源机器人的位姿差
                    pstRobot.Transform(at(nFromStepId).m_pstRobot);   // m_pstRobot是机器人（车中心）的位姿

                    // pstRobot即map坐标系下nToStepId到nFromStepId的位姿差,是△量，也即，从nFromStepId观测到nToStepId的姿态（source相对targetg的△）


                    m_Conditions.AddConstraint(nFromStepId, nToStepId, pstRobot);
                     std::cout<< "****** csm match ok"<<std::endl;
                }
                else
                       std::cout<< "****** csm match faild"<<std::endl;

                at(i).m_scanLocal.at(0).m_pstScanner = bakpos;

            }


        }
    }
#endif


    return vect;
}
#endif

//
//   利用“激光里程计”技术提高数据集准确度。
//
void CSlamDataSet::ApplyLaserOdometry()
{
#if 0
    CSlamDataSet* copy = new CSlamDataSet(*this);

    CFrame frm;
    for (int i = 0; i < (int)size() - 1; i++)
    {
        CPosture pstMove;
        at(i + 1).InvTransform(frm);
        CPosture p0 = at(i+1).m_pstRobot;

        if (AlignSingleStepWith(i + 1, i, &pstMove))
            frm *= pstMove;
        else
            at(i + 1).m_bMatchLast = false;

        CPosture &p1 = at(i+1).m_pstRobot;
        CPosture &p2 = copy->at(i+1).m_pstRobot;

        UpdateEstMove();
        CPosture p3 = copy->at(i+1).m_pstRobot;

        copy->AlignStepsWith(i + 1, i);

        cout << "step " << i + 1 << ": (" << p0.x << ", " << p0.y << ", " << p0.fThita << ")," <<
                "(" << p3.x << ", " << p3.y << ", " << p3.fThita << "), " <<
                p1.x -p2.x << ", " << p1.y - p2.y << ", " << p1.fThita - p2.fThita << endl;
        int kkk = 9;
    }
#endif

//    UpdateEstMove(0, (int)size() - 1);
#if 1
    for (int i = 0; i < (int)size() - 1; i++)
        AlignStepsWith(i + 1, i);
#endif
}

//
//   对从第nStepId开始的所有的帧按给定的姿态变化进行调整。
//
void CSlamDataSet::AlignStepsWith(int nStepId, int nRefStepId)
{
    CPosture pstMove;
    AlignSingleStepWith(nStepId, nRefStepId, &pstMove);

    CFrame frm = pstMove;
	
#if 0
    // jzz test 添加随机噪声
    frm.x += (double)(rand() % 100 + 1) / 1000.f;
    frm.y += (double)(rand() % 100 + 1) / 1000.f;
    if (frm.x > 0.05 && frm.y > 0.05)
    {
        frm.x = -frm.x;
        frm.y = -frm.y;
    }
    else if (frm.x > 0.05)
        frm.x = -frm.x;
    else if (frm.y > 0.05)
        frm.y = -frm.y;

    cout << "x: " << frm.x << " " << "y: " << frm.y << endl;
#endif

    for (int i = nStepId + 1; i < (int)size() - 1; i++)
        at(i).InvTransform(frm);

    UpdateEstMove(0, (int)size() - 1);
}

//
//   对从第nStepId开始的所有的帧按给定的姿态变化进行调整(需要叠加给定的坐标变换)。
//
void CSlamDataSet::AlignStepsWith(int nStartStep, int nRefStepId,
                                  Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> &trans)
{
    CPosture pstMove;
    AlignSingleStepWith(nStartStep, nRefStepId, &pstMove);

    Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> tr;
    tr = PostureToAffine(pstMove.x, pstMove.y, pstMove.fThita);
    tr = tr * trans;

    CPosture pstTrans = AffineToPosture(tr);
    CFrame frm(pstTrans);

    for (int i = nStartStep + 1; i < (int)size() - 1; i++)
        at(i).InvTransform(frm);

    UpdateEstMove(0, (int)size() - 1);
}

//
//   根据给定的初始姿态生成全局数据。
//
void CSlamDataSet::CreateGlobalData()
{
    //lishen
    //CPosture pstInit(0, 0, 0);

    CPosture pstInit=at(0).m_pstRobot;

    for (int i = 0; i < (int)size(); i++)
        at(i).CreateGlobalData(pstInit, m_ScannerParam);
}

//
//   施加虚拟环境。
//
void CSlamDataSet::ApplyVirtObjects(CLiveObjects &objs)
{
    for (int i = 0; i < (int)size(); i++)
    {
        objs.Run(i);
        at(i).ApplyVirtObjects(objs, i);
    }
}

//
//   判断指定的屏幕点是否触及某个帧的机器人位姿。
//
int CSlamDataSet::PointHitPose(const CPnt &pt, float fDistGate)
{
    for (int i = 0; i < (int)size(); i++)
        if (at(i).PointHitPose(pt, fDistGate))
            return i;

    return -1;
}

#ifdef CERES_OPTIMIZATION
//
//   判断指定的屏幕点是否触及某个建议约束。
//
int CSlamDataSet::PointHitSuggestedConstraint(const CPnt &pt, float fDistGate)
{
    if (m_nStepCache < 0)
        return -1;

    for (int i = 0; i < (int)m_DetectedConstraints.size(); i++)
    {
        int id1 = m_DetectedConstraints[i].nStepId1;
        int id2 = m_DetectedConstraints[i].nStepId2;
        CPnt pt1 = at(id1).m_pstRobot;
        CPnt pt2 = at(id2).m_pstRobot;
        CLine ln(pt1, pt2);
        if (ln.PointHit(pt, fDistGate))
            return i;
    }

    return -1;
}

//
//    判断指定的屏幕点是否触及某个确认约束
//
int CSlamDataSet::PointHitConstraint(const CPnt &pt, float fDistGate)
{
    for (int i = 0; i < (int)m_Conditions.m_Constraints.size(); i++)
    {
        CConstraint &c = m_Conditions.m_Constraints[i];
        int id1 = c.nStepId1;
        int id2 = c.nStepId2;
        CPnt pt1 = at(id1).m_pstRobot;
        CPnt pt2 = at(id2).m_pstRobot;
        CLine ln(pt1, pt2);
        if (ln.PointHit(pt, fDistGate))
            return i;
    }

    return -1;
}

void CSlamDataSet::ClearAllConstraints()
{
    m_Conditions.Clear();
    ClearSuggestedConstraints();
}

void CSlamDataSet::DelSuggestedConstraint(int i)
{
    if (i >= m_DetectedConstraints.size())
        return;

    m_DetectedConstraints.erase(m_DetectedConstraints.begin() + i);
}

void CSlamDataSet::DeleteConstraint(int index)
{
#if 0
    if (i >= m_Constraints.size())
        return;

    m_Constraints.erase(m_Constraints.begin() + i);
#endif

    m_Conditions.DeleteConstraint(index);
}

void CSlamDataSet::ClearSuggestedConstraints()
{
    m_nStepCache = -1;
    m_DetectedConstraints.clear();
}

//
//   全局优化程序。
//

#if 0
bool CSlamDataSet::GlobalOptimizeProc(int nCurStep, int nInterval)
{
    // 仅当是关键帧时，才启动一次优化过程
    if (nCurStep >= nInterval && nCurStep < (int)size() && (nCurStep % nInterval) == 0)
    {
        // 查找可能的约束条件
        UpdateSuggestedConstraints(nCurStep, nInterval);

        // 向优化器中加入刚刚找到的约束条件
        AddConstraints();

        // 启动优化器
        Optimize();

        return true;
    }
    else
        return false;
}

#endif



#endif





#if 1

int g_oldStepNum = 0;

bool CSlamDataSet::GlobalOptimizeProc(int nCurStep, int nInterval,int oldStepNum,bool bExpandMap,bool bFrozen)
{
    static long timecur = 5000;
    static Pose curodom;



    // 如果当前还没有到最后一步，则后移一

    static unsigned long long lastLaserStampTime;
    int firstStep = oldStepNum ;

    g_oldStepNum = oldStepNum;

    std::cout<<"firstStep: "<<firstStep<<std::endl;

    if (nCurStep < (int)size() )
    {
        CSlamStepData StepData( at(nCurStep));
       if(nCurStep == firstStep )
       {
            pslamDataSetObj = this;

            if(m_pCartoSlam==nullptr)
            {
                m_pCartoSlam = CCartoSlam::GetInstance();

            }
            if(m_pCartoSlam!=nullptr)
            {
                auto pCartoParm = CartoParmSingleton::GetInstance();
                pCartoParm->SetDefaultCartoParm();
                pCartoParm->LoadJsonForParam();

                if(bExpandMap)
                {
                    Pose init{StepData.m_pstRobot.x ,StepData.m_pstRobot.y , StepData.m_pstRobot.fThita};

                    //std::cout<<"expand init"<< StepData.m_pstRobot.x<<" "<<StepData.m_pstRobot.y<<" "<<StepData.m_pstRobot.fThita<<std::endl;

                    double stamp = (double)(StepData.m_scanLocal[0].m_dwTimeStamp-1)/1000.0;

                    //std::cout<<"expand init 2"<< StepData.m_pstRobot.x<<" "<<StepData.m_pstRobot.y<<" "<<StepData.m_pstRobot.fThita<<std::endl;

                    //std::cout<<"stamp"<< stamp<<std::endl;

                    m_pCartoSlam->StartExpandMapnew(init,nullptr, stamp);

                    if(bFrozen)
                        m_bExpandMap = true;
                    else
                        m_bExpandMap = false;


                }
                else
                {
                    m_pCartoSlam->createMap();
                     m_bExpandMap = false;
                }
                 m_pCartoSlam->RegisterSlamCallBack((SlamResultCbFunc)GridSlamCallback);
                 m_pCartoSlam->RegisterBuildOverCallBack((BuildOverFunc)BuildMapOverCallback);
                 m_pCartoSlam->RegisterOptOverCallBack((OptOverFunc)OptOverCallback);


                 m_pCartoSlam->AddLaserID((std::string)("scan0"));

                 m_isCartoSlaming = true;
            }
            curodom.x = 0;
            curodom.y = 0;
            curodom.theta = 0;
            for(int j=0;j<(int)(size());j++)
            {
                at(j).m_bCartoNode = false;
                at(j).m_display = false;
                at(j).m_pstRobotBeforeOpt = at(j).m_pstRobot;
            }
            for(int j=0;j<firstStep;j++)
            {

                at(j).m_display = true;

            }
       }

       cout<<"nCurStep  = "<<nCurStep<<endl;




        int lasernum = StepData.m_scanLocal.size();   // laser sensor  numbers   1

        Pose localpos;

        for(int k=0;k<lasernum;k++)
        {
            if(k==1)
                break;

            sensor_msgs::LaserScan lscan;
            sensor_msgs::OdometryProto odom;


            Eigen::Matrix<double, 3, 1> lasertf(m_ScannerParam[k].m_pst.x,
                                                 m_ScannerParam[k].m_pst.y,
                                                 m_ScannerParam[k].m_pst.fThita) ;

             //std::cout << "laser tf "<<m_ScannerParam[k].m_pst.x<<" "<<m_ScannerParam[k].m_pst.y<<" "<<m_ScannerParam[k].m_pst.fThita<<std::endl;

             if(k == 0)
                lscan.sensor_id =(std::string)("scan0");
             if(k == 1)
                lscan.sensor_id =(std::string)("scan1");
            lscan.angle_increment = m_ScannerParam[k].m_fReso;
            lscan.angle_max = m_ScannerParam[k].m_fEndAngle;
            lscan.angle_min = m_ScannerParam[k].m_fStartAngle;

            //std::cout << "angle_min"<< lscan.angle_min<<" "<<lscan.angle_max<<" "<<lscan.angle_increment<<std::endl;

            lscan.laser_tf = lasertf;
            lscan.range_max =  m_ScannerParam[k].m_fMaxRange;
            lscan.range_min =  1;//  m_ScannerParam[k].m_fMinRange;
            lscan.scan_time = 0.02;   //50hz
            lscan.time_increment = 0.000005;

            std::cout << "odom m_pstStamped = "<< at(nCurStep).m_pst.m_dwTimeStamp<<std::endl;

            //std::cout << "StepData.m_scanLocal[k].m_dwTimeStamp  = "<< StepData.m_scanLocal[k].m_dwTimeStamp <<std::endl;


            if(nCurStep != firstStep )
            {
                if(StepData.m_scanLocal[k].m_dwTimeStamp == lastLaserStampTime)
                {
                    StepData.m_scanLocal[k].m_dwTimeStamp = StepData.m_scanLocal[k].m_dwTimeStamp+250;
                    at(nCurStep).m_scanLocal[k].m_dwTimeStamp = at(nCurStep).m_scanLocal[k].m_dwTimeStamp+250;
                }
            }


            lastLaserStampTime = StepData.m_scanLocal[k].m_dwTimeStamp;

            std::cout << "odom m_pstStamped = "<< StepData.m_pst.m_dwTimeStamp<<std::endl;


            long int sec = (long int)(StepData.m_scanLocal[k].m_dwTimeStamp/1000.0);
            long int usec = (long int)(((double)StepData.m_scanLocal[k].m_dwTimeStamp/1000.0-sec)*1000000ll);


             lscan.header.stamp.SetTime(sec , usec);

             //std::cout << "m_pst  "<< StepData.m_pst.x<<" "<<StepData.m_pst.y<<" "<<StepData.m_pst.fThita<<std::endl;
            // std::cout << "odom m_pstStamped = "<< StepData.m_pst.m_dwTimeStamp<<std::endl;
           //  std::cout << "m_nCount size ="<< m_ScannerParam[k].m_nLineCount<<std::endl;
             if(bExpandMap)
             {
                 curodom.x = StepData.m_pstRobot.x ;
                 curodom.y = StepData.m_pstRobot.y ;
                 curodom.theta = StepData.m_pstRobot.fThita;
             }
             else
             {

                 if(nCurStep == 0 )
                 {
                     //dq 建图起点根据dx确定
                     curodom.x = StepData.m_pstRobot.x ;
                     curodom.y = StepData.m_pstRobot.y ;
                     curodom.theta = StepData.m_pstRobot.fThita;

                     // curodom.x = 0;
                     // curodom.y = 0;
                     // curodom.theta = 0;
                 }
                 else
                 {
                     double s = sin(curodom.theta), c = cos(curodom.theta);

                     curodom.x = c*StepData.m_pstMoveEst.x - s*StepData.m_pstMoveEst.y + curodom.x;
                     curodom.y = s*StepData.m_pstMoveEst.x + c*StepData.m_pstMoveEst.y + curodom.y;
                     curodom.theta = curodom.theta + StepData.m_pstMoveEst.fThita;
                 }
             }


             odom.x = curodom.x;
             odom.y = curodom.y;
             odom.theta = curodom.theta;


             odom.stamp = (double)(StepData.m_scanLocal[k].m_dwTimeStamp-2)/1000.0;


             std::cout << "odom 1 "<< odom.x<<" "<<odom.y<<" "<<180.0*odom.theta/3.1415<<std::endl;

             std::cout << "odom 2 "<< StepData.m_pst.x<<" "<<StepData.m_pst.y<<" "<<std::endl;

            for(int m=0;m< m_ScannerParam[k].m_nLineCount;m++)
            {
                float r =  StepData.m_scanLocal[k].m_pPoints[m].r/1000.0;
                                       //r<???
                  lscan.ranges.push_back(r);
            }

            //dq 建图起点根据dx确定
            if(nCurStep == firstStep)
                m_pCartoSlam->SetInitNodePose(odom.stamp,curodom);

            //if(nCurStep == firstStep && bExpandMap)
            //{

            //    m_pCartoSlam->SetInitNodePose(odom.stamp,curodom);
            //}

            if(m_pCartoSlam!=nullptr)
            {
                m_pCartoSlam->SetFrozen(bFrozen);
                m_pCartoSlam->HandleEncoderData(odom);
                m_pCartoSlam->HandleLaserData(lscan);
                m_pCartoSlam->GetLocalPose(localpos);
            }

        }

        timecur = timecur+ 250;

    }


    if(nCurStep == ((int)size()-1))
        m_pCartoSlam->StopMapping();

    usleep(5000);
    return true;

}

#endif

Pose xytInvMul31(Pose a, Pose b)
{
    Pose res;
    double theta = a.theta;
    double ca = cos(theta), sa = sin(theta);
    double dx = b.x - a.x;
    double dy = b.y - a.y;

    res.x = ca*dx + sa*dy;
    res.y = -sa*dx + ca*dy;
    res.theta = b.theta - a.theta;

    return res;
}
Pose xytMultiply(Pose a, Pose b)
{
    Pose r;
    double s = sin(a.theta), c = cos(a.theta);

    r.x = c*b.x - s*b.y + a.x;
    r.y = s*b.x + c*b.y + a.y;
    r.theta = a.theta + b.theta;

    return r;
}


void CSlamDataSet::StopMapping(void)
{

    std::cout << "StopMapping  ********** "<<std::endl;

    if(m_pCartoSlam!=nullptr)
    {
         m_pCartoSlam->StopMapping();
    }

}

void *ResetCartoSlam(void *arg)
{
    CSlamDataSet *p = (CSlamDataSet *)arg;
    if (p== NULL)
        return NULL;

    p->CartoReset();

    return NULL;
}

void CSlamDataSet::CartoReset(void)
{
    usleep(1000);

   // m_pCartoSlam->Reset();  //lishen ?????????
}
void CSlamDataSet::BuildMapOver()
{
    std::cout << "BuildMapOverCallback  ********** "<<std::endl;


     if(m_pCartoSlam==nullptr)
         return;

     mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> allnodes = m_pCartoSlam->GetNodeData();


     map<int,Pose> optRes;

     Pose delta;
     int lastj = 0;

     lastj = allnodes.size();

     for (const auto& node_data : allnodes) {



         common::optional<mapping::TrajectoryNodePose::ConstantPoseData> constant_pose_data = node_data.data.constant_pose_data;

         mapping::TrajectoryNodePose::ConstantPoseData tmp = 	constant_pose_data.value();

         transform::Rigid3d global_pose = node_data.data.global_pose;


         double theta = transform::GetYaw(global_pose.rotation());
         Eigen::Matrix<double, 3, 1> xyz = global_pose.translation();

         Eigen::Matrix<double, 3, 1> mm = transform::RotationQuaternionToAngleAxisVector(global_pose.rotation());
         Eigen::Matrix<double, 3, 1> tt = global_pose.translation();


         Pose global;
         global.x = tt(0);
         global.y = tt(1);
         global.theta = mm(2);


         global.x = xyz(0);
         global.y = xyz(1);
         global.theta = theta;


         //std::cout<<" global.xyt" <<global.x<<" "<<global.y<<" "<<std::endl;

         for(int j=0;j<(int)(size());j++)
         {

             long int sec = (long int)(at(j).m_scanLocal[0].m_dwTimeStamp/1000.0);
             long int usec = (long int)(((double)at(j).m_scanLocal[0].m_dwTimeStamp/1000.0-sec)*1000000ll);


             if(tmp.time.Sec() == sec && tmp.time.Usec() == usec)
             {
                 optRes[j] = global;

                 Pose localpos;

                 localpos.x = at(j).m_pstRobot.x;
                 localpos.y = at(j).m_pstRobot.y;
                 localpos.theta = at(j).m_pstRobot.fThita;
                 delta =  xytInvMul31(global, localpos);

                 lastj = j;

             }

         }

     }
     for(int j=lastj+1;j<(int)size();j++)
     {
         Pose localpos;

         localpos.x = at(j).m_pstRobot.x;
         localpos.y = at(j).m_pstRobot.y;
         localpos.theta = at(j).m_pstRobot.fThita;

         Pose before = localpos;
         Pose after = xytMultiply(before, delta);


         optRes[j] = after;
     }


      map<int,Pose>::iterator iter;

      for(iter=optRes.begin();iter!=optRes.end();iter++)
      {
          CPosture pst(iter->second.x, iter->second.y, iter->second.theta);

          at(iter->first).TransformToRobotPose(pst);
         at(iter->first).m_scanLocal[0].m_pstScanner =  at(iter->first).m_scanGlobal[0].m_pstScanner;
      }




        UpdateEstMove();


        m_isCartoSlaming = false;

        pthread_t tid;
        pthread_create(&tid, NULL, ResetCartoSlam, this);
}


bool CSlamDataSet::NewAddConstraints(int fromId, int toId)
{
    bool csmok = false;
    float  disrange = 0.1;
    sm_result result;

   // std::cout<< " fromID = "<<fromId<< "toId =  "<<toId<<std::endl;

    csmok =  (MatchSteps(fromId, toId, result));   //i to

    if(fabs(fromId-toId)<40)
        disrange = 0.07;
    else
        disrange = 0.20;


  //  std::cout<< " result.pstMove x= "<<result.pstMove.x<< "y =  "<<result.pstMove.y<<"dis="<<result.pstMove.DistanceTo(CPnt(0, 0))<<std::endl;

    if (csmok)
    {
        if (result.valid_percent < 0.2 || result.pstMove.DistanceTo(CPnt(0, 0)) >disrange)
        {
             std::cout<<"&&&&&&&&&&&&& can   not    AddConstraint******!!!!!!! "<< result.valid_percent<<std::endl;
             return false;
        }
        else
        {
               std::cout<<"AddConstraint******!!!!!!! "<<std::endl;

               // 取得激光器相对于机器人的安装姿态
               CPosture pstScannerRelative = m_ScannerParam[0].m_pst;

               std::cout<<"hhh"<<m_ScannerParam[0].m_pst.x<< " "<<m_ScannerParam[0].m_pst.y<<" "<<m_ScannerParam[0].m_pst.fThita<<std::endl;
               // 计算目标激光器的位姿
               CPosture pst(result.x[0], result.x[1], result.x[2]);
               CFrame frm1(at(fromId).m_scanGlobal[0].m_pstScanner);   // jzz: target位姿
               pst.InvTransform(frm1);                                      // source 位姿（匹配后的）  【不是车体中心，是激光头在map下的位姿】

               // 计算目标机器人的位姿
               CPosture pstRobot(0, 0, 0);
               pstRobot.Transform(pstScannerRelative);
               CFrame frm2(pst);
               pstRobot.InvTransform(frm2);

               // 计算目标机器人与源机器人的位姿差
               pstRobot.Transform(at(fromId).m_pstRobot);   // m_pstRobot是机器人（车中心）的位姿

               // pstRobot即map坐标系下nToStepId到nFromStepId的位姿差,是△量，也即，从nFromStepId观测到nToStepId的姿态（source相对targetg的△）

               m_Conditions.AddConstraint(fromId, toId, pstRobot);

        }
    }
    return csmok;

}
void CSlamDataSet::GetCartoConstraints(vector<CConstraint> &vtConstraints)
{
    if(m_pCartoSlam ==nullptr)
        return;

    mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> trajectory_nodes_ = m_pCartoSlam->GetTrajectoryNodePoses();

    map<int,int> IdPair;
    map<int,int>::iterator iter_IdPair;

    for (const auto& node_id_data : trajectory_nodes_)
    {

        mapping::TrajectoryNodePose::ConstantPoseData constant_pose_data = 	node_id_data.data.constant_pose_data.value();

      //  if (node_id_data.data.constant_pose_data != nullptr)
        {

            common::Time tt = constant_pose_data.time;
             mapping::NodeId node_id = node_id_data.id;


            if(node_id.node_index>=0 && node_id.node_index<(int)(size()))
            {
                int begin = max(0,node_id.node_index-5);
                int end = min((int)(size()),node_id.node_index+5);
                bool flag = false;
                for(int j=begin;j<end;j++)
                {


                    long int sec = (long int)(at(j).m_scanLocal[0].m_dwTimeStamp/1000.0);
                    long int usec = (long int)(((double)at(j).m_scanLocal[0].m_dwTimeStamp/1000.0-sec)*1000000ll);

                    if(tt.Sec() == sec && tt.Usec() == usec)
                    {
                        IdPair[node_id.node_index] = j;
                        flag = true;
                        break;
                    }

                }
                if(!flag)
                {
                    for(int j=0;j<(int)(size());j++)
                    {
                        long int sec = (long int)(at(j).m_scanLocal[0].m_dwTimeStamp/1000.0);
                        long int usec = (long int)(((double)at(j).m_scanLocal[0].m_dwTimeStamp/1000.0-sec)*1000000ll);

                        if(tt.Sec() == sec && tt.Usec() == usec)
                        {
                            IdPair[node_id.node_index] = j;

                            break;
                        }


                    }
                }

            }
        }

    }

    map<int, vector<int>> contraintPairs = m_pCartoSlam->GetPossibleConstraintPairs();
    map<int, vector<int>>::iterator iter;

    for(iter = contraintPairs.begin();iter!=contraintPairs.end();iter++)
    {
        int node_id = iter->first;
        iter_IdPair = IdPair.find(node_id);

        if(iter_IdPair!=IdPair.end())
        {
            int fromId = IdPair[node_id];

            vector<int> toIds = iter->second;

            int count = 0;
            for(int i=0;i<toIds.size();i++)
            {
                int toId = toIds.at(i);
                bool icpok = NewAddConstraints(fromId,toId);

                count++;
                if(count>10)
                    break;
                if(icpok)
                    break;

            }
        }

    }

}


void CSlamDataSet::SecondOptimization(void)
{
    //////////////////////////////////////////////////////////////

    std::cout<<"seondopt"<<std::endl;

   for(int i=0; i<(int)size();i++)
    {
        at(i).m_scanLocal[0].m_pstScanner =  at(i).m_scanGlobal[0].m_pstScanner;
    }

    UpdateEstMove();

    vector<CConstraint> vtConstraints;
    GetCartoConstraints(vtConstraints);

    float   range = 3;
    int nInterval = 5;

       for(int i=0; i<(int)size();i =i+nInterval)
       {
            for(int j=0;j<i;j=j+nInterval)
            {
                CPosture &pst1 = at(i).m_pstRobot;
                CPosture &pst2 = at(j).m_pstRobot;


                if (pst1.DistanceTo(pst2) > range)
                    continue;

                  NewAddConstraints(i,j);
            }

       }

    Optimize();

}


 bool CSlamDataSet::LoadExpandedDx(FILE *fp)
 {
     LoadRawScanBinary(fp);

     if(m_pCartoSlam==nullptr)
     {
         m_pCartoSlam = CCartoSlam::GetInstance();

     }

     m_pCartoSlam->OpenExpandDx(fp);

     return true;
 }
void CSlamDataSet::SetExpandedMap(std::unique_ptr<mapping::ProbabilityGrid> pmap)
{
   // m_probMapExpanded = std::move(pmap);

}

void CSlamDataSet::SaveGridMap(std::string filename,std::unique_ptr<mapping::ProbabilityGrid> pmap)
{

    int idx = filename.rfind("/");
    string dir = filename.substr(0,idx+1);
    std::string pbstr = dir + "data.pb";

    FILE *fp1 = fopen(pbstr.c_str(), "wb");
    if (fp1 == NULL)
         return ;

    int nFileVersion = 210;
    bool bOK = SaveRawScanBinary(fp1,nFileVersion);

    m_pCartoSlam->SaveBinary(fp1);

    fclose(fp1);

    ///////save  probmap

    UpdateEstMove();

    int kInitialProbabilityGridSize = 100;
    double resolution = 0.05;

    Eigen::Vector2d max = 0.5 * kInitialProbabilityGridSize * resolution * Eigen::Vector2d::Ones();

    proto::ProbabilityGridRangeDataInserterOptions2D options;
    options.hit_probability = 0.550000 ;
    options.miss_probability = 0.49000 ;
    options.insert_free_space = true;

    mapping::ProbabilityGridRangeDataInserter2D range_data_inserter_(options);

    std::string probfilename;

    std::unique_ptr<mapping::ProbabilityGrid> pProbMap;

    if(pmap!=nullptr && m_bExpandMap  )    
        pProbMap = std::move(pmap); 
    else      
        pProbMap = common::make_unique<mapping::ProbabilityGrid>(MapLimits(resolution, max.x(),max.y(),CellLimits(kInitialProbabilityGridSize,kInitialProbabilityGridSize)));

    resolution = 0.1;
    mapping::ProbabilityGrid fastmap(MapLimits(resolution, max.x(),max.y(),CellLimits(kInitialProbabilityGridSize,kInitialProbabilityGridSize)));


    //for(int i=g_oldStepNum; i<(int)size();i=i+1)
    for(int i=0; i<(int)size();i=i+1)
    {
        sensor::RangeData all_range_data;

        //激光器在世界坐标系下的姿态
        CPosture pstScanner = at(i).m_scanGlobal[0].m_pstScanner;


        std::cout<<"i = "<<i<<" "<<pstScanner.x <<" "<<pstScanner.y<<std::endl;

        timeval time0;
        timeval time1;
        timeval time2;
        gettimeofday(&time0,NULL);

        for(int j=0;j<at(i).m_scanGlobal[0].m_nCount; j++)
        {
            CScanPoint &spGlobal = at(i).m_scanGlobal[0].m_pPoints[j];

            const Eigen::Vector3f hit_in_local(spGlobal.x,spGlobal.y, 0);
            const Eigen::Vector3f origin_in_local(pstScanner.x,pstScanner.y, 0);
            const Eigen::Vector3f delta = hit_in_local - origin_in_local;
            const float range = delta.norm();

            if (range >= m_ScannerParam.at(0).m_fMinRange)
            {
                if (range <= m_ScannerParam.at(0).m_fMaxRange)
                {
                         all_range_data.returns.push_back(hit_in_local);
                }
                else
                {
                         all_range_data.misses.push_back(origin_in_local + 5.0 / range * delta);
                }
            }
        }
        gettimeofday(&time1,NULL);

        double deltime1 = (time1.tv_sec - time0.tv_sec)*1000 + (double)(time1.tv_usec -time0.tv_usec)/1000 ;

        if(all_range_data.returns.size() >0)
        {
            Eigen::Vector3f tt(at(i).m_pstRobot.x,at(i).m_pstRobot.y,at(i).m_pstRobot.fThita);
            all_range_data.origin = tt; //mutable_trajectory_node.global_pose.cast<float>().translation();
            range_data_inserter_.Insert(all_range_data, pProbMap.get());

           // range_data_inserter_.Insert(all_range_data, &fastmap);

        }   

        gettimeofday(&time2,NULL);

        double deltime2 = (time2.tv_sec - time1.tv_sec)*1000 + (double)(time2.tv_usec -time1.tv_usec)/1000 ;

        printf(" &&&&&&&&&&&&&&&&&&&&  deltime1 = %f    %f\n", deltime1,deltime2 );

    }


    pProbMap->saveMap(filename);


#if 1


    std::unique_ptr<ProbabilityGrid> ffmap = pProbMap->CreateLowerResolutionMap(2);


  //  if(ffmap!=nullptr)
  //      ffmap->saveMap(dir+"fastmap.txt");
  //  else
  //      std::cout<<"error\n";

    string strname = dir+"Gridmap.map";

    FILE *fp = fopen(strname.c_str(), "wb");
    if (fp == NULL)
        false;

    int submapnum = 1;
    if (fwrite(&submapnum, sizeof(int), 1, fp) != 1)
         return ;

    int branch_and_bound_depth = 3;
    proto::FastCorrelativeScanMatcherOptions2D fastmatchoption={0.5, 3.1415/9, branch_and_bound_depth};
    std::shared_ptr<scan_matching::FastCorrelativeScanMatcher2D> matcher;

    matcher= std::make_shared<scan_matching::FastCorrelativeScanMatcher2D>(*(ffmap.get()), fastmatchoption);

        //子图ID
        int submapID = 0;
        if (fwrite(&submapID, sizeof(int), 1, fp) != 1)
         return ;

        matcher->SaveBinary(fp);

        fclose(fp);

         std::cout<<"save over   "<<std::endl;


#endif

      return;
 }

void CSlamDataSet::GridSlamCallback(slam_result *slam, std::vector<opt_result> *opt)
{
    pslamDataSetObj->GridSlamUpdatePose(slam, opt);
}

void CSlamDataSet::BuildMapOverCallback()
{
     pslamDataSetObj->BuildMapOver();
}
void CSlamDataSet::OptOverCallback(std::vector<opt_result> *opt)
{
     pslamDataSetObj->UpdataSetData(opt);
}

void CSlamDataSet::UpdataSetData(std::vector<opt_result> *opt)
{

    map<int,Pose> optRes;

    Pose delta;


   // std::cout<<"opt->size()"<<opt->size()<<" curid"<<curid<<std::endl;

    int j = 0;


    for (const auto& node_data : *opt)
    {
        optRes[j] = node_data.global_pose;
        j++;
    }



   // std::cout<<" count "<<count<<std::endl;

     map<int,Pose>::iterator iter;

     for(iter=optRes.begin();iter!=optRes.end();iter++)
     {
         CPosture pst(iter->second.x, iter->second.y, iter->second.theta);
         at(iter->first).TransformToRobotPose(pst);

     }

}

void CSlamDataSet::GridSlamUpdatePose(slam_result *slam, std::vector<opt_result> *opt)
{
      timeval curnodestamp = slam->stamp;
      int curid = -1;

      for(int j=0;j<(int)size();j++)
      {
          long int sec = (long int)(at(j).m_scanLocal[0].m_dwTimeStamp/1000.0);
          long int usec = (long int)(((double)at(j).m_scanLocal[0].m_dwTimeStamp/1000.0-sec)*1000000ll);

          if((long int)curnodestamp.tv_sec == sec && (long int)curnodestamp.tv_usec == usec)
          {
              curid = j;
              break;
          }

      }

      map<int,Pose> optRes;

      Pose delta;
      int lastj = 0;


      std::cout<<"opt->size()"<<opt->size()<<" curid"<<curid<<std::endl;

      int count = 0;

      for (const auto& node_data : *opt) {

          timeval opttime = node_data.stamp;

          int j= count;
          //for(int j=0;j<(int)(size());j++)
          {

              long int sec = (long int)(at(j).m_scanLocal[0].m_dwTimeStamp/1000.0);
              long int usec = (long int)(((double)at(j).m_scanLocal[0].m_dwTimeStamp/1000.0-sec)*1000000ll);


              //if(opttime.tv_sec == sec && opttime.tv_usec == usec)
              {
                  optRes[j] = node_data.global_pose;

                  Pose localpos;

                 // std::cout<<"j = "<<j<<"  "<<node_data.global_pose.x<<" "<<node_data.global_pose.y<<" "<<node_data.global_pose.theta<<std::endl;

                  localpos.x = at(j).m_pstRobot.x;
                  localpos.y = at(j).m_pstRobot.y;
                  localpos.theta = at(j).m_pstRobot.fThita;
                  delta =  xytInvMul31(node_data.global_pose, localpos);

                  lastj = j;
                   at(j).m_bCartoNode = true;
                   at(j).m_display = true;

                   count++;
              }
          }
      }

     // std::cout<<" count "<<count<<std::endl;

      for(int j=lastj+1;j<curid;j++)
      {
          Pose localpos;

          localpos.x = at(j).m_pstRobot.x;
          localpos.y = at(j).m_pstRobot.y;
          localpos.theta = at(j).m_pstRobot.fThita;

          Pose before = localpos;
          Pose after = xytMultiply(before, delta);

          optRes[j] = after;

      }


       map<int,Pose>::iterator iter;

       for(iter=optRes.begin();iter!=optRes.end();iter++)
       {
           CPosture pst(iter->second.x, iter->second.y, iter->second.theta);
           at(iter->first).TransformToRobotPose(pst);

       }

}

bool CSlamDataSet::RotateMap(double angle)
{
    if(m_pCartoSlam!=nullptr)
        m_pCartoSlam->RotateMap(angle);
}


int CSlamDataSet::GetFirstOptFinishedStepNum()
{
    if(m_pCartoSlam==nullptr)
        return 0;
    return m_pCartoSlam->GetNumFinishedNodes();
}

int CSlamDataSet::GetSecondOptFinishedStepNum()
{

}



#ifdef _MFC_VER

void CSlamDataSet::Plot(int nStepId, CScreenReference &ScrnRef, CDC *pDC, COLORREF clrRawPoint, bool bShowPose,
                        COLORREF crScanner, bool bShowFeature, COLORREF clrFeature)
{
    at(nStepId).Plot(ScrnRef, pDC, clrRawPoint, RGB(255, 255, 0), bShowPose, crScanner, bShowFeature, clrFeature);
}

#elif defined QT_VERSION

//
//  显示某一帧及其对比帧。
//
void CSlamDataSet::Plot(int nStepId, CScreenReference &ScrnRef, QPainter *pPainter, QColor clrRawPoint, bool bShowPose,
                        QColor clrPose, int nCompStepId, bool bTransformScan, QColor clrCompRawPoint,
                        QColor clrCompPose)
{
    // 显示“待比较帧”情况
    if (nCompStepId >= 0)
    {
        at(nCompStepId).Plot(ScrnRef, pPainter, clrCompRawPoint, clrCompRawPoint, true, clrCompPose);
    }

    // 在无需变换，或无需比较的情况下，直接显示当前帧的状态
    if (nCompStepId < 0 || !bTransformScan || !m_result.valid)
        at(nStepId).Plot(ScrnRef, pPainter, clrRawPoint, Qt::yellow, bShowPose, clrPose);
    else
        m_TransformedStep.Plot(ScrnRef, pPainter, Qt::cyan, Qt::cyan, bShowPose, Qt::cyan);
}

//
//   显示全图。
//
void CSlamDataSet::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrRawPoint, bool bShowPose,
                        QColor clrPose, int interval, int nMaxStep)
{
    if (nMaxStep < 0)
        nMaxStep = (int)size() - 1;

    // 画出所有的点云(不画激光器)
    for (int i = 0; i <= nMaxStep; i++)
    {
        if (i % interval == 0)
            at(i).Plot(ScrnRef, pPainter, clrRawPoint, Qt::yellow, false, clrPose, false);
    }

    // 重新绘制所有的高亮点(在此单独画它们可以避免被点云遮挡)
    for (int i = 0; i <= nMaxStep; i++)
    {
        if (i % interval == 0)
            at(i).PlotHighIntensPoints(ScrnRef, pPainter, Qt::yellow);
    }

    // 绘制所有的激光器位姿(在此单独画它们可以避免被点云遮挡)
    if (bShowPose)
        for (int i = 0; i <= nMaxStep; i++)
        {
                if(!at(i).m_display)
                    continue;
            QColor clrPose1 = clrPose;
            if (!at(i).m_bMatchLast)
                clrPose1 = Qt::gray;
            QString str;
            str.sprintf("%d", i + 1);
            at(i).m_pstRobot.Draw(ScrnRef, pPainter, clrPose1, clrPose1, 40, 150, 1, str, Qt::white);
        }
}

//
//   显示位姿。
//
void CSlamDataSet::PlotPose(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrPose, int nStartStep, int nEndStep)
{
    if (nEndStep < 0)
        nEndStep = (int)size() - 1;

    for (int i = nStartStep; i <= nEndStep; i++)
    {

        at(i).m_pstRobot.Draw(ScrnRef, pPainter, clrPose, clrPose, 40, 150);
    }
}

//
//   显示点云配准的统计数据。
//
void CSlamDataSet::ShowCorrStatus(QPainter *pPainter, int nCurStep)
{
    if (!m_result.valid)
        return;

    pPainter->setPen(Qt::lightGray);

    // 显示散点匹配数据
    QString str;
    str.sprintf("匹配点数:%d(%d%%), 匹配误差:%.3f, %.3f", m_result.nvalid, (int)(m_result.valid_percent * 100),
                (float)(m_result.error), (float)(m_result.error2));
    pPainter->drawText(10, 45, str);

    str.sprintf("循环次数:%d, 运行时间:%d", m_result.iterations, m_result.time_cost);
    pPainter->drawText(10, 70, str);

    CFrame &frm = at(nCurStep).m_frmToLastStep;
    str.sprintf("位姿差距:(%.3f, %.3f, %.2f)", frm.x, frm.y, CAngle::NormAngle2(frm.fThita) * 180 / PI);
    pPainter->drawText(10, 95, str);
}

//
//   画出两个帧之间的点云的对应关系.
//
void CSlamDataSet::PlotCorr(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr, int nRefStep, int nCurStep,
                            bool bTransformScan, bool bForceCompare)
{
    CSlamStepData &ref = at(nRefStep);
    CSlamStepData &cur = (bTransformScan) ? m_TransformedStep : at(nCurStep);

    CCorrList *pList = NULL;

    // 即时计算匹配关系
    if (m_nCorrCurStep < 0 || m_nCorrRefStep < 0 || m_nCorrCurStep != nCurStep || m_nCorrRefStep != nRefStep ||
        bForceCompare)
    {
        m_result.valid = false;
        if (at(nCurStep).PointCloudMatch(at(nRefStep), &m_corrList, &m_result))
        {
            m_TransformedStep = at(nCurStep);
            m_TransformedStep.InvTransform(m_result.pstMove);
            m_nCorrCurStep = nCurStep;
            m_nCorrRefStep = nRefStep;
        }
        else
        {
            return;
        }
    }

    pList = &m_corrList;
    if (pList->m_pIdx == NULL)
        return;

    for (int i = 0; i < pList->m_nCount; i++)
    {
        if (pList->m_pIdx[i] >= 0)
        {
            CPnt &pt1 = cur.m_scanGlobal[0].m_pPoints[i * CSM_SAMPLE_NUM];
            CPnt &pt2 = ref.m_scanGlobal[0].m_pPoints[pList->m_pIdx[i] * CSM_SAMPLE_NUM];
            if (pt1.DistanceTo(pt2) < 0.001)
                continue;

            CLine ln(pt1, pt2);
            ln.Draw(ScrnRef, pPainter, clr);
        }
    }
}

#ifdef CERES_OPTIMIZATION
//
//   画出所有的姿态约束。
//
void CSlamDataSet::PlotConstraints(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr, int nMaxStep)
{
    if (nMaxStep < 0)
        nMaxStep = (int)size() - 1;

    for (int i = 0; i < (int)m_Conditions.m_Constraints.size(); i++)
    {
        CConstraint &c = m_Conditions.m_Constraints[i];
        int id1 = c.nStepId1;
        int id2 = c.nStepId2;
        if (id1 > nMaxStep || id2 > nMaxStep)
            continue;

        CPnt pt1 = at(id1).m_pstRobot;
        CPnt pt2 = at(id2).m_pstRobot;
        CLine ln(pt1, pt2);
        ln.Draw(ScrnRef, pPainter, clr, 2, 3, true);
    }
}

//
//   画出所有被固定的位姿。
//
void CSlamDataSet::PlotFixedPoses(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr, int nMaxStep)
{
    if (nMaxStep < 0)
        nMaxStep = (int)size() - 1;

    for (int i = 0; i < (int)m_Conditions.m_FixedPoses.size(); i++)
    {
        int j = m_Conditions.m_FixedPoses[i];
        if (j > nMaxStep)
            continue;

        at(j).m_pstRobot.Draw(ScrnRef, pPainter, clr, clr, 60, 150);
    }
}

//
//   画出所有可能的姿态约束。
//
void CSlamDataSet::PlotSuggestedConstraints(CScreenReference &ScrnRef, QPainter *pPainter, int nStepId, int nInterval,
                                            QColor clr)
{
    for (int i = 0; i < (int)m_DetectedConstraints.size(); i++)
    {
        int id1 = m_DetectedConstraints[i].nStepId1;
        int id2 = m_DetectedConstraints[i].nStepId2;
        CPnt pt1 = at(id1).m_pstRobot;
        CPnt pt2 = at(id2).m_pstRobot;
        CLine ln(pt1, pt2);
        ln.Draw(ScrnRef, pPainter, clr, 2, 3, true);
    }
}
#endif

#endif
