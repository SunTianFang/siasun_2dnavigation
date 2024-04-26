#include "stdafx.h"
#include "SlamStepData.h"
#include "ScrnRef.h"
#include "Frame.h"
#include "FeatureCreationParam.h"
#include "CsmMatcher.h"

#include "fast_correlative_scan_matcher_2d.h"
#include "probability_grid.h"
#include "probability_grid_range_data_inserter_2d.h"
#include  "voxel_filter.h"
#include "sensorproto.h"
#include "ceres_scan_matcher_2d.h"


#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define SCANNER_ARROW_LEN 0.45f    // 箭头长450mm
#define SCANNER_RADIUS 0.08f    // 显示扫描器位姿的圆形的半径

extern CFeatureCreationParam FeatureCreationParam;

//#define ADD_NOISE
//#define DEL_ODOMETRY_DATA

int kkkkk = 0;

///////////////////////////////////////////////////////////////////////////////

//
//   拷贝构造函数。
//
CSlamStepData::CSlamStepData(const CSlamStepData &Obj)
{
    m_pstMoveEst = Obj.m_pstMoveEst;
    m_pstRobot = Obj.m_pstRobot;
    m_scanLocal = Obj.m_scanLocal;
    m_scanGlobal = Obj.m_scanGlobal;
    m_bCorrOk = Obj.m_bCorrOk;
    m_corrList = Obj.m_corrList;
    m_smResult = Obj.m_smResult;
    m_bMatchLast =  Obj.m_bMatchLast;

    m_pst = Obj.m_pst;



     m_display = Obj.m_display;
     m_bCartoNode = Obj.m_bCartoNode;
     m_pstRobotBeforeOpt = Obj.m_pstRobotBeforeOpt;
}

//
//   二进制文件读取原始扫描点数据。
//
bool CSlamStepData::LoadRawScanBinary(FILE *fp, const CScannerGroupParam &Param, int nFileVersion, bool bFirstStep)
{
    m_scanLocal.clear();
    m_scanLocal.resize(Param.size());

    m_scanGlobal.clear();
    m_scanGlobal.resize(Param.size());

    // 如果文件版本在V2.00以下
    if (nFileVersion < 200)
    {
        // 读入观测姿态
        float f[3];
        if (fread(&f, sizeof(float), 3, fp) != 3)
            return false;

#ifdef DEL_ODOMETRY_DATA
        f[0] = f[1] = f[2] = 0;
#endif

        m_pstMoveEst.x = f[0];
        m_pstMoveEst.y = f[1];
        m_pstMoveEst.fThita = f[2];

#ifdef ADD_NOISE
        if ((kkkkk++ % 4) == 0)
        {
            m_pstMoveEst.x -= 0.2f * ((kkkkk % 3) - 1);
//            m_pstMoveEst.y += 0.2f * ((kkkkk % 7) - 1);

            m_pstMoveEst.fThita += TO_RADIAN(5.0f * ((kkkkk %3) - 1));
        }
#endif

    }

    // 如果文件版本高于V2.00
    else
    {
        unsigned int u, uTimeStamp;
        if (fread(&u, sizeof(int), 1, fp) != 1)
            return false;

        // 版本在V2.1以上时，数据具有时间戳
        if (nFileVersion >= 210)
        {
            if (fread(&uTimeStamp, sizeof(int), 1, fp) != 1)
                return false;

            // 为相对姿态打上时间戳
            m_pstMoveEst.Stamp(uTimeStamp);
        }

        float f[3];

        // (u & BIT(1)) != 0 , 表示读入速度向量
        if ((u & 0x04) != 0)
        {
            if (fread(&f, sizeof(float), 3, fp) != 3)
                return false;

            m_vel.x = f[0];
            m_vel.y = f[1];
            m_vel.fThita = f[2];
        }

        // 读入观测姿态
        if (fread(&f, sizeof(float), 3, fp) != 3)
            return false;

#ifdef DEL_ODOMETRY_DATA
        f[0] = f[1] = f[2] = 0;
#endif

        m_pstMoveEst.x = f[0];
        m_pstMoveEst.y = f[1];
        m_pstMoveEst.fThita = f[2];

#ifdef ADD_NOISE
        if ((kkkkk++ % 4) == 0)
        {
            m_pstMoveEst.x -= 0.2f * ((kkkkk % 3) - 1);
//            m_pstMoveEst.y += 0.2f * ((kkkkk % 7) - 1);
            m_pstMoveEst.fThita += TO_RADIAN(5.0f * ((kkkkk % 3) - 1));
        }
#endif

        // (u & BIT(1)) != 0 , 表示读入绝对姿态值
        if ((u & 0x02) != 0)
        {
            if (fread(&f, sizeof(float), 3, fp) != 3)
                return false;

            // 在此读入机器人的绝对姿态
            m_pst.x = f[0];
            m_pst.y = f[1];
            m_pst.fThita = f[2];

            //lishen
            m_pstRobot.x = f[0];
            m_pstRobot.y = f[1];
            m_pstRobot.fThita = f[2];


            if (nFileVersion >= 210)
                m_pst.Stamp(uTimeStamp);
        }
    }

    // 数据集的第一步，姿态增量必须为零！
    if (bFirstStep)
        m_pstMoveEst.SetPosture(0, 0, 0);

    // 分别读取各个激光器的扫描数据
    for (size_t i = 0; i < Param.size(); i++)
    {
        // 读入局部点云数据
        if (!m_scanLocal[i].LoadBinary(fp, m_pstMoveEst, Param[i], nFileVersion))
            return false;
    }

    return true;
}

//
//   将原始扫描点数据写入二进制文件。
//
bool CSlamStepData::SaveRawScanBinary(FILE *fp, int nFileVersion, bool bSaveGlobalPosture)
{
    // 如果文件版本高于2.00，需要写入姿态类型0，表示后面仅写入相对姿态
    // (如果姿态类型为0x02，则表示后面会连续写入相对姿态和绝对姿态)
    if (nFileVersion >= 200)
    {
        unsigned int u = 0;
        if (bSaveGlobalPosture)
            u |= 0x06;

        if (fwrite(&u, sizeof(unsigned int), 1, fp) != 1)
            return false;

         //std::cout<<"u "<<u<<std::endl;

        // 版本在V2.1以上时，数据具有时间戳
        if (nFileVersion >= 210)
        {
             unsigned int uTimeStamp = m_pstMoveEst.m_dwTimeStamp;
            if (fwrite(&uTimeStamp, sizeof(unsigned int), 1, fp) != 1)
                return false;

           // std::cout<<"uTimeStamp "<<uTimeStamp<<std::endl;

        }


        if ((u & 0x04) != 0)
        {
           float f[3];
           f[0] = m_vel.x;
           f[1] = m_vel.y;
           f[2] = m_vel.fThita;

           //std::cout<<"m_vel.x "<<m_vel.x<<"m_vel.y "<<m_vel.y<<std::endl;

           if (fwrite(&f, sizeof(float), 3, fp) != 3)
               return false;
        }



        float f[3];
        f[0] = m_pstMoveEst.x;
        f[1] = m_pstMoveEst.y;
        f[2] = m_pstMoveEst.fThita;

        if (fwrite(&f, sizeof(float), 3, fp) != 3)
            return false;

         //  std::cout<<"m_pstMoveEst.x "<<m_pstMoveEst.x<<"m_pstMoveEst.y "<<m_pstMoveEst.y<<std::endl;

        // 如果要求数据格式中含有绝对姿态，现在写入它
        if ((u & 0x02) != 0)
        {
            //lishen
            //f[0] = m_pst.x;
            //f[1] = m_pst.y;
            //f[2] = m_pst.fThita;

            f[0] = m_pstRobot.x;
            f[1] = m_pstRobot.y;
            f[2] = m_pstRobot.fThita;

            if (fwrite(&f, sizeof(float), 3, fp) != 3)
                return false;
        }
    }

    // 写入点云数据
    for (size_t i = 0; i < m_scanLocal.size(); i++)
        if (!m_scanLocal[i].SaveBinary(fp, nFileVersion))
            return false;

    return true;
}

#if 0
//
//   将原始扫描点数据写入二进制文件。
//
bool CSlamStepData::SaveRawScanBinary(FILE *fp, int nFileVersion, bool bSaveGlobalPosture)
{
    // 如果文件版本高于2.00，需要写入姿态类型0，表示后面仅写入相对姿态
    // (如果姿态类型为0x02，则表示后面会连续写入相对姿态和绝对姿态)
    if (nFileVersion >= 200)
    {
        unsigned int u = 0;
        if (bSaveGlobalPosture)
            u |= 0x02;

        if (fwrite(&u, sizeof(int), 1, fp) != 1)
            return false;

        float f[3];
        f[0] = m_pstMoveEst.x;
        f[1] = m_pstMoveEst.y;
        f[2] = m_pstMoveEst.fThita;

        if (fwrite(&f, sizeof(float), 3, fp) != 3)
            return false;

        // 如果要求数据格式中含有绝对姿态，现在写入它
        if ((u & 0x02) != 0)
        {
            //lishen
            //f[0] = m_pst.x;
            //f[1] = m_pst.y;
            //f[2] = m_pst.fThita;

            f[0] = m_pstRobot.x;
            f[1] = m_pstRobot.y;
            f[2] = m_pstRobot.fThita;

            if (fwrite(&f, sizeof(float), 3, fp) != 3)
                return false;
        }
    }

    // 写入点云数据
    for (size_t i = 0; i < m_scanLocal.size(); i++)
        if (!m_scanLocal[i].SaveBinary(fp, nFileVersion))
            return false;

    return true;
}


#endif

//   清除所有数据。
//
void CSlamStepData::Clear()
{
    m_bCorrOk = false;
    m_bMatchLast = true;
}

#if 1
//
//   在给定的矩形区域中收集本步数据内的所有高亮度点。
//   说明：
//    nScannerId >= 0: 仅考虑该激光器的数据
//    nScannerId < 0:  考虑所有激光器的数据
//
void CSlamStepData::CollectHighIntensPoints(const CRectangle &r, CPointFeatureSet &highIntensPoints, int nScannerId)
{
    highIntensPoints.Clear();

    for (int i = 0; i < (int)m_scanGlobal.size(); i++)
    {
        if (nScannerId >= 0 && i != nScannerId)
            continue;

        CScan &scan = m_scanGlobal[i];
        for (int j = 0; j < (int)scan.m_nCount; j++)
        {
            CScanPoint &sp = scan.m_pPoints[j];
            if (r.Contain(sp) && sp.m_bHighReflective)
            {
                CPointFeature p(scan.m_pPoints[j]);
                highIntensPoints += p;
            }
        }
    }
}
#endif

#if 1
//   jzz:
//   在给定的矩形区域中收集本步数据内的所有点。
//   说明：
//    nScannerId >= 0: 仅考虑该激光器的数据
//    nScannerId < 0:  考虑所有激光器的数据
//
void CSlamStepData::CollectInRectPoints(const CRectangle &r, CPointFeatureSet &inRectPoints, int nScannerId)
{
    inRectPoints.Clear();

    for (int i = 0; i < (int)m_scanGlobal.size(); i++)
    {
        if (nScannerId >= 0 && i != nScannerId)
            continue;

        CScan &scan = m_scanGlobal[i];
        for (int j = 0; j < (int)scan.m_nCount; j++)
        {
            CScanPoint &sp = scan.m_pPoints[j];
            if (r.Contain(sp))
            {
                CPointFeature p(scan.m_pPoints[j]);
                inRectPoints += p;
            }
        }
    }
}
#endif

//
//   设置步内的各个扫描点云是否在屏幕上可见。
//
void CSlamStepData::SetScansVisible(bool *pVisible)
{
    for (int i = 0; i < (int)m_scanGlobal.size(); i++)
        m_scanGlobal[i].SetVisible(pVisible[i]);
}

//
//   根据给定的原始坐标系，将数据转换到世界坐标系下。
//
void CSlamStepData::CreateGlobalData(CPosture &pstInit, const CScannerGroupParam &ScannerGroupParam)
{
    CFrame frame1(pstInit);
    pstInit = m_pstMoveEst;
    pstInit.InvTransform(frame1);
    m_pstRobot = pstInit;      // 机器人在世界坐标系内的绝对姿态

    // 将点云变换到世界坐标系中
    for (size_t i = 0; i < m_scanLocal.size(); i++)
    {
        // 根据激光器在机器人上的相对安装位置计算其在世界坐标系下的姿态
        CFrame frmRobot(m_pstRobot);
        const CLaserScannerParam &Param = ScannerGroupParam[i];    // 对应激光器的参数
        CPosture pstScanner(Param.m_pst);                          // 激光器相对机器人的安装姿态
        pstScanner.InvTransform(frmRobot);

        // 先将全局扫描数据初始化为原局部扫描数据
        m_scanGlobal[i] = m_scanLocal[i];

        // 应用扫描角度限制
        m_scanGlobal[i].ApplyNewScannerAngles(Param.m_AppAngleRange);

        // 在此标记哪些强反光点
        m_scanGlobal[i].MarkReflectivePoints(FeatureCreationParam.m_RefParam.nMinReflectorIntensity);

        // 变换到全局坐标系中
        m_scanGlobal[i].InvTransform(pstScanner);
        m_scanGlobal[i].m_pstScanner = pstScanner;
        // jzz:为局部坐标系点云添加里程位姿
        m_scanLocal[i].m_pstScanner = pstScanner;
    }
}

//
//   将本步数据与给定的步(一般是上一步)进行点云匹配。
//   注意：目前程序仅支持对两个数据步中第一个扫描集之间的匹配!!!!
//
bool CSlamStepData::PointCloudMatch(const CSlamStepData &another, CCorrList *pList, sm_result *pResult)
{
    if (pList == NULL)
    {
        m_bCorrOk = m_scanGlobal[0].PointCloudMatch(another.m_scanGlobal[0], m_frmToLastStep, m_corrList, &m_smResult);
        if (pResult != NULL)
            *pResult = m_smResult;

        CScan Scan1 = m_scanGlobal[0];
        Scan1.InvTransform(m_frmToLastStep);

        int nCountValid = 0;
        float fError = 0;
        for (int i = 0; i < m_corrList.m_nCount; i++)
        {
            if (m_corrList.m_pIdx[i] >= 0)
            {
                CPnt &pt1 = Scan1.m_pPoints[i];
                CPnt &pt2 = another.m_scanGlobal[0].m_pPoints[m_corrList.m_pIdx[i]];
                float d = pt1.DistanceTo(pt2);
                fError += d;
                nCountValid++;
            }
        }
        fError /= nCountValid;
        m_smResult.error2 = fError;
        return m_bCorrOk;
    }
    else
    {
        // CFrame frm;
        // 注意单位转换，mm->m
        CScan sourceScan = m_scanLocal.at(0);
        CScan targetScan = another.m_scanLocal.at(0);
        for (int i = 0; i < sourceScan.m_nCount; i++)
        {
            sourceScan.m_pPoints[i].r /= 1000;
            targetScan.m_pPoints[i].r /= 1000;
//            cout << "r: " << sourceScan.m_pPoints[i].r << endl;
        }

//        return m_scanGlobal[0].PointCloudMatch(another.m_scanGlobal[0], m_frmToLastStep, *pList, pResult);
        // m_scanGlobal与m_scanLocal区别:Global仅把x,y的值转换至了世界坐标系，r值仍是局部坐标系
        // csm计算时，采用r值进行赋值，因此本质上Global与Local的值是一样的都是局部坐标系，且单位是mm
        // 但是在CScan转CsmScan时，计算角度的时候采用（arctan2(y, x))，因此如果用Global数据，此处会出错
        // 故改为Local数据进行csm计算，并且为了得到正确的first_guess，将Local数据的m_pstScanner赋值为Global的m_pstScanner(当前帧的世界坐标系下位姿)
//        return m_scanLocal[0].PointCloudMatch(another.m_scanLocal[0], m_frmToLastStep, *pList, pResult);  // jzz: 采用原始点云（局部）坐标系
        // 由于单位不统一，先转换单位再计算，直接除1000并且覆盖赋值会出错，因为下次迭代another会再次除1000，导致数据出错
        return sourceScan.PointCloudMatch(targetScan, m_frmToLastStep, *pList, pResult);
    }
}

//
//   取得指定的世界散点。
//
CScanPoint *CSlamStepData::GetWorldRawPoint(int nScannerId, int nIdx)
{
    return &m_scanGlobal[nScannerId].m_pPoints[nIdx];
}

//
//   将本步的数据进行指定的坐标变换。
//
void CSlamStepData::Transform(const CFrame &frame)
{
    for (size_t i = 0; i < m_scanGlobal.size(); i++)
        m_scanGlobal[i].Transform(frame);

    m_pstRobot.Transform(frame);
}

//
//   将本步的数据进行指定的坐标逆变换。
//
void CSlamStepData::InvTransform(const CFrame &frame)
{
    for (size_t i = 0; i < m_scanGlobal.size(); i++)
        m_scanGlobal[i].InvTransform(frame);

    m_pstRobot.InvTransform(frame);
}

//
//   将全部点绕机器人的中心点进行旋转。
//
void CSlamStepData::Rotate(float angle)
{
    for (size_t i = 0; i < m_scanGlobal.size(); i++)
        m_scanGlobal[i].RotatePos(angle, m_pstRobot.x, m_pstRobot.y);

    CPnt pt = m_pstRobot;
    m_pstRobot.Rotate(angle, pt);
}

//
//   将全部的点进行平动。
//
void CSlamStepData::Move(float dx, float dy)
{
    for (size_t i = 0; i < m_scanGlobal.size(); i++)
        m_scanGlobal[i].Move(dx, dy);

    m_pstRobot.Move(dx, dy);
}

//
//   启用关于扫描角度范围的约束。
//
void CSlamStepData::ApplyScanAngleRule(int nScannerId, float fMinAngle, float fMaxAngle)
{
    m_scanLocal[nScannerId].ApplyScanAngleRule(fMinAngle, fMaxAngle);
    m_scanGlobal[nScannerId].ApplyScanAngleRule(fMinAngle, fMaxAngle);
}

// 启用关于扫描距离的约束
void CSlamStepData::ApplyScanDistRule(int nScannerId, float fMinDist, float fMaxDist)
{
    m_scanLocal[nScannerId].ApplyScanDistRule(fMinDist, fMaxDist);
    m_scanGlobal[nScannerId].ApplyScanDistRule(fMinDist, fMaxDist);
}

//
//   判断指定的屏幕点是否触本步数据中的某个原始点。
//   返回值：
//     -1 : 没找到
//    其它: 视结果为32位无符号数，D16-D23位(8位)代表激光器编号，(D0-D15)16位为光点的编号。
//
int CSlamStepData::PointHitRawPoint(const CPnt &pt, float fDistGate)
{
    for (size_t i = 0; i < m_scanGlobal.size(); i++)
    {
        for (size_t j = 0; j < m_scanGlobal[i].m_nCount; j++)
        {
            if (m_scanGlobal[i].m_pPoints[j].DistanceTo(pt) < fDistGate)
                return (int)((i << 16) | j);
        }
    }
    return -1;
}

//
//   判断指定的屏幕点是否触本步数据中的机器人位姿。
//
int CSlamStepData::PointHitPose(const CPnt& pt, float fDistGate)
{
    if (fDistGate < SCANNER_RADIUS)
        fDistGate = SCANNER_RADIUS;

    return (m_pstRobot.DistanceTo(pt) < fDistGate);
}

//
//   对本步中的指定传感器扫描数据进行扫描匹配(用于标定同一机器人使用的多个激光器的相对安装姿态)。
//
bool CSlamStepData::MatchInternalScans(int nScanId1, int nScanId2, CPosture &result)
{
    // 激光扫描器编号合法性验证
    if (nScanId1 >= m_scanGlobal.size() || nScanId2 >= m_scanGlobal.size())
        return false;

    CCsmScan scan1(m_scanGlobal[nScanId1]);
    CCsmScan scan2(m_scanGlobal[nScanId2]);
    CCsmMatcher Matcher;

    sm_result sr;
    if (Matcher.Match(&scan1, &scan2, sr))
    {
        result = sr.pstDiff;
        return true;
    }
    else
        return false;
}

//
//   将本步与另一个给定的数据步进行匹配。
//
bool CSlamStepData::Match(const CSlamStepData &another, sm_result &result)
{
    // jzz: 注意单位转换，mm->m
    CScan targetScan = m_scanLocal.at(0);
    CScan sourceScan = another.m_scanLocal.at(0);
    for (int i = 0; i < sourceScan.m_nCount; i++)
    {
        sourceScan.m_pPoints[i].r /= 1000;
        targetScan.m_pPoints[i].r /= 1000;
//            cout << "r: " << sourceScan.m_pPoints[i].r << endl;
    }

    // 目前，匹配仅限于对两个数据步中第一个激光器数据之间的匹配
//    CCsmScan scan1(m_scanGlobal[0]);
//    CCsmScan scan2(another.m_scanGlobal[0]);
    CCsmScan scan1(targetScan);
    CCsmScan scan2(sourceScan);
    CCsmMatcher Matcher;
    // jzz: 设置质量评估参数阈值
    Matcher.setQualityThreshold(0.01, 0.015, 0.35, 0.35, 10, 0.3, 0.3);

    // 开始匹配
    if (Matcher.Match(&scan1, &scan2, result))
        return true;
    else
        return false;
}

//
//   将本步坐标变换到与给定的数据步相匹配。
//
bool CSlamStepData::AlignTo(const CSlamStepData &another, CPosture *ppstMove)
{
    sm_result result;
    if (PointCloudMatch(another, &m_corrList, &result))
    {
        // 根据需要，返回姿态变换值
        if (ppstMove != NULL)
            *ppstMove = result.pstMove;

        InvTransform(result.pstMove);
        return true;
    }
    else
        return false;
}

//
//   将本步坐标变换，使机器人位姿等于给定的位姿。
//
bool CSlamStepData::TransformToRobotPose(const CPosture& pstTarget)
{
    CFrame frm(m_pstRobot);
    float cx = frm.x;
    float cy = frm.y;
    float ang = frm.fThita;

    CPosture pst = pstTarget;
    pst.Transform(frm);

    // 下面将此姿态变化量转换到世界坐标系内
    float x = pst.x;
    float y = pst.y;
    float theta = pst.fThita;

    Rotate(theta);
    float dx = x * cos(ang) - y * sin(ang);
    float dy = x * sin(ang) + y * cos(ang);
    Move(dx, dy);

    return true;
}

//
//   施加虚拟环境。
//
void CSlamStepData::ApplyVirtObjects(const CLiveObjects &objs, int stepId)
{
    for (size_t i = 0; i < m_scanGlobal.size(); i++)
    {
        m_scanGlobal[i].ApplyVirtObjects(objs, stepId);

        // 同时还应调整m_scanLocal内的数据
        for (int j = 0; j < m_scanGlobal[i].m_nCount; j++)
        {
            CScanPoint &spLocal = m_scanLocal[i].m_pPoints[j];
            CScanPoint &spGlobal = m_scanGlobal[i].m_pPoints[j];
            spLocal.r = spGlobal.r;
            spLocal.UpdateCartisian();
            spLocal.x /= 1000;
            spLocal.y /= 1000;
        }
    }
}

#ifdef _MFC_VER

void CSlamStepData::Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF clrRawPoint, COLORREF clrHighLightRawPoint, bool bShowPose,
                         COLORREF clrPose, bool bShowFeature, COLORREF clrFeature)
{
    for (size_t i = 0; i < m_scanGlobal.size(); i++)
    {
        if (i == 1)
            clrRawPoint = RGB(0, 200, 200);

        // 绘制原始点云
        m_scanGlobal[i].Plot(ScrnRef, pDC, clrRawPoint, clrHighLightRawPoint, bShowPose);
    }
}

#elif defined QT_VERSION

void CSlamStepData::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor clrRawPoint, QColor clrHighLightRawPoint, bool bShowPose,
                         QColor clrPose, bool bShowFeature, QColor clrFeature)
{
	//lishen
    if(!m_display)
        return;
    for (size_t i = 0; i < m_scanGlobal.size(); i++)
    {
	//lishen ????
        if(i==0)
        {
            if (i == 1)
                clrRawPoint = QColor(0, 200, 200);

        // 绘制原始点云
        m_scanGlobal[i].Plot(ScrnRef, pPainter, clrRawPoint, clrHighLightRawPoint, false, clrPose);

            // 画出机器人姿态
            if (bShowPose)
                m_pstRobot.Draw(ScrnRef, pPainter, clrPose, clrPose, 40, 150);
        }
    }
}

//
//   显示该步中的高亮点。
//
void CSlamStepData::PlotHighIntensPoints(CScreenReference &ScrnRef, QPainter *pPainter, QColor clr)
{
//    if(!m_display)
//        return;
    for (int i = 0; i < (int)m_scanGlobal.size(); i++)
    {
        CScan &scan = m_scanGlobal[i];
        for (int j = 0; j < scan.m_nCount; j++)
        {
            CScanPoint &sp = scan.m_pPoints[j];
            if (sp.m_bHighReflective)
                sp.Draw(ScrnRef, pPainter, clr, 1, 0, 20);
        }
    }
}

#endif
