//
//   The interface of class "CRoboManager".
//
#include <unistd.h>
#include "RoboManager.h"
#include "RoboLocClnt.h"
#include "LaserMapping.h"
#include "LaserAutoMapping.h"
#include "LocalizeFactory.h"
#include "RawMap.h"
#include "BaseOdometry.h"
#include "CanMan.h"
#include "ParameterObject.h"
#include "HttpCommunicationGlobalData.h"
#include "LaserAutoMapping.h"
#include "Calibrate.h"


#include "RecTopvisionImage.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define ROBO_MANAGER_CTRL_CYCLE     45  //30ms

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CRoboManager".
namespace robo {

CRoboManager::CRoboManager()
{
    m_aWorkMode = RLP_MODE_STANDBY;
    m_aHaltThread = false;

    m_aPadMode      = RLP_MODE_LOCALIZATION;    // 刚上电默认为定位模式 add 2022.06.06
    m_aLastPadMode    = RLP_MODE_LOCALIZATION;
    m_aLastRoboMode   = RLP_MODE_STANDBY;
    m_aLastMode = RLP_MODE_LOCALIZATION;
}

CRoboManager::~CRoboManager()
{
    Clear();
    m_aHaltThread = true;
}

void CRoboManager::Clear()
{

}

bool CRoboManager::Initialize()
{
    bool bRet = false;

    // 是否进入仿真模式
    bool simulate_ = false;
    auto pParameterObject = ParameterObjectSingleton::GetInstance();
    pParameterObject->GetParameterValue("Simulate", simulate_);

//    // 初始化数据集仿真定位
//    if(simulate_){
//        auto pDataSetLocalize = DataSetLocalizeSingleton::GetInstance();
//        bRet = pDataSetLocalize->Initialize();
//        if(!bRet){
//            return false;
//        }
//    }
//    else {
//    }

    // 初始化定位模块
    auto pLocalize = LocalizeFactorySingleton::GetInstance();

    bRet = pLocalize->Initialize(simulate_);
    if(!bRet){
        return false;
    }

    if(!simulate_){
        //初始化CAN设备
        bool can_activation = false;
        auto pCanManager = CanManagerSingleton::GetInstance();
        if(pParameterObject->GetParameterValue("CAN_Activation", can_activation)){
            if(can_activation){
                bRet = pCanManager->Initialize();
                if(!bRet){
                    return false;
                }
            }
        }
        auto pBaseOdom = BaseOdomSingleton::GetInstance();
        bRet = pBaseOdom->Initialize();
    }
    return bRet;
}

void CRoboManager::SupportRoutine()
{
#if 0
    cpu_set_t maskCom;
    CPU_ZERO(&maskCom);
    CPU_SET(3, &maskCom);
    pthread_setaffinity_np(pthread_self(), sizeof(maskCom), &maskCom);
#endif
     LCMTask::GetLcmInstance().LCMInitSubscrible(this);


    while(1)
    {
        // 模式切换
        ChangeMode();

        // 更新里程计
        auto pOdometry = BaseOdomSingleton::GetInstance();
        pOdometry->UpdateEstimateOdom();

        switch(m_aWorkMode.load())
        {
        case RLP_MODE_POWERON:
            break;

        case RLP_MODE_STANDBY:
            StandBy();
            break;

        case RLP_MODE_MAPPING:
            DoMapping();
            break;

        case RLP_MODE_LOCALIZATION:
            DoLocalization();
            break;

        case RLP_MODE_SCAN:
            DoScan();
            break;
            //lishen
        case RLP_MODE_AUTOMAPPING:
            break;

        case RLP_MODE_CALIBRATION:
            break;

        default:
            break;
        }

        if(m_aHaltThread.load()){
            return;
        }
        usleep(ROBO_MANAGER_CTRL_CYCLE*1000);
    }
}

bool CRoboManager::Stop()
{
    StopMapping();
    StopLocalization();
    StopScan();
    m_aHaltThread = true;
    return true;
}

// 空闲
bool CRoboManager::StandBy()
{

    return true;
}

// 执行建图过程
bool CRoboManager::DoMapping()
{

    return true;
}

// 执行定位过程
bool CRoboManager::DoLocalization()
{

    return true;
}

// 执行扫描过程
bool CRoboManager::DoScan()
{

    return true;
}


 void CRoboManager::HandlePadMsg ( int msg )
 {
     if ( msg == RLP_MODE_AUTOMAPPING )
     {     
         m_aPadMode = RLP_MODE_AUTOMAPPING;
     }

     if ( msg == RLP_MODE_STOPMAPPING )
     {
         m_aPadMode = RLP_MODE_STOPMAPPING;
     }

     if ( msg == RLP_MODE_MAPPING )
     {
         m_aPadMode = RLP_MODE_MAPPING;
     }
     if ( msg == RLP_MODE_CALIBRATION )
     {
         m_aPadMode = RLP_MODE_CALIBRATION;
     }

     // 结束建图时回调
     if ( msg == RLP_MODE_LOCALIZATION )
     {
         m_aPadMode = RLP_MODE_LOCALIZATION;
     }
 }

// by cgd
void CRoboManager::HandleSaveMapMsg ( int msg,vector<double> dparams )
{
    auto pMapping = LaserAutoMappingSingleton::GetInstance();

    switch ( msg )
    {
        case 1:
            pMapping->SaveMap();
            break;

        case 2:
            pMapping->CancelSaveMap();
            break;
        case 3:
        {
            pMapping->RotateMap(dparams[0]);
            pMapping->SaveMap();
        }
            break;
    }
}

// 切换定位系统工作模式
bool CRoboManager::ChangeMode()
{
    bool bRet = false;
    auto pRoboClnt = RoboClntSingleton::GetInstance();
    if ( !pRoboClnt ) {
        return false;
    }

    unsigned char new_mode;
    unsigned char m_aRoboClntMode = pRoboClnt->GetNewWorkMode();

    if(m_aLastRoboMode != m_aRoboClntMode )// 从车体获取模式变化
    {
        new_mode = m_aRoboClntMode;
        m_aLastRoboMode = m_aRoboClntMode;
        std::cout<<"test by yu : RoboMode Change!!!!"<<std::endl;
    }else if(m_aLastPadMode != m_aPadMode)
    {      // 从 pad 获取模式变化
        new_mode = m_aPadMode;
        m_aLastPadMode = m_aPadMode;
        std::cout<<"test by yu : PadMode Change!!!!"<<std::endl;
    }else{
        new_mode = m_aLastMode;
    }

     //从标定软件获取模式变化
     if(GData::getObj().startRecordDxFlag){
         new_mode  = RLP_MODE_MAPPING;
     }
     else if(GData::getObj().finishRecordDxFlag){
         new_mode  = RLP_MODE_LOCALIZATION;
     }

    m_aLastMode = new_mode;

    if ( m_aWorkMode.load() != new_mode )
    {
        // 更改成新模式
        switch(new_mode)
        {
        case RLP_MODE_POWERON:
            m_aWorkMode = RLP_MODE_POWERON;
            bRet = true;
            break;

        case RLP_MODE_STANDBY:
            bRet = EnterStandByMode();
            break;

        case RLP_MODE_MAPPING:
            bRet = EnterMappingMode();
            break;

        case RLP_MODE_LOCALIZATION:
            bRet = EnterLocalizationMode();
            break;

        case RLP_MODE_SCAN:
            bRet = EnterScanMode();
            break;

        case RLP_MODE_AUTOMAPPING:      //lishen    RLP_MODE_SCAN = 5

            bRet = EnterAutoMappingMode();
            break;

        case RLP_MODE_STOPMAPPING:  // RLP_MODE_STOPMAPPING = 6

            if(StopAutoMapping())
                m_aWorkMode = RLP_MODE_STOPMAPPING;
            break;

        case RLP_MODE_CALIBRATION: //lishen
             bRet = EnterCalibrationMode();
            break;

        default:
            break;
        }
    }
    else{
    }

    if(GData::getObj().finishRecordDxFlag){
        GData::getObj().finishRecordDxFlag = false;
    }
    // 模式切换时是否需要清除部分数据?
    pRoboClnt->SetCurWorkMode(m_aWorkMode.load());
    return bRet;
}

// 进入空闲模式
bool CRoboManager::EnterStandByMode()
{
    bool bRet = StopCurAction();
    if(bRet) {
        m_aWorkMode = RLP_MODE_STANDBY;
    }
    return bRet;
}

// 进入建图模式
bool CRoboManager::EnterMappingMode()
{
    bool bStop = StopCurAction();
    bool bStart = StartMapping();
    if(bStop && bStart){
        m_aWorkMode = RLP_MODE_MAPPING;
        return true;
    }
    else {
        return false;
    }
}

// 进入Auto建图模式
bool CRoboManager::EnterAutoMappingMode()
{
    bool bStop = StopCurAction();
    if(bStop)
    {
        bool bStart = StartAutoMapping();
        if( bStart){
            m_aWorkMode = RLP_MODE_AUTOMAPPING;
            return true;
        }
        else {

                std::cout<<"StartLocalization"<<std::endl;
                m_aPadMode = RLP_MODE_LOCALIZATION;
                bool bStart = StartLocalization();
                if( bStart){
                    m_aWorkMode = RLP_MODE_LOCALIZATION;
                    return true;
                }
                else {
                    return false;
                }


        }
    }
    return false;

}

// 进入定位模式
bool CRoboManager::EnterLocalizationMode()
{
    bool bStop = StopCurAction();
    if(bStop)
    {
        bool bStart = StartLocalization();
        if( bStart){
            m_aWorkMode = RLP_MODE_LOCALIZATION;
            return true;
        }
        else {
            return false;
        }
    }
    return false;
}
bool CRoboManager::EnterCalibrationMode()
{
    bool bStop = StopCurAction();
    bool bStart = StartCalibration();

    if(!bStop)
        return false;

    if(bStop && bStart){
        m_aWorkMode = RLP_MODE_CALIBRATION;

        return true;
    }
    else {
        return false;
    }
}

// 进入扫描模式
bool CRoboManager::EnterScanMode()
{
    bool bStop = StopCurAction();
    bool bStart = StartScan();
    if(bStop && bStart){
        m_aWorkMode = RLP_MODE_SCAN;
        return true;
    }
    else {
        return false;
    }
}

// 停止当前的动作
bool CRoboManager::StopCurAction()
{

    bool bRet = false;
    switch(m_aWorkMode.load())
    {
    case RLP_MODE_POWERON:
        bRet = true;
        break;

    case RLP_MODE_STANDBY:
        bRet = true;
        break;

    case RLP_MODE_MAPPING:
        bRet = StopMapping();
        break;

    case RLP_MODE_LOCALIZATION:
        bRet = StopLocalization();
        break;

    case RLP_MODE_SCAN:
        bRet = StopScan();
        break;

    case RLP_MODE_AUTOMAPPING:      // RLP_MODE_AUTOMAPPING = 5
        bRet = StopAutoMapping();
        break;

    case RLP_MODE_STOPMAPPING:      // RLP_MODE_STOPMAPPING = 6
        bRet = true;
        break;

    case RLP_MODE_CALIBRATION:
        bRet = StopCalibration();
        break;

    default:
        break;
    }
    return bRet;
}

// 停止建图过程
bool CRoboManager::StopMapping()
{
    auto pMapping = LaserMappingSingleton::GetInstance();
    bool bRet = pMapping->StopMapping();

    return bRet;
}

// 停止建图过程
bool CRoboManager::StopAutoMapping()
{

    auto pMapping = LaserAutoMappingSingleton::GetInstance();
    bool bRet = pMapping->StopMapping();


   return bRet;
}



// 停止定位过程
bool CRoboManager::StopLocalization()
{
    auto pLocalize = LocalizeFactorySingleton::GetInstance();
    bool bRet = pLocalize->StopLocalize();
    return bRet;
}

// 停止扫描过程
bool CRoboManager::StopScan()
{

    return true;
}

// 开始建图过程
bool CRoboManager::StartMapping()
{
    auto pMapping = LaserMappingSingleton::GetInstance();
    bool bRet = pMapping->StartMapping();
    pMapping->SetWorkMode(RLP_MODE_MAPPING);
    return bRet;
}

// lishen 开始建图过程
bool CRoboManager::StartAutoMapping()
{
    auto pAutoMapping = LaserAutoMappingSingleton::GetInstance();

    // 初始化mapping模块
    bool bRet = pAutoMapping->Initialize();
    if(!bRet){
        return false;
    }

    auto pLocalize = LocalizeFactorySingleton::GetInstance();
    CStampedPos curPos = pLocalize->GetCurPose();


    Pose pos{curPos.x,curPos.y,curPos.fThita};

    pAutoMapping->SetInitPos(pos);

    bRet = pAutoMapping->StartMapping();

    /////////////////////??????
    if(bRet)
    {
        //pAutoMapping->SetWorkMode(RLP_MODE_AUTOMAPPING);
        pLocalize->UnloadMap();
    }

    return bRet;
}
// 开始定位过程
bool CRoboManager::StartLocalization()
{
    auto pLocalize = LocalizeFactorySingleton::GetInstance();
    bool bRet = pLocalize->StartLocalize();

    auto pMapping = LaserMappingSingleton::GetInstance();
    pMapping->SetWorkMode(RLP_MODE_LOCALIZATION);

    // get the localize dataset size
    int dataset_size = RAW_MAP_CAPACITY_LOCAL;
    auto pParameterObject = ParameterObjectSingleton::GetInstance();
    pParameterObject->GetParameterValue("Mapping_LocalizeDataSetSize", dataset_size);

    auto pRawMap = RawMapSingleton::GetInstance();
    pRawMap->Clear();
    pRawMap->SetMaxCount(dataset_size);
    return bRet;
}

bool CRoboManager::StartCalibration()
{
    auto pCalibrate = CalibrateSingleton::GetInstance();
    bool bRet = pCalibrate->StartCalibration();
    if(bRet)
    {
        pCalibrate->SetWorkMode(RLP_MODE_CALIBRATION);
    }

    return bRet;
}
bool CRoboManager::StopCalibration()
{
    //CalibrateSingleton::Get

    auto pCalibrate = CalibrateSingleton::GetInstance();
    bool bRet = pCalibrate->StopCalibration();

    return bRet;
}


bool CRoboManager::GetStaticObjects(int count, vector<vector<float> > points, vector<CPosture> &psts)
{
    auto pLocalize = LocalizeFactorySingleton::GetInstance();
    pLocalize->GetStaticObjects(count, points, psts);
    return true;
}
bool CRoboManager::GetPlans(int count, vector<vector<float> > plans, vector<int> type)
{
    auto pLocalize = LocalizeFactorySingleton::GetInstance();
    pLocalize->GetPlans(count, plans, type);
    return true;
}
bool CRoboManager::GetMap(char* *sparams)
{
    auto pLocalize = LocalizeFactorySingleton::GetInstance();
    pLocalize->UnloadMap();
    string map_name = *sparams;
    pLocalize->LoadMap(map_name);
    return true;
}
// by DQ pad添加模板区域
bool CRoboManager::ChangeObjects(int8_t *iparams,double*dparams)
{
    if(iparams == NULL)
        return false;
    // 模板个数
    int count = iparams[0];
    // 标志位
    int scaler = 0;
    // 楼层数（未启用）
    int floor = 0;
    float f[3];
    // 记录多个模板形状的容器<每个模板的形状（中心为原点）>
    vector<vector<float> > points;
    // 记录多个模板位姿的容器
    vector<CPosture> psts;



   // std::cout<<"\033[31;1m count = "<<count<<"\033[0m"<<std::endl;

    if(count!=0)
    {
        for(int i = 0; i < count; i++)
        {

            // vector<float>point(dparams+scaler, dparams+scaler+iparams[i+1]+1);
            // 形状坐标（x1,y1,x2,y2,x3,y3...）+ 形状（0 || 1 || 2）+ 模板位姿（x,y,theta）+ 层数（未启用）
            vector<float>point;
            for(int j = 0;j<iparams[i+1]+1;j++)
            {
                point.push_back(dparams[scaler+j]);
            }
            f[0] = dparams[scaler+iparams[i+1]+1];
            f[1] = dparams[scaler+iparams[i+1]+2];
            f[2] = (360-dparams[scaler+iparams[i+1]+3])*PI/180;
            floor = dparams[scaler+iparams[i+1]+4];
            CPosture pst(f[0], f[1], f[2]);
            points.push_back(point);
            psts.push_back(pst);
            scaler += iparams[i+1] + 5;
            point.clear();
          //  std::cout<<"\033[31;1m f[0] = "<<f[0]<<"\033[0m"<<std::endl;
        }
    }
    auto pLocalize = LocalizeFactorySingleton::GetInstance();
    if(!pLocalize->ChangeObj(count, points, psts))
        std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    points.clear();
    psts.clear();
    return true;
}

bool CRoboManager::ChangePlan(int8_t *iparams,double*dparams)
{
    if(iparams == NULL)
        return false;
    // 个数
    int plan_num = iparams[0];
    // 楼层数（未启用）
    int floor = 0;
    // 记录模板区域（左上，右下, 楼层）
    vector<vector<float> > plans;
    vector<int> type;
    if(plan_num!=0)
    {
        for(int i = 0; i < plan_num; i++)
        {
            vector<float>plan;
            for(int j = 0; j < 4; j++)
            {
                plan.push_back(dparams[5*i+j]);
            }
            floor = dparams[5*i+4];
            plans.push_back(plan);
            type.push_back(iparams[i+1]);
            plan.clear();
        }
    }
    auto pLocalize = LocalizeFactorySingleton::GetInstance();
    if(!pLocalize->ChangePlan(plan_num, plans, type))
        return false;

    plans.clear();
    return true;
}
// 开始扫描过程
bool CRoboManager::StartScan()
{

    return true;
}

// 获取当前工作模式
int CRoboManager::GetCurWorkMode ( void )
{
    return m_aWorkMode.load();
}

// 获取结束建图状态
int CRoboManager::GetAutoMappingState ( void )
{
    return LaserAutoMappingSingleton::GetInstance()->GetMappingStatus();
}

} // namespace robo

