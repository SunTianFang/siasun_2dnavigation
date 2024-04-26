//
//   The interface of class "CRoboManager".
//

#pragma once

#include <stdio.h>
#include <mutex>
#include <atomic>
#include "MagicSingleton.h"


//by dq muban
#include "LineElement.h"

namespace robo {

class CRoboManager
{
private:
    std::mutex robo_mtx;
    std::atomic_uchar  m_aWorkMode;
    std::atomic_bool   m_aHaltThread;
    unsigned char       m_aPadMode;     // pad设置的工作模式
    unsigned char       m_aLastPadMode;     //By yu.上次工作模式
    unsigned char       m_aLastRoboMode;     //
    unsigned char       m_aLastMode;
private:
    CRoboManager();
    ~CRoboManager();

    friend MagicSingleton<CRoboManager>;

private:
    // 空闲
    bool StandBy();

    // 执行建图过程
    bool DoMapping();

    // 执行定位过程
    bool DoLocalization();

    // 执行扫描过程
    bool DoScan();

    // 切换定位系统工作模式
    bool ChangeMode();

public:
    bool Initialize();

    void Clear();

    void SupportRoutine();

    bool Stop();

    // pad设置工作模式
    void HandlePadMsg ( int msg );

    // pad选择是否保存地图
    void HandleSaveMapMsg ( int msg ,vector<double> dparams);

    // 获取当前工作模式
    int GetCurWorkMode ( void );

    // 获取结束建图状态
    int GetAutoMappingState ( void );

    //by DQ pad添加模板区域
    bool GetStaticObjects(int count, vector<vector<float> > points, vector<CPosture> &psts);
    bool ChangeObjects(int8_t *iparams, double *dparams);
    bool GetPlans(int count, vector<vector<float> > plans, vector<int> type);
    bool ChangePlan(int8_t *iparams,double*dparams);
    bool GetMap(char* *sparams);
private:
    // 进入空闲模式
    bool EnterStandByMode();

    // 进入建图模式
    bool EnterMappingMode();

    // 进入定位模式
    bool EnterLocalizationMode();

    // 进入扫描模式
    bool EnterScanMode();

    // 停止当前的动作
    bool StopCurAction();

    // 停止建图过程
    bool StopMapping();

    // 停止定位过程
    bool StopLocalization();

    // 停止扫描过程
    bool StopScan();

    // 开始建图过程
    bool StartMapping();

    // 开始定位过程
    bool StartLocalization();

    // 开始扫描过程
    bool StartScan();

    // by lishen
    bool EnterAutoMappingMode();

    // by lishen  开始建图
    bool StartAutoMapping();

    // by lishen 停止建图过程
    bool StopAutoMapping();

    // by lishen 进入标定模式
    bool EnterCalibrationMode();

    // by lishen  开始标定
    bool StartCalibration();

    // by lishen 停止标定
    bool StopCalibration();

};

} // namespace robo

using RoboManagerSingleton = MagicSingleton<robo::CRoboManager>;

