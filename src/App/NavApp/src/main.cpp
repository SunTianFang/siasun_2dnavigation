#include <thread>
#include <iostream>
#include "blackboxhelper.hpp"
#include "ShareMem.h"
#include "RoboLocClnt.h"
#include "SensorFamily.h"
#include "RawMap.h"
#include "AutoOutPutBlackBox.h"
#include <sys/resource.h>
#include "systemInfo.h"
#include "RoboManager.h"
#include "LocalizeFactory.h"
#include "ParameterObject.h"
#include "Project.h"
#include "PLS.h"
#include <signal.h>
#include <execinfo.h>
#include "Config.h"
#include "HttpCommunicationGlobalData.h"
#include "webserver.h"



//附加东西
bool readJffFormat = false;

#ifdef USE_BLACK_BOX
// 定义日志文件
CBlackBox NetBox;
CBlackBox LaserBox;
CBlackBox NavBox;
CBlackBox EventBox;
CBlackBox CanBox;
CBlackBox CustomBox;
CBlackBox LocBox;
CBlackBox MsgBox;
// 定义共享内存
cShareMem gShareMem;
void* m_pSharedMemoryBaseAdd;
#endif

// initialize the process.
bool Initialize();

// save the log files.
bool SaveFile();

// exit the process.
void ExitCore(int signo);

// linux core dump file.
bool dumpSetting();

void SigSegv_handler(int signal);

void SetSigSegv();

int main(int argc, char **argv)
{

    //dq 10-19
    SetSigSegv();

    ApiManager api;
    api.config();

//    Sleep(15000);
    if(!Initialize()){
        return -100;

    }

#ifdef USE_BLACK_BOX
    FILE_BlackBox(MsgBox, "Robot localization initialize OK!");
    FILE_BlackBox(MsgBox, "Software Version:", VER_IN_BLACKBOX);
    FILE_BlackBox(MsgBox, "Release Date:", DATE_OF_RELEASE);
    FILE_BlackBox(MsgBox, "Release Time:", TIME_OF_RELEASE);
#endif
    std::cout << "Robot localization initialize OK!" << std::endl;
    std::cout << "Software Version:" << VER_IN_BLACKBOX << std::endl;
    std::cout << "Release Date:" << DATE_OF_RELEASE << std::endl;
    std::cout << "Release Time:" << TIME_OF_RELEASE << std::endl;
    //signal(SIGINT, ExitCore);

    // main thread
    std::thread RoboManThrd(std::bind(&robo::CRoboManager::SupportRoutine,RoboManagerSingleton::GetInstance()));
    auto pParameterObject = ParameterObjectSingleton::GetInstance();
    bool useSoftPls = false;
    pParameterObject->GetParameterValue("Diagnosis_UseSoftPls", useSoftPls);
    if(useSoftPls)
    {
        std::thread PLSThrd(std::bind(&pls::CPlsManager::SupportRoutine,PlsManagerSingleton::GetInstance()));
        PLSThrd.join();
    }
    RoboManThrd.join();

    ExitCore(0 /*SIGINT*/);
    return 0;
}

// initialize the process.
bool Initialize()
{
    // 初始化参数列表
    auto pParameterObject = ParameterObjectSingleton::GetInstance();
    auto pRoboMan = RoboManagerSingleton::GetInstance();
    pParameterObject->InitializeParameters();

    // 是否进入http通信模式
    bool DiagnosisHttpServerFlag = false;
    pParameterObject->GetParameterValue("Diagnosis_HttpServerFlag", DiagnosisHttpServerFlag);
    if(DiagnosisHttpServerFlag){
        HTTP_SERVER_OBJ.start("8101", "", "");
    }
    // 是否进入仿真模式
    bool simulate_ = false;
    pParameterObject->GetParameterValue("Simulate", simulate_);
    if(simulate_){
#ifdef USE_BLACK_BOX
        FILE_BlackBox(MsgBox, "Enter the simulate mode!");
#endif
        std::cout << "Enter the simulate mode!" << std::endl;
    }
    else {
#ifdef USE_BLACK_BOX
        FILE_BlackBox(MsgBox, "Enter the normal mode!");
#endif
        std::cout << "Enter the normal mode!" << std::endl;
    }
#ifdef USE_BLACK_BOX
    m_pSharedMemoryBaseAdd = gShareMem.CreatShareMem(FTOK_KEY_PATH,SHARE_MEM_MAX_SIZE);
    CBlackBox::CreateReserveData(m_pSharedMemoryBaseAdd, RESERVE_DATA_ADDRESS_OFFSET);
    VERIFY(EventBox.Create(_T("Event"),m_pSharedMemoryBaseAdd,EVEN_BASE_ADDRESS_OFFSET, 512000));
    VERIFY(CanBox.Create(_T("Can"),m_pSharedMemoryBaseAdd, CAN_BASE_ADDRESS_OFFSET,512000));
    VERIFY(NavBox.Create(_T("Nav"),m_pSharedMemoryBaseAdd, NAV_BASE_ADDRESS_OFFSET,512000));
    VERIFY(LaserBox.Create(_T("Laser"), m_pSharedMemoryBaseAdd,LASER_BASE_ADDRESS_OFFSET,512000));
    VERIFY(CustomBox.Create(_T("Custom"), m_pSharedMemoryBaseAdd,CUSTOM_BASE_ADDRESS_OFFSET,512000));
    VERIFY(NetBox.Create(_T("Net"), m_pSharedMemoryBaseAdd,NET_BASE_ADDRESS_OFFSET,512000));
    VERIFY(LocBox.Create(_T("Loc"), m_pSharedMemoryBaseAdd,LOC_BASE_ADDRESS_OFFSET,(512000*10)));
    VERIFY(MsgBox.Create(_T("Msg"), m_pSharedMemoryBaseAdd,MSG_BASE_ADDRESS_OFFSET,512000));
#endif
    auto pBlackBox = AutoBlackBoxSingleton::GetInstance();
    pBlackBox->Start();
    // 定义定位系统通信通道
    auto pRoboClnt = RoboClntSingleton::GetInstance(_T(""), RL_SERVER_UDP_PORT, FALSE, TRUE);
    if(!pRoboClnt){
        return false;
    }
    // Create the udp communicate channel.
    if(!pRoboClnt->CreateUdpCom())
    {
#ifdef USE_BLACK_BOX
        FILE_BlackBox(MsgBox, "Create the udp communicate fail!");
#endif
        std::cout << "Create the udp communicate fail!" << std::endl;
        return false;
    }
    bool use_dump = false;
    pParameterObject->GetParameterValue("Diagnosis_UseCoreDump", use_dump);
    if(use_dump){
        dumpSetting();
    }
    auto pFamily = SensorFamilySingleton::GetInstance();
    if(!simulate_ && !pFamily->Initialize()) {
#ifdef USE_BLACK_BOX
        FILE_BlackBox(MsgBox, "Create the sensor family fail!");
#endif
        std::cout << "Create the sensor family fail!" << std::endl;
        return false;
    }
    if(!pRoboMan->Initialize()) {
#ifdef USE_BLACK_BOX
        FILE_BlackBox(MsgBox, "The robo manager initialize failed!");
#endif
        std::cout << "The robo manager initialize failed!" << std::endl;
        return false;
    }
#if 0
    SystemInfo sysinfo(12);
    sysinfo.autoRun(1000);
#endif
    return true;
}

// save the log files.
bool SaveFile()
{
#ifdef USE_BLACK_BOX
    EventBox.Save(_T(LOG_FILE_PATH"Event"));
    CanBox.Save(_T(LOG_FILE_PATH"Can"));
    NavBox.Save(_T(LOG_FILE_PATH"Nav"));
    LaserBox.Save(_T(LOG_FILE_PATH"Laser"));
    CustomBox.Save(_T(LOG_FILE_PATH"Custom"));
    NetBox.Save(_T(LOG_FILE_PATH"Net"));
    LocBox.Save(_T(LOG_FILE_PATH"Loc"));
    MsgBox.Save(_T(LOG_FILE_PATH"Msg"));
#endif
    return true;
}

// exit the process.
void ExitCore(int signo)
{
    auto pFamily = SensorFamilySingleton::GetInstance();
    pFamily->Stop();
    auto pRoboClnt = RoboClntSingleton::GetInstance();
    if(pRoboClnt){
         pRoboClnt->Stop();
         RoboClntSingleton::DesInstance();
    }

    LocalizeFactorySingleton::DesInstance();

    auto pBlackBox = AutoBlackBoxSingleton::GetInstance();
    pBlackBox->Stop();

    //SlamProcSingleton::DesInstance();
    RawMapSingleton::DesInstance();

    auto pRoboMan = RoboManagerSingleton::GetInstance();
    pRoboMan->Stop();
    auto pParameterObject = ParameterObjectSingleton::GetInstance();
    bool useSoftPls = false;
    pParameterObject->GetParameterValue("Diagnosis_UseSoftPls", useSoftPls);
    if(useSoftPls)
    {
        auto pPlsMan = PlsManagerSingleton::GetInstance();
        pPlsMan->Stop();
    }
    //SaveFile();
    std::cout << "ExitCore: signo = " << signo << std::endl;
}

// linux core dump file.
bool dumpSetting()
{
    struct rlimit rlmt;
    if (getrlimit(RLIMIT_CORE, &rlmt) == -1) {
        return FALSE;
    }
    rlmt.rlim_cur = (rlim_t)(1024*1024*4000);
    rlmt.rlim_max  = (rlim_t)(1024*1024*4000);
    if (setrlimit(RLIMIT_CORE, &rlmt) == -1) {
        return FALSE;
    }
    if (getrlimit(RLIMIT_CORE, &rlmt) == -1) {
        return FALSE;
    }
    return true;
}
#define BACKTRACE_BUF_SIZE          50

void SigSegv_handler(int signal)
{
    int j, nptrs;
    void *buffer[BACKTRACE_BUF_SIZE];
    char **strings;

#if 1
    nptrs = backtrace(buffer, BACKTRACE_BUF_SIZE);
#else
    nptrs = backtrace_arm(buffer, BACKTRACE_BUF_SIZE);
#endif

    printf("signal_test exit: backtrace() returned %d addresses, sig:%d\n", nptrs, signal);

    switch (signal) {
    case SIGFPE: // 8:
        printf("The signal type: Floating point exception!\n");
        break;
    case SIGILL: // 4:
        printf("The signal type: Illegal instruction!\n");
        break;
    case SIGSEGV: // 11:
        printf("The signal type: Segmentation fault!\n");
        break;
    case SIGBUS: // 7:
        printf("The signal type: Bus error!\n");
        break;
    case SIGABRT: // 6:
        printf("The signal type: Aborted!\n");
        break;
    case SIGTRAP: // 5:
        printf("The signal type: Trace/breakpoint trap!\n");
        break;
    case SIGSYS: // 31:
        printf("The signal type: Bad system call!\n");
        break;
    case SIGTERM: // 15:
        printf("The signal type: Terminated!\n");
        break;
    case SIGINT: // 2:
        printf("The signal type: Interrupt!\n");
        break;
    case SIGQUIT: // 3:
        printf("The signal type: Quit!\n");
        break;
    case SIGKILL: // 9:
        printf("The signal type: Killed!\n");
        break;
    case SIGHUP: // 1:
        printf("The signal type: Hangup!\n");
        break;
    case SIGALRM: // 14:
        printf("The signal type: Alarm clock!\n");
        break;
    case SIGVTALRM: // 26:
        printf("The signal type: Virtual timer expired!\n");
        break;
    case SIGPROF: // 27:
        printf("The signal type: Profiling timer expired!\n");
        break;
    case SIGIO: // 29:
        printf("The signal type: I/O possible!\n");
        break;
    case SIGPIPE: // 13:
        printf("The signal type: Broken pipe!\n");
        break;
    case SIGXCPU: // 24:
        printf("The signal type: CPU time limit exceeded!\n");
        break;
    case SIGXFSZ: // 25:
        printf("The signal type: File size limit exceeded!\n");
        break;
    case SIGUSR1: // 10:
        printf("The signal type: User defined signal 1!\n");
        break;
    case SIGUSR2: // 12:
        printf("The signal type: User defined signal 2!\n");
        break;
    default:
        printf("The signal type: Unkown signal:%d!\n", signal);
        break;
    }

    /* The call backtrace_symbols_fd(buffer, nptrs, STDOUT_FILENO)
    would produce similar output to the following: */

    strings = backtrace_symbols(buffer, nptrs);
    if (strings == NULL) {
        printf("backtrace_symbols");
        exit(EXIT_FAILURE);
    }

    for (j = 0; j < nptrs; j++) {
        printf("%s\n", strings[j]);
    }

    free(strings);
    exit(-1);
}
void SetSigSegv()
{
    signal(SIGSEGV, SigSegv_handler);
    signal(SIGILL, SigSegv_handler);
    signal(SIGBUS, SigSegv_handler);
    signal(SIGFPE, SigSegv_handler);
    signal(SIGABRT, SigSegv_handler);
    signal(SIGTRAP, SigSegv_handler);
    signal(SIGSYS, SigSegv_handler);
    signal(SIGTERM, SigSegv_handler);
    signal(SIGINT, SigSegv_handler);
    signal(SIGQUIT, SigSegv_handler);
    signal(SIGKILL, SigSegv_handler);
    signal(SIGHUP, SigSegv_handler);
    signal(SIGALRM, SigSegv_handler);
    signal(SIGVTALRM, SigSegv_handler);
    signal(SIGPROF, SigSegv_handler);
    signal(SIGIO, SigSegv_handler);
    signal(SIGPIPE, SigSegv_handler);
    signal(SIGXCPU, SigSegv_handler);
    signal(SIGXFSZ, SigSegv_handler);
    signal(SIGUSR1, SigSegv_handler);
    signal(SIGUSR2, SigSegv_handler);
}
