//
//   The interface of class "CSensorFamily".
//

#include"SensorFamily.h"
#include <fstream>
#include "json/json.h"
#include"Sick581LaserScanner.h"
#include"WJ716LaserScanner.h"
#include "blackboxhelper.hpp"
#include "HttpCommunicationGlobalData.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

#define DAEMON_THREAD_CTRL_CYCLE     1000  //1000ms

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CSensorFamily".
namespace sensor {

CSensorFamily::CSensorFamily()
{
    sensor_family.clear();
    m_aCount = 0;
    m_bStarted = false;
}

CSensorFamily::~CSensorFamily()
{
    Clear();
}

void CSensorFamily::Clear()
{
    for (unsigned int i = 0; i < sensor_family.size(); i++)
    {
        if (sensor_family[i] != NULL) {
            delete sensor_family[i];
            sensor_family[i] = NULL;
        }
    }
    sensor_family.clear();
}

bool CSensorFamily::LoadLaserParam()
{
    bool bRet = true;

    std::ifstream FileLaserParm(WORK_PATH"LaserParm.json");
    Json::Reader Jreader;
    Json::Value LaserParmRoot;

    if(!FileLaserParm)
    {
        return false;
    }

    if(Jreader.parse(FileLaserParm, LaserParmRoot))
    {
        sensor_family.clear();
        if (!LaserParmRoot["laser"].isNull())
        {
            m_aCount = LaserParmRoot["laser"].size();
        }

        GData::getObj().laser_num = (int)m_aCount;

        for(int i = 0; i < m_aCount.load(); i++)
        {
            CLaserScannerParam* parm = new CLaserScannerParam();
            CSensorData* data = new CSensorData();
            if(parm == NULL || data == NULL) {
                bRet = false;
                continue;
            }

            parm->LaserId = i;

            if (!LaserParmRoot["laser"][i]["State"].isNull()) {
                parm->state = LaserParmRoot["laser"][i]["State"].asBool();
            }

            if (!LaserParmRoot["laser"][i]["IP"].isNull()) {
                parm->strIP = LaserParmRoot["laser"][i]["IP"].asString();
            }

            if (!LaserParmRoot["laser"][i]["hostIP"].isNull()) {
                parm->hostIP = LaserParmRoot["laser"][i]["hostIP"].asString();
            }
            if (!LaserParmRoot["laser"][i]["LaserProductor"].isNull()) {
                parm->LaserProductor = LaserParmRoot["laser"][i]["LaserProductor"].asInt();
            }
            //反光板识别强度门限  默认值(倍加福R2000:700；Hokuyo：7000；SickNano：254)
            if (!LaserParmRoot["laser"][i]["criteritonThreshold"].isNull()) {
                parm->criteritonThreshold = LaserParmRoot["laser"][i]["criteritonThreshold"].asInt();
            }
            if (!LaserParmRoot["laser"][i]["StartAngle"].isNull()) {
                parm->m_fStartAngle = LaserParmRoot["laser"][i]["StartAngle"].asDouble()/180.0*PI;
            }

            if (!LaserParmRoot["laser"][i]["EndAngle"].isNull()) {
                parm->m_fEndAngle = LaserParmRoot["laser"][i]["EndAngle"].asDouble()/180.0*PI;
            }

            if (!LaserParmRoot["laser"][i]["LaserCount"].isNull()) {
                parm->m_nLineCount = LaserParmRoot["laser"][i]["LaserCount"].asInt();
            }
            parm->Set(parm->m_fStartAngle, parm->m_fEndAngle, parm->m_nLineCount);

            if (!LaserParmRoot["laser"][i]["x"].isNull()) {
                parm->m_pst.x = LaserParmRoot["laser"][i]["x"].asDouble() /** 1000*/;
            }

            if (!LaserParmRoot["laser"][i]["y"].isNull()) {
                parm->m_pst.y = LaserParmRoot["laser"][i]["y"].asDouble() /** 1000*/;
            }

            if (!LaserParmRoot["laser"][i]["thita"].isNull()) {
                parm->m_pst.fThita = LaserParmRoot["laser"][i]["thita"].asDouble() / 180.0 * PI;
            }

            parm->m_fRefViewAngle = CAngle::NormAngle(parm->m_fEndAngle - parm->m_fStartAngle);

            if (!LaserParmRoot["laser"][i]["MaxRange"].isNull()) {
                parm->m_fMaxRange = LaserParmRoot["laser"][i]["MaxRange"].asDouble();
            }

            if (!LaserParmRoot["laser"][i]["MinRange"].isNull()) {
                parm->m_fMinRange = LaserParmRoot["laser"][i]["MinRange"].asDouble();
            }

            if(!LaserParmRoot["laser"][i]["LaserUnInverted"].isNull()){
                 parm->g_bLaserUnInverted = LaserParmRoot["laser"][i]["LaserUnInverted"].asBool();
             } else {
                 // 处理空值的逻辑，例如设置一个默认值
                 parm->g_bLaserUnInverted = false; // 默认值为 false
             }

            int nRangeCount = 0;
            float fRange[2] = {0.0};
            if (!LaserParmRoot["laser"][i]["VisualRange"].isNull()) {
                nRangeCount = LaserParmRoot["laser"][i]["VisualRange"].size();
            }

            for(int j = 0; j < nRangeCount; j++)
            {
                if (!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].isNull()) {
                    fRange[0] = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].asDouble() / 180 * PI;
                }
                else {
                    fRange[0] = 0.0;
                }

                if (!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].isNull()) {
                    fRange[1] = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].asDouble() / 180 * PI;
                }
                else {
                    fRange[1] = 0.0;
                }
                CDataRange range(fRange[0], fRange[1]);
                parm->m_AppAngleRange.push_back(range);
            }


            if(i == 0){
                GData::getObj().front_laser_state = int(parm->state);
                GData::getObj().front_x = float(parm->m_pst.x);
                GData::getObj().front_y = float(parm->m_pst.y);
                GData::getObj().front_theta = float(parm->m_pst.fThita);
                GData::getObj().front_laser_count = int(parm->m_nLineCount);
                GData::getObj().front_start_angle = float(parm->m_fStartAngle);
                GData::getObj().front_end_angle = float(parm->m_fEndAngle);
                GData::getObj().front_visual_range_size = int(nRangeCount);
                GData::getObj().front_visual_angle_start = float(fRange[0]);
                GData::getObj().front_visual_angle_end = float(fRange[1]);
            }
            else {
                GData::getObj().end_laser_state = int(parm->state);
                GData::getObj().end_x = float(parm->m_pst.x);
                GData::getObj().end_y = float(parm->m_pst.y);
                GData::getObj().end_theta = float(parm->m_pst.fThita);
                GData::getObj().end_laser_count = int(parm->m_nLineCount);
                GData::getObj().end_start_angle = float(parm->m_fStartAngle);
                GData::getObj().end_end_angle = float(parm->m_fEndAngle);
                GData::getObj().end_visual_range_size = int(nRangeCount);
                GData::getObj().end_visual_angle_start = float(fRange[0]);
                GData::getObj().end_visual_angle_end = float(fRange[1]);
            }


            data->parm = parm;
            data->scanner = NULL;
            sensor_family.push_back(data);
        }
    }
    FileLaserParm.close();

    return bRet;
}

// the daemon thread
void CSensorFamily::SupportRoutineProxy()
{
#ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "-- SupportRoutineProxy is Live !!!");
#endif
    // 通信超时后重联
    if(sensor_family.empty()){
        return;
    }
    for (unsigned int i = 0; i < sensor_family.size(); i++){
        if (sensor_family[i] == NULL || sensor_family[i]->scanner == NULL || sensor_family[i]->parm == NULL
                || !sensor_family[i]->parm->state){
            continue;
        }
        unsigned long long time_now = GetTickCount();
        unsigned long long time_raw = sensor_family[i]->scanner->GetRawTimeStamp();
        unsigned long long connect_time = sensor_family[i]->scanner->m_nConnectTime;
        std::string str_ip = sensor_family[i]->parm->strIP;
        std::string host_ip = sensor_family[i]->parm->hostIP;
        int laser_id = sensor_family[i]->parm->LaserId;
        int ping_ack = 0;
        if(labs(static_cast<long int>(time_now - time_raw)) > SCANNER_RECONNECT_TIME
                && labs(static_cast<long int>(time_now - connect_time)) > SCANNER_CONNECT_SPAN){
            // ping ok
            ping_ack = m_Ping.Ping(str_ip);

#ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "-->the scanner prepare to reconnect,i=", i, ",ping_ack=", ping_ack);
#endif

            if(ping_ack == 1){
                // stop the scanner thread
                sensor_family[i]->scanner->Stop();

#ifdef USE_BLACK_BOX
                FILE_BlackBox(LocBox, "-->the scanner call the destructor ,i=", i);
#endif
                usleep(3000 * 1000);

                // start the scanner thread
                bool bRet = sensor_family[i]->scanner->Start(str_ip.c_str(), host_ip.c_str(), laser_id);
                usleep(1000 * 1000);

#ifdef USE_BLACK_BOX
                FILE_BlackBox(LocBox, "-->the scanner reconnect,i=", i ,",ping_ack=", ping_ack, ",Ret=", (int)bRet);
#endif
            }
            else {
#ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "-->the scanner prepare to reconnect,i=", i, ",ping_ack=", ping_ack);
#endif
            }
        }else
        {
#ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "-->the scanner ,i=", i, ",time_now=", static_cast<double>(time_now),",time_raw=",static_cast<double>(time_raw),",connect_time=",static_cast<double>(connect_time));
#endif
        }
    }
}

bool CSensorFamily::Initialize()
{
    bool bRet = true;

    if(!LoadLaserParam()) {
        return false;
    }

    if(sensor_family.empty()) {
        return false;
    }

    for(int i = 0; i < sensor_family.size(); i++)
    {
        if(sensor_family[i]->parm == NULL) {
            bRet = false;
            continue;
        }

        switch (sensor_family[i]->parm->LaserProductor)
        {
        case PEPPERL_FUCHS:
        {
            cPfR2000LaserScanner* pScanner_Pf = new cPfR2000LaserScanner(sensor_family[i]->parm->m_nLineCount,
                                                                         sensor_family[i]->parm->m_fStartAngle,
                                                                         sensor_family[i]->parm->m_fEndAngle);
            if(pScanner_Pf == NULL) {
                bRet = false;
                continue;
            }
            sensor_family[i]->scanner = pScanner_Pf;
            break;
        }

        case HOKUYO:
        {
            cHokuyoLaserScanner* pScanner_Hky = new cHokuyoLaserScanner(sensor_family[i]->parm->m_nLineCount,
                                                                        sensor_family[i]->parm->m_fStartAngle,
                                                                        sensor_family[i]->parm->m_fEndAngle);
            if(pScanner_Hky == NULL) {
                bRet = false;
                continue;
            }
            sensor_family[i]->scanner = pScanner_Hky;
            break;
        }

        case SICK:
        {
            sick::cSickSafetyLaserScanner* pScanner_Sick = new sick::cSickSafetyLaserScanner(sensor_family[i]->parm->m_nLineCount,
                                                                                             sensor_family[i]->parm->m_fStartAngle,
                                                                                             sensor_family[i]->parm->m_fEndAngle);
            if(pScanner_Sick == NULL) {
                bRet = false;
                continue;
            }
            sensor_family[i]->scanner = pScanner_Sick;
            break;
        }

        case SICK581:
        {
            cSick581LaserScanner * pScanner_Sick = new cSick581LaserScanner(sensor_family[i]->parm->m_nLineCount,
                                                                            sensor_family[i]->parm->m_fStartAngle,
                                                                            sensor_family[i]->parm->m_fEndAngle);
            if(pScanner_Sick == NULL) {
                bRet = false;
                continue;
            }
            sensor_family[i]->scanner = pScanner_Sick;
            break;
        }

        case WJ716:
        {
            cWJ716LaserScanner * pScanner_WJ = new cWJ716LaserScanner(sensor_family[i]->parm->m_nLineCount,
                                                                            sensor_family[i]->parm->m_fStartAngle,
                                                                            sensor_family[i]->parm->m_fEndAngle,
                                                                            sensor_family[i]->parm);
            if(pScanner_WJ == NULL) {
                bRet = false;
                continue;
            }
            sensor_family[i]->scanner = pScanner_WJ;
            break;
        }
        case WJ719:
        {
            cWJ719LaserScanner * pScanner_WJ = new cWJ719LaserScanner(sensor_family[i]->parm->m_nLineCount,
                                                                            sensor_family[i]->parm->m_fStartAngle,
                                                                            sensor_family[i]->parm->m_fEndAngle,
                                                                            sensor_family[i]->parm);
            if(pScanner_WJ == NULL) {
                bRet = false;
                continue;
            }
            sensor_family[i]->scanner = pScanner_WJ;
            break;
        }

        case LEIMOUF30:
        {
            cLeimouf30LaserScanner * pScanner_LM = new cLeimouf30LaserScanner(sensor_family[i]->parm->m_nLineCount,
                                                                            sensor_family[i]->parm->m_fStartAngle,
                                                                            sensor_family[i]->parm->m_fEndAngle,
                                                                            sensor_family[i]->parm);
            if(pScanner_LM == NULL) {
                bRet = false;
                continue;
            }
            sensor_family[i]->scanner = pScanner_LM;
            break;
        }

        case FRR2:
        {
            cFRR2000LaserScanner* pScanner_FR = new cFRR2000LaserScanner(sensor_family[i]->parm->m_nLineCount,
                                                                         sensor_family[i]->parm->m_fStartAngle,
                                                                         sensor_family[i]->parm->m_fEndAngle,
                                                                         sensor_family[i]->parm);
            if(pScanner_FR == NULL) {
                bRet = false;
                continue;
            }
            sensor_family[i]->scanner = pScanner_FR;
            break;
        }

        case LDS_50:
        {
            cLDS_50LaserScanner* pScanner_LDS = new cLDS_50LaserScanner(sensor_family[i]->parm->m_nLineCount,
                                                                         sensor_family[i]->parm->m_fStartAngle,
                                                                         sensor_family[i]->parm->m_fEndAngle,
                                                                         sensor_family[i]->parm);
            if(pScanner_LDS == NULL) {
                bRet = false;
                continue;
            }
            sensor_family[i]->scanner = pScanner_LDS;
            break;
        }
        case LDS_E320_S:
        {
            cLDS_E320LaserScanner* pScanner_LDS = new cLDS_E320LaserScanner(sensor_family[i]->parm->m_nLineCount,
                                                                         sensor_family[i]->parm->m_fStartAngle,
                                                                         sensor_family[i]->parm->m_fEndAngle,                                                               
                                                                        sensor_family[i]->parm);
            if(pScanner_LDS == NULL) {
                bRet = false;
                continue;
            }
            sensor_family[i]->scanner = pScanner_LDS;
            break;
        }
        default:
            break;
        }

        // start the scanner thread
        std::string str_ip = sensor_family[i]->parm->strIP;
        std::string host_ip = sensor_family[i]->parm->hostIP;
        // ping the scanner
        int ping_ack = m_Ping.Ping(str_ip);
        bool start_ok = false;
        if(sensor_family[i]->parm->state && sensor_family[i]->scanner != NULL && (ping_ack == 1))
        {
            start_ok = sensor_family[i]->scanner->Start(str_ip.c_str(), host_ip.c_str(),  sensor_family[i]->parm->LaserId);

            //By Sam: For diagnosis tool.Add By yu.
            if (0 == i)
            {
                if (start_ok)
                    GData::getObj().front_laser_state = 1;
                else
                    GData::getObj().front_laser_state = 0;
            }
            else
            {
                if (start_ok)
                    GData::getObj().end_laser_state = 1;
                else
                    GData::getObj().end_laser_state = 0;
            }
        }
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "-->start the scanner,i=", i, ",ping_ack=", ping_ack, ",start_ok=", (int)start_ok);
//        FILE_BlackBox(LocBox, "start the scanner,i=", i, ",start_ok=", (int)start_ok);
#endif
    }

    // 延迟１秒
    usleep(1000 * 1000);
    // create the daemon thread
    if(!CreateThread(DAEMON_THREAD_CTRL_CYCLE)){
        bRet = false;
    }

    m_bStarted = true;
    return bRet;
}

//
//   取得所有激光器的参数，以一个向量的方式返回。
//
CScannerGroupParam CSensorFamily::GetScannerGroupParam()
{
    // 在堆栈上分配激光器参数对象，并预订空间大小
    CScannerGroupParam params;
    params.resize(sensor_family.size());

    // 从各激光器对象中收集激光器参数
    for (int i = 0; i < (int)sensor_family.size(); i++)
    {
        CSensorData *sensor_data = sensor_family[i];
        if (sensor_data != NULL && sensor_data->parm != NULL)
            params[i] = *sensor_data->parm;
    }

    return params;
}

unsigned int CSensorFamily::GetCount()
{
    unsigned int count = static_cast<unsigned int>(sensor_family.size());
    return count;
}

bool CSensorFamily::GetState(unsigned int index)
{
    bool bRet = false;
    if(sensor_family.empty()) {
        return false;
    }

    if(index < GetCount()) {
        if(NULL != sensor_family[index]->parm) {
            bRet = sensor_family[index]->parm->state;
        }
    }
    return bRet;
}

bool CSensorFamily::DataReady(unsigned int index)
{
    bool bRet = false;
    if(sensor_family.empty()) {
        return false;
    }

    if(index < GetCount()) {
        if(NULL != sensor_family[index]->scanner) {
            bRet = sensor_family[index]->scanner->DataReady();
        }
    }
    return bRet;
}
//leimou pls
int CSensorFamily::GetType(unsigned int index)
{
    int type = -1;
    if(sensor_family.empty()) {
        return false;
    }
    if(index < GetCount()) {
        if(NULL != sensor_family[index]->scanner) {
            type = sensor_family[index]->scanner->m_uType;
        }
    }

    return type;
}

bool CSensorFamily::Stop()
{
    if (!m_bStarted)
        return false;

    // close the communicate thread
    for (unsigned int i = 0; i < sensor_family.size(); i++){
        if (sensor_family[i] != NULL && sensor_family[i]->scanner != NULL) {
            sensor_family[i]->scanner->Stop();
        }
    }

    // stop the daemon thread.
    StopThread();

    m_bStarted = false;
    return true;
}

bool CSensorFamily::GetPointCloud(unsigned int index,int*& pDist, int*& pIntensity)
{
    bool bRet = false;
    if(sensor_family.empty() || index >= GetCount()) {
        return false;
    }

    if(sensor_family[index]->scanner != NULL) {
        bRet = sensor_family[index]->scanner->GetPointCloud(pDist, pIntensity);
    }

    return bRet;
}

bool CSensorFamily::GetRawPointCloud(unsigned int index, std::shared_ptr<sensor::CRawPointCloud>& pCloud)
{
    bool bRet = false;
    if(sensor_family.empty() || index >= GetCount()) {
        return false;
    }

    if(sensor_family[index]->scanner != NULL) {
        bRet = sensor_family[index]->scanner->GetRawPointCloud(pCloud);
    }

    return bRet;
}

CSensorData* CSensorFamily::GetSensorData(unsigned int index)
{
    if(sensor_family.empty() || index >= GetCount()) {
        return NULL;
    }

    return sensor_family[index];
}

bool CSensorFamily::IsBlocked()
{
    bool state = false;
    if(sensor_family.empty()) {
        return state;
    }
    for (unsigned int i = 0; i < sensor_family.size(); i++){
        if (sensor_family[i] != NULL && sensor_family[i]->scanner != NULL && sensor_family[i]->parm != NULL) {
            if(sensor_family[i]->parm->state && sensor_family[i]->scanner->IsBlocked()) {
                state = true;
                //By Sam: For diagnosis tool.Add Bu yu.
                int laser_id = sensor_family[i]->parm->LaserId;
                if (0 == laser_id)
                {
                    GData::getObj().front_laser_state = 0;
                    //std::cout << "By Sam: In IsBlocked, laser i = " << laser_id << ", front_laser_state = 0" << std::endl;
                }
                else
                    if (1 == laser_id)
                    {
                        GData::getObj().end_laser_state = 0;
                        //std::cout << "By Sam: In IsBlocked, laser i = " << laser_id << ", end_laser_state = 0" << std::endl;
                    }
            }
            else{

                //By Sam: For diagnosis tool
                int laser_id = sensor_family[i]->parm->LaserId;
                if (0 == laser_id)
                {
                    GData::getObj().front_laser_state = 1;
                    //std::cout << "By Sam: In IsBlocked, laser i = " << laser_id << ", front_laser_state = 1" << std::endl;
                }
                else
                    if (1 == laser_id)
                    {
                        GData::getObj().end_laser_state = 1;
                        //std::cout << "By Sam: In IsBlocked, laser i = " << laser_id << ", end_laser_state = 1" << std::endl;
                    }
            }

        }
    }
    return state;
}

} // namespace sensor
