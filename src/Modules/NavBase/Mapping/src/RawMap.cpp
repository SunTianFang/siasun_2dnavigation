//
//   The interface of class "CRawMap".
//

#include <fstream>
#include"RawMap.h"
#include "SensorFamily.h"
#include "json/json.h"
#include "HttpCommunicationGlobalData.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CRawMap".
namespace mapping {

CRawMap::CRawMap()
{
    raw_map.clear();
    m_aCount = 0;
    m_aMaxCount = 0;
    m_aFileSaving = false;
    m_nStartTime = 0;
    m_nWriteSensorCount = 0;
}

CRawMap::~CRawMap()
{
    Clear();
}

void CRawMap::Clear()
{
    std::lock_guard<std::mutex> lock(map_mtx);
    for (unsigned int i = 0; i < raw_map.size(); i++) {
        raw_map[i].Clear();
    }
    raw_map.clear();
    m_aCount = 0;
    m_aMaxCount = 0;
    m_nStartTime = 0;
}

bool CRawMap::AddRawScan(const sensor::CRawScan& pScan)
{
    if(m_aFileSaving.load()) {
        return false;
    }
    std::lock_guard<std::mutex> lock(map_mtx);

    if(m_aCount.load() >= m_aMaxCount.load()) {
        raw_map.front().Clear();
        raw_map.pop_front();
        raw_map.push_back(pScan);
        m_aCount = m_aMaxCount.load();
    }
    else {
        raw_map.push_back(pScan);
        m_aCount += 1;
    }

    return true;
}

bool CRawMap::GetFrontRawScan(sensor::CRawScan& pScan)
{
    std::lock_guard<std::mutex> lock(map_mtx);
    if(raw_map.empty()) {
        return false;
    }
    pScan = raw_map.front();
    return true;
}

bool CRawMap::GetBackRawScan(sensor::CRawScan& pScan)
{
    std::lock_guard<std::mutex> lock(map_mtx);
    if(raw_map.empty()) {
        return false;
    }
    pScan = raw_map.back();
    return true;
}
bool CRawMap::GetRawScans(std::deque<sensor::CRawScan> &pRawScans)
{
     std::lock_guard<std::mutex> lock(map_mtx);
     if(raw_map.empty()) {
         return false;
     }
     pRawScans = raw_map;
     return true;
}

void CRawMap::SetMaxCount(unsigned long max_count)
{
    m_aMaxCount = max_count;
}

unsigned int CRawMap::GetSize()
{
    std::lock_guard<std::mutex> lock(map_mtx);
    return static_cast<unsigned int>(raw_map.size());
}

#if 0
bool CRawMap::WriteLaserParm(FILE* pFile)
{
    std::ifstream FileLaserParm(WORK_PATH"LaserParm.json");
    Json::Reader Jreader;
    Json::Value LaserParmRoot;

    if(!FileLaserParm || !pFile){
        return false;
    }


    int nVersion = 210;
    int SensorCount = 0;
    int LaserCount[4] = {0};
    int LaserProductor[4] = {0};
    bool LaserState[4] = {false};
    int RealSensorCount = 0;

    if (Jreader.parse(FileLaserParm, LaserParmRoot))
    {
        LaserParmRoot.toStyledString();

        // 文件格式版本号
        if (!LaserParmRoot["version"].isNull()) {
            nVersion = LaserParmRoot["version"].asInt();
        }
        fwrite(&nVersion, sizeof(int), 1, pFile);

        //启动时间戳
        fwrite(&m_nStartTime, sizeof(unsigned int), 1, pFile);

        //激光器个数
        if(!LaserParmRoot["laser"].isNull()) {
            SensorCount = LaserParmRoot["laser"].size();
        }
        RealSensorCount = SensorCount;
        for(int j = 0; j < SensorCount; j++) {
            if(!LaserParmRoot["laser"][j]["State"].isNull()) {
                LaserState[j] = LaserParmRoot["laser"][j]["State"].asBool();
            }
            else {
                LaserState[j] = false;
            }

            //soft pls LeiMouF30
            //if(!LaserState[j] || LaserParmRoot["laser"][j]["LaserProductor"].asInt() == LEIMOUF30)
            if(!LaserState[j] || LaserParmRoot["laser"][j]["LaserProductor"].asInt() == WJ716)
                RealSensorCount--;
        }
        fwrite(&RealSensorCount, sizeof(int), 1, pFile);

        //激光器参数部分
        for(int i = 0; i < SensorCount; i++) {
            //soft pls LeiMouF30
            if(!LaserState[i]|| LaserParmRoot["laser"][i]["LaserProductor"].asInt() == LEIMOUF30) {
                continue;
            }
            if(!LaserState[i]|| LaserParmRoot["laser"][i]["LaserProductor"].asInt() == WJ716) {
                continue;
            }

            double val_d = 0.0;
            float val_f = 0.0;
            int val_i = 0;

            if(!LaserParmRoot["laser"][i]["LaserProductor"].isNull()) {
                LaserProductor[i] = LaserParmRoot["laser"][i]["LaserProductor"].asInt();
            }

            if(!LaserParmRoot["laser"][i]["StartAngle"].isNull()) {
                val_d = LaserParmRoot["laser"][i]["StartAngle"].asDouble() / 180.0 * PI;
                val_f = float(val_d);
            }
            else {
                val_f = 0.0;
            }
            fwrite(&val_f, sizeof(float), 1, pFile);

            if(!LaserParmRoot["laser"][i]["EndAngle"].isNull()) {
                val_d = LaserParmRoot["laser"][i]["EndAngle"].asDouble() / 180.0 * PI;
                val_f = float(val_d);
            }
            else {
                val_f = 0.0;
            }
            fwrite(&val_f, sizeof(float), 1, pFile);

            if(!LaserParmRoot["laser"][i]["LaserCount"].isNull()) {
                LaserCount[i] = LaserParmRoot["laser"][i]["LaserCount"].asInt();
            }
            else {
                LaserCount[i] = false;
            }
            fwrite(&LaserCount[i],sizeof(int),1,pFile);

            if(!LaserParmRoot["laser"][i]["x"].isNull()) {
                val_d = LaserParmRoot["laser"][i]["x"].asDouble();
                val_f = float(val_d);
            }
            else {
                val_f = 0.0;
            }
            fwrite(&val_f, sizeof(float), 1, pFile);

            if(!LaserParmRoot["laser"][i]["y"].isNull()) {
                val_d = LaserParmRoot["laser"][i]["y"].asDouble();
                val_f = float(val_d);
            }
            else {
                val_f = 0.0;
            }
            fwrite(&val_f, sizeof(float), 1, pFile);

            if(!LaserParmRoot["laser"][i]["thita"].isNull()) {
                val_d = LaserParmRoot["laser"][i]["thita"].asDouble() / 180.0 * PI;
                val_f = float(val_d);
            }
            else {
                val_f = 0.0;
            }
            fwrite(&val_f, sizeof(float), 1, pFile);

            if(!LaserParmRoot["laser"][i]["VisualRange"].isNull()) {
                val_i = LaserParmRoot["laser"][i]["VisualRange"].size();
            }
            else {
                val_i = 0;
            }
            fwrite(&val_i, sizeof(int), 1, pFile);

            for(int j = 0; j < val_i; j++ )
            {
                if(!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].isNull()) {
                    val_d = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].asDouble() / 180 * PI;
                    val_f = float(val_d);
                }
                else {
                    val_f = 0.0;
                }
                fwrite(&val_f, sizeof(float), 1, pFile);

                if(!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].isNull()) {
                    val_d = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].asDouble() / 180 * PI;
                    val_f = float(val_d);
                }
                else {
                    val_f = 0.0;
                }
                fwrite(&val_f, sizeof(float), 1, pFile);
            }
        }
    }
    FileLaserParm.close();

    return true;
}
#endif
bool CRawMap::WriteLaserParm(FILE* pFile,bool bPlsLaserWrite)
{
    std::ifstream FileLaserParm(WORK_PATH"LaserParm.json");
    Json::Reader Jreader;
    Json::Value LaserParmRoot;

    if(!FileLaserParm || !pFile){
        return false;
    }


    int nVersion = 210;
    int SensorCount = 0;
    int LaserCount[4] = {0};
    int LaserProductor[4] = {0};
    bool LaserState[4] = {false};
    int RealSensorCount = 0;

    if (Jreader.parse(FileLaserParm, LaserParmRoot))
    {
        LaserParmRoot.toStyledString();

        // 文件格式版本号
        if (!LaserParmRoot["version"].isNull()) {
            nVersion = LaserParmRoot["version"].asInt();
        }
        fwrite(&nVersion, sizeof(int), 1, pFile);

        //启动时间戳
        fwrite(&m_nStartTime, sizeof(unsigned int), 1, pFile);

        //激光器个数
        if(!LaserParmRoot["laser"].isNull()) {
            SensorCount = LaserParmRoot["laser"].size();
        }

        m_nWriteSensorCount = SensorCount;

        for(int j = 0; j < SensorCount; j++) {
            if(!LaserParmRoot["laser"][j]["State"].isNull()) {
                LaserState[j] = LaserParmRoot["laser"][j]["State"].asBool();
            }
            else {
                LaserState[j] = false;
            }

            //soft pls LeiMouF30
            //if(!LaserState[j] || LaserParmRoot["laser"][j]["LaserProductor"].asInt() == LEIMOUF30)
            if(!LaserState[j] || (LaserParmRoot["laser"][j]["LaserProductor"].asInt() == WJ716 && (!bPlsLaserWrite)))
               m_nWriteSensorCount--;
        }
        fwrite(&m_nWriteSensorCount, sizeof(int), 1, pFile);

        //激光器参数部分
        for(int i = 0; i < SensorCount; i++) {
            //soft pls LeiMouF30
            if(!LaserState[i]|| (LaserParmRoot["laser"][i]["LaserProductor"].asInt() == LEIMOUF30&& (!bPlsLaserWrite))) {
                continue;
            }
            if(!LaserState[i]|| (LaserParmRoot["laser"][i]["LaserProductor"].asInt() == WJ716&& (!bPlsLaserWrite))) {
                continue;
            }

            double val_d = 0.0;
            float val_f = 0.0;
            int val_i = 0;

            if(!LaserParmRoot["laser"][i]["LaserProductor"].isNull()) {
                LaserProductor[i] = LaserParmRoot["laser"][i]["LaserProductor"].asInt();
            }

            if(!LaserParmRoot["laser"][i]["StartAngle"].isNull()) {
                val_d = LaserParmRoot["laser"][i]["StartAngle"].asDouble() / 180.0 * PI;
                val_f = float(val_d);
            }
            else {
                val_f = 0.0;
            }
            fwrite(&val_f, sizeof(float), 1, pFile);

            if(!LaserParmRoot["laser"][i]["EndAngle"].isNull()) {
                val_d = LaserParmRoot["laser"][i]["EndAngle"].asDouble() / 180.0 * PI;
                val_f = float(val_d);
            }
            else {
                val_f = 0.0;
            }
            fwrite(&val_f, sizeof(float), 1, pFile);

            if(!LaserParmRoot["laser"][i]["LaserCount"].isNull()) {
                LaserCount[i] = LaserParmRoot["laser"][i]["LaserCount"].asInt();
            }
            else {
                LaserCount[i] = false;
            }
            fwrite(&LaserCount[i],sizeof(int),1,pFile);

            if(!LaserParmRoot["laser"][i]["x"].isNull()) {
                val_d = LaserParmRoot["laser"][i]["x"].asDouble();
                val_f = float(val_d);
            }
            else {
                val_f = 0.0;
            }
            fwrite(&val_f, sizeof(float), 1, pFile);

            if(!LaserParmRoot["laser"][i]["y"].isNull()) {
                val_d = LaserParmRoot["laser"][i]["y"].asDouble();
                val_f = float(val_d);
            }
            else {
                val_f = 0.0;
            }
            fwrite(&val_f, sizeof(float), 1, pFile);

            if(!LaserParmRoot["laser"][i]["thita"].isNull()) {
                val_d = LaserParmRoot["laser"][i]["thita"].asDouble() / 180.0 * PI;
                val_f = float(val_d);
            }
            else {
                val_f = 0.0;
            }
            fwrite(&val_f, sizeof(float), 1, pFile);

            if(!LaserParmRoot["laser"][i]["VisualRange"].isNull()) {
                val_i = LaserParmRoot["laser"][i]["VisualRange"].size();
            }
            else {
                val_i = 0;
            }
            fwrite(&val_i, sizeof(int), 1, pFile);

            for(int j = 0; j < val_i; j++ )
            {
                if(!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].isNull()) {
                    val_d = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].asDouble() / 180 * PI;
                    val_f = float(val_d);
                }
                else {
                    val_f = 0.0;
                }
                fwrite(&val_f, sizeof(float), 1, pFile);

                if(!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].isNull()) {
                    val_d = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].asDouble() / 180 * PI;
                    val_f = float(val_d);
                }
                else {
                    val_f = 0.0;
                }
                fwrite(&val_f, sizeof(float), 1, pFile);
            }
        }
    }
    FileLaserParm.close();

    return true;
}


bool CRawMap::WriteScanData(FILE* pFile)
{
    if(!pFile) {
        return false;
    }

    m_aFileSaving = true;
    std::lock_guard<std::mutex> lock(map_mtx);

    //总的观测步数
    int map_size = static_cast<int>(raw_map.size());
    fwrite(&map_size, sizeof(int), 1, pFile);

    cout << "write raw map data, size: " << map_size << endl;

    auto pFamily = SensorFamilySingleton::GetInstance();

    for (int i = 0; i < map_size; i++) {
        unsigned long index = static_cast<unsigned long>(i);
        //里程数据标志字
        fwrite(&raw_map[index].odom_data.odom_flag, sizeof(unsigned int), 1, pFile);
        //里程数据的时间戳
        fwrite(&raw_map[index].odom_data.time_stamp, sizeof(unsigned int), 1, pFile);

        //机器人在当前数据步的速度分量
        float fvel[3] = {0.0};
        fvel[0] = static_cast<float>(raw_map[index].odom_data.velocity.fXLinear);
        fvel[1] = static_cast<float>(raw_map[index].odom_data.velocity.fYLinear);
        fvel[2] = static_cast<float>(raw_map[index].odom_data.velocity.fAngular);
        fwrite(fvel, 3*sizeof(float), 1, pFile);

        //机器人在当前数据步的相对里程姿态
        float fLocalOdm[3] = {0.0};
        fLocalOdm[0] = static_cast<float>(raw_map[index].odom_data.local_pst.x);
        fLocalOdm[1] = static_cast<float>(raw_map[index].odom_data.local_pst.y);
        fLocalOdm[2] = static_cast<float>(raw_map[index].odom_data.local_pst.fThita);
        fwrite(fLocalOdm, 3*sizeof(float), 1, pFile);

        //机器人在当前数据步的全局(绝对)里程姿态
        float fGlobalOdm[3] = {0.0};
        fGlobalOdm[0] = static_cast<float>(raw_map[index].odom_data.global_pst.x);
        fGlobalOdm[1] = static_cast<float>(raw_map[index].odom_data.global_pst.y);
        fGlobalOdm[2] = static_cast<float>(raw_map[index].odom_data.global_pst.fThita);
        fwrite(fGlobalOdm, 3*sizeof(float), 1, pFile);

        // 激光传感器点云数据
        unsigned long sensor_size = raw_map[index].point_cloud.size();

        sensor_size = m_nWriteSensorCount ;

        for(unsigned long j = 0; j < sensor_size; j++) {
            //std::cout<<"< sensor_size; "<< sensor_size<<std::endl;
            std::shared_ptr<sensor::CRawPointCloud>& pCloud = raw_map[index].point_cloud[j];
            if(!pCloud) {
                continue;
            }
            unsigned long dist_size = pCloud->distance.size();
            unsigned long inten_size = pCloud->intensity.size();
            if(dist_size != inten_size || dist_size < pCloud->num_points) {
                continue;
            }

            //激光器数据的时间戳
            unsigned int time_stamp = static_cast<unsigned int>(pCloud->timestamp_raw);
            fwrite(&time_stamp, sizeof(unsigned int), 1, pFile);

            sensor::CSensorData* pSensorData = pFamily->GetSensorData(pCloud->laser_id);
            int criteritonThreshold = pSensorData->parm->criteritonThreshold;

            for(unsigned int m = 0; m < pCloud->num_points; m++) {
                unsigned int dist_val = pCloud->distance[m];
                unsigned int inten_val = pCloud->intensity[m];
                unsigned short dist_ = 0;
                unsigned char inten_ = 0;
                dist_ = static_cast<unsigned short>(dist_val/2);

                // pf
//                if(pCloud->laser_type == PEPPERL_FUCHS) {
//                    if(inten_val > 2500) { //700->2500
//                        inten_val = 255;
//						inten_ = static_cast<unsigned char>(inten_val);
//                    }
//                    else
//                        inten_ = static_cast<unsigned char>(inten_val*255/2500);
//                }
//                //hokuyo
//                if(pCloud->laser_type == HOKUYO) {
//                    if(inten_val > 7000) {
//                        inten_val = 255;
//						inten_ = static_cast<unsigned char>(inten_val);
//                    }
//                    else
//                        inten_ = static_cast<unsigned char>(inten_val*255/7000);
//                }
//                //sick nano
//                if(pCloud->laser_type == SICK){
//                    if(inten_val > 2500) { //255->2500
//                        inten_val = 255;
//                        inten_ = static_cast<unsigned char>(inten_val);
//                    }
//                    else
//                        inten_ = static_cast<unsigned char>(inten_val*255/2500);
//                }
//                //sick 581???
//                if(pCloud->laser_type == SICK581){
//                    if(inten_val > 2500) { //255->2500
//                        inten_val = 255;
//                        inten_ = static_cast<unsigned char>(inten_val);
//                    }
//                    else
//                        inten_ = static_cast<unsigned char>(inten_val*255/2500);
//                }
//                //wanji 716min
//                if(pCloud->laser_type == WJ716) {
//                    if(inten_val > 2500) { //700->2550
//                        inten_val = 255;
//                        inten_ = static_cast<unsigned char>(inten_val);

//                    }
//                    else
//                        inten_ = static_cast<unsigned char>(inten_val*255/2500);
//                }
//                //wanji 719
//                if(pCloud->laser_type == WJ719) {
//                    if(inten_val > 2500) {     //wt_fix 20230104
//                        inten_val = 255;
//                        inten_ = static_cast<unsigned char>(inten_val);

//                    }
//                    else
//                        inten_ = static_cast<unsigned char>(inten_val*255/2500);
//                }

                inten_ = static_cast<unsigned char>(inten_val);

                // 激光的极径和强度
                fwrite(&dist_, sizeof(unsigned short), 1, pFile);
                fwrite(&inten_, sizeof(unsigned char), 1, pFile);
            }
        }
    }

    m_aFileSaving = false;
    return true;
}

//void CRawMap::SetStartTime(unsigned int nTime)
//{
//    m_nStartTime = nTime;
//}

bool CRawMap::GetScanData()
{
    std::lock_guard<std::mutex> lock(map_mtx);

    //观测步数
    int map_size = static_cast<int>(raw_map.size());
    if(map_size <= 0){
        return false;
    }

    unsigned long index = static_cast<unsigned long>(map_size - 1);

    //里程数据标志字
    GData::getObj().odomFlag = raw_map[index].odom_data.odom_flag;
    //里程数据的时间戳
    GData::getObj().odomTimeStamp = static_cast<unsigned int>(raw_map[index].odom_data.time_stamp);

    //机器人在当前数据步的速度分量
    float fvel[3] = {0.0};
    fvel[0] = static_cast<float>(raw_map[index].odom_data.velocity.fXLinear);
    fvel[1] = static_cast<float>(raw_map[index].odom_data.velocity.fYLinear);
    fvel[2] = static_cast<float>(raw_map[index].odom_data.velocity.fAngular);
    GData::getObj().velX = fvel[0];
    GData::getObj().velY = fvel[1];
    GData::getObj().velTheta = fvel[2];

    //机器人在当前数据步的相对里程姿态
    float fLocalOdm[3] = {0.0};
    fLocalOdm[0] = static_cast<float>(raw_map[index].odom_data.local_pst.x);
    fLocalOdm[1] = static_cast<float>(raw_map[index].odom_data.local_pst.y);
    fLocalOdm[2] = static_cast<float>(raw_map[index].odom_data.local_pst.fThita);
    GData::getObj().localOdomX = fLocalOdm[0];
    GData::getObj().localOdomY = fLocalOdm[1];
    GData::getObj().localOdomTheta = fLocalOdm[2];

    //机器人在当前数据步的全局(绝对)里程姿态
    float fGlobalOdm[3] = {0.0};
    fGlobalOdm[0] = static_cast<float>(raw_map[index].odom_data.global_pst.x);
    fGlobalOdm[1] = static_cast<float>(raw_map[index].odom_data.global_pst.y);
    fGlobalOdm[2] = static_cast<float>(raw_map[index].odom_data.global_pst.fThita);
    GData::getObj().globalOdomX = fGlobalOdm[0];
    GData::getObj().globalOdomY = fGlobalOdm[1];
    GData::getObj().globalOdomTheta = fGlobalOdm[2];

    // 激光传感器点云数据
    unsigned long sensor_size = raw_map[index].point_cloud.size();
    int data_temp = 0;

    for(unsigned long j = 0; j < sensor_size; j++) {
        std::shared_ptr<sensor::CRawPointCloud>& pCloud = raw_map[index].point_cloud[j];
        if(!pCloud) {
            continue;
        }
        unsigned long dist_size = pCloud->distance.size();
        unsigned long inten_size = pCloud->intensity.size();
        if(dist_size != inten_size || dist_size < pCloud->num_points) {
            continue;
        }

        //激光器数据的时间戳
        unsigned int laser_time_stamp = static_cast<unsigned int>(pCloud->timestamp_raw);
        GData::getObj().laserTimeStamp = laser_time_stamp;

        if(j == 0){
            GData::getObj().num_points_front = pCloud->num_points;

            if (!GData::getObj().front_laser_state)
            {
                for(int i = 0; i < pCloud->num_points; i++)
                {
                    unsigned short dist_ = 0;
                    unsigned char inten_ = 0;
                    GData::getObj().distance[i]  = dist_;
                    GData::getObj().intensity[i] = inten_;
                }
                data_temp = pCloud->num_points;
                std::cout << "~~ By Sam: update front laser data !!!!!" << std::endl;
                continue;
            }
        }
        else{
            GData::getObj().num_points_end = pCloud->num_points;

            if (!GData::getObj().end_laser_state)
            {
                for(int i = 0; i < pCloud->num_points; i++)
                {
                    int cloud_num = data_temp + i;
                    unsigned short dist_ = 0;
                    unsigned char inten_ = 0;
                    GData::getObj().distance[cloud_num]  = dist_;
                    GData::getObj().intensity[cloud_num] = inten_;
                }
                std::cout << "~~ By Sam: update end laser data !!!!!" << std::endl;
                continue;
            }
        }

        for(unsigned int m = 0; m < pCloud->num_points; m++) {
            unsigned int dist_val = pCloud->distance[m];
            unsigned int inten_val = pCloud->intensity[m];
            unsigned short dist_ = 0;
            unsigned char inten_ = 0;
            dist_ = static_cast<unsigned short>(dist_val/2);

            // pf
//            if(pCloud->laser_type == PEPPERL_FUCHS) {
//                if(inten_val > 2500) { //700->2500
//                    inten_val = 255;
//                    inten_ = static_cast<unsigned char>(inten_val);
//                }
//                else
//                    inten_ = static_cast<unsigned char>(inten_val*255/2500);
//            }
//            //hokuyo
//            if(pCloud->laser_type == HOKUYO) {
//                if(inten_val > 7000) {
//                    inten_val = 255;
//                    inten_ = static_cast<unsigned char>(inten_val);
//                }
//                else
//                    inten_ = static_cast<unsigned char>(inten_val*255/7000);
//            }
//            //sick nano
//            if(pCloud->laser_type == SICK){
//                if(inten_val > 2500) { //255->2500
//                    inten_val = 255;
//                    inten_ = static_cast<unsigned char>(inten_val);
//                }
//                else
//                    inten_ = static_cast<unsigned char>(inten_val*255/2500);
//            }
//            //sick 581???
//            if(pCloud->laser_type == SICK581){
//                if(inten_val > 2500) { //255->2500
//                    inten_val = 255;
//                    inten_ = static_cast<unsigned char>(inten_val);
//                }
//                else
//                    inten_ = static_cast<unsigned char>(inten_val*255/2500);
//            }
//            if(pCloud->laser_type == WJ716) {
//                if(inten_val > 2500) { //700->2500
//                    inten_val = 255;
//                    inten_ = static_cast<unsigned char>(inten_val);
//                }
//                else
//                    inten_ = static_cast<unsigned char>(inten_val*255/2500);
//            }
//            if(pCloud->laser_type == WJ719) {
//                if(inten_val > 2500) {   //wt_fix 20230104
//                    inten_val = 255;
//                    inten_ = static_cast<unsigned char>(inten_val);
//                }
//                else
//                    inten_ = static_cast<unsigned char>(inten_val*255/2500);
//            }

            inten_ = static_cast<unsigned char>(inten_val);

            // 激光的极径和强度
            GData::getObj().distance[data_temp]  = dist_;
            GData::getObj().intensity[data_temp] = inten_;
            data_temp++;
        }
    }
    return true;
}

void CRawMap::SetStartTime(unsigned int nTime)
{
    m_nStartTime = nTime;
}


bool CRawMap::ReadScanData(FILE* pFile,int nVersion,int sensorCount, vector<int> lineCounts)
{


    std::lock_guard<std::mutex> lock(map_mtx);

    if(!pFile){
        return false;
    }

    if(lineCounts.size()!=sensorCount)
        return false;

    raw_map.clear();

    //总的观测步数
    int map_size ;
    if (fread(&map_size, sizeof(int), 1, pFile) != 1)
        return false;
    cout << "read raw map data, size: " << map_size << endl;


   // map_size--;  //??????????????
    bool bEof = false;
    // 根据帧数读入全部的局部数据
    for (int i = 0; i < map_size; i++)
    {

        sensor::CRawScan scan;

        //里程数据标志字
        if (fread(&scan.odom_data.odom_flag, sizeof(unsigned int), 1, pFile)!=1)
            return false;

         if(nVersion>=210)
         {
            //里程数据的时间戳
            if (fread(&scan.odom_data.time_stamp, sizeof(unsigned int), 1, pFile)!=1)
                return false;

            cout << "time_stamp " <<scan.odom_data.time_stamp << endl;
         }


        float fvel[3] = {0.0};
        if (fread(fvel, 3*sizeof(float), 1, pFile)!=1)
            return false;
        scan.odom_data.velocity.fXLinear = fvel[0];
        scan.odom_data.velocity.fYLinear = fvel[1];
        scan.odom_data.velocity.fAngular = fvel[2];


        //机器人在当前数据步的相对里程姿态
        float fLocalOdm[3] = {0.0};
        if (fread(fLocalOdm, 3*sizeof(float), 1, pFile)!=1)
            return false;
        scan.odom_data.local_pst.x = fLocalOdm[0];
        scan.odom_data.local_pst.y = fLocalOdm[1];
        scan.odom_data.local_pst.fThita = fLocalOdm[2];


        //机器人在当前数据步的全局(绝对)里程姿态
        float fGlobalOdm[3] = {0.0};
        if (fread(fGlobalOdm, 3*sizeof(float), 1, pFile)!=1)
            return false;
        scan.odom_data.global_pst.x = fGlobalOdm[0];
        scan.odom_data.global_pst.y = fGlobalOdm[1];
        scan.odom_data.global_pst.fThita = fGlobalOdm[2];

        cout << "fGlobalOdm " <<fGlobalOdm[0]<<" "<<fGlobalOdm[1]<<" "<<fGlobalOdm[2] << endl;

        cout << "sensorCount " <<sensorCount << endl;

        for(unsigned long j = 0; j < sensorCount; j++)
        {
            auto ploud = std::make_shared<sensor::CRawPointCloud>();

            if(nVersion>=210)
            {
                //激光器数据的时间戳
                unsigned int time_stamp ;
                if(fread(&time_stamp, sizeof(unsigned int), 1, pFile)!=1)
                    return false;
                cout << "sensor time_stamp " <<time_stamp << endl;

                ploud->timestamp_raw = time_stamp;
                cout << "lineCounts.at(j)" <<lineCounts.at(j) << endl;
            }


            for(unsigned int m = 0; m < lineCounts.at(j); m++) {

                unsigned short dist_ = 0;
                unsigned char inten_ = 0;
                // 激光的极径和强度
                if (fread(&dist_, sizeof(unsigned short), 1, pFile)!=1)
                    return false;
                if (fread(&inten_, sizeof(unsigned char), 1, pFile)!=1)
                    return false;


                ploud->distance.push_back(dist_*2);
                ploud->intensity.push_back(inten_);


            }
            ploud->num_points = lineCounts.at(j);
            scan.point_cloud.push_back(ploud);
        }

       raw_map.push_back(scan);
    }

    return true;


}

} // namespace mapping
