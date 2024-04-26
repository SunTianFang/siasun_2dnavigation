#include "ParameterObject.h"
#include <fstream>
#include "json/json.h"
#include "Geometry.h"
#include "Project.h"

namespace robo
{

bool ParameterObject::InitializeParameters()
{
    if(!LoadRoboLocParm()){
        return false;
    }

    if(!LoadRoboAdvParm()){
        return false;
    }

    if(!LoadSimuParm()){
        return false;
    }

    return true;
}

bool ParameterObject::LoadRoboLocParm()
{
    std::ifstream FileLocParm(WORK_PATH"RoboLocParm.json");
    Json::Reader Jreader;
    Json::Value LocParmRoot;
    if(!FileLocParm){
        return false;
    }
    if (!Jreader.parse(FileLocParm, LocParmRoot)){
        FileLocParm.close();
        return false;
    }
    LocParmRoot.toStyledString();

    // set the Mapping parameters
    if (!LocParmRoot["Mapping"]["Resolution"].isNull()) {
         AddParameter("Mapping_Resolution", LocParmRoot["Mapping"]["Resolution"].asDouble());
    }
    if (!LocParmRoot["Mapping"]["MappingDataSetSize"].isNull()) {
         AddParameter("Mapping_MappingDataSetSize", LocParmRoot["Mapping"]["MappingDataSetSize"].asInt());
    }
    if (!LocParmRoot["Mapping"]["LocalizeDataSetSize"].isNull()) {
         AddParameter("Mapping_LocalizeDataSetSize", LocParmRoot["Mapping"]["LocalizeDataSetSize"].asInt());
    }

    if (!LocParmRoot["TopVision"]["Enable"].isNull()) {
         AddParameter("TopVision_Enable", LocParmRoot["TopVision"]["Enable"].asBool());
    }



    // set the CAN parameters
    if (!LocParmRoot["CAN"]["Activation"].isNull()) {
         AddParameter("CAN_Activation", LocParmRoot["CAN"]["Activation"].asBool());
    }

    // set the Gyro parameters
    if (!LocParmRoot["Gyro"]["Enable"].isNull()) {
         AddParameter("Gyro_Enable", LocParmRoot["Gyro"]["Enable"].asBool());
    }
    if (!LocParmRoot["Gyro"]["Remote"].isNull()) {
         AddParameter("Gyro_Remote", LocParmRoot["Gyro"]["Remote"].asBool());
    }

    // set the Filter parameters
    if (!LocParmRoot["Filter"]["Enable"].isNull()) {
         AddParameter("Filter_Enable", LocParmRoot["Filter"]["Enable"].asBool());
    }

    unsigned int nFactorCount = 0;
    std::vector<double> factor_vector;
    if (!LocParmRoot["Filter"]["Factor"].isNull()) {
        nFactorCount = LocParmRoot["Filter"]["Factor"].size();
        factor_vector.reserve(nFactorCount);
        for(unsigned int i = 0; i < nFactorCount; i++){
            factor_vector.push_back(LocParmRoot["Filter"]["Factor"][i].asDouble());
        }
        AddParameter("Filter_Factor", factor_vector);
    }

    // set the Evaluate parameters
    if (!LocParmRoot["Evaluate"]["MinQualityLevel"].isNull()) {
         AddParameter("Evaluate_MinQualityLevel", LocParmRoot["Evaluate"]["MinQualityLevel"].asInt());
    }
    if (!LocParmRoot["Evaluate"]["MinMatchNum"].isNull()) {
         AddParameter("Evaluate_MinMatchNum", LocParmRoot["Evaluate"]["MinMatchNum"].asUInt());
    }
    if (!LocParmRoot["Evaluate"]["LocGoodKeepDist"].isNull()) {
         AddParameter("Evaluate_LocGoodKeepDist", LocParmRoot["Evaluate"]["LocGoodKeepDist"].asDouble());
    }
    if (!LocParmRoot["Evaluate"]["LocErrorTolerateDist"].isNull()) {
         AddParameter("Evaluate_LocErrorTolerateDist", LocParmRoot["Evaluate"]["LocErrorTolerateDist"].asDouble());
    }
    if (!LocParmRoot["Evaluate"]["LocFailTolerateDist"].isNull()) {
         AddParameter("Evaluate_LocFailTolerateDist", LocParmRoot["Evaluate"]["LocFailTolerateDist"].asDouble());
    }

    // set the SyncTime parameters
    if (!LocParmRoot["SyncTime"]["RobotEnd"]["Enable"].isNull()) {
         AddParameter("SyncTime_RobotEnd_Enable", LocParmRoot["SyncTime"]["RobotEnd"]["Enable"].asBool());
    }
    if (!LocParmRoot["SyncTime"]["RobotEnd"]["Remote"].isNull()) {
         AddParameter("SyncTime_RobotEnd_Remote", LocParmRoot["SyncTime"]["RobotEnd"]["Remote"].asBool());
    }
    if (!LocParmRoot["SyncTime"]["RobotEnd"]["CycleTime"].isNull()) {
         AddParameter("SyncTime_RobotEnd_CycleTime", LocParmRoot["SyncTime"]["RobotEnd"]["CycleTime"].asUInt());
    }
    if (!LocParmRoot["SyncTime"]["RobotEnd"]["TolerateSpan"].isNull()) {
         AddParameter("SyncTime_RobotEnd_TolerateSpan", LocParmRoot["SyncTime"]["RobotEnd"]["TolerateSpan"].asUInt());
    }
    if (!LocParmRoot["SyncTime"]["RobotEnd"]["Timeout"].isNull()) {
         AddParameter("SyncTime_RobotEnd_Timeout", LocParmRoot["SyncTime"]["RobotEnd"]["Timeout"].asUInt());
    }
    if (!LocParmRoot["SyncTime"]["SensorEnd"]["Enable"].isNull()) {
         AddParameter("SyncTime_SensorEnd_Enable", LocParmRoot["SyncTime"]["SensorEnd"]["Enable"].asBool());
    }

    // set the Diagnosis parameters
    if (!LocParmRoot["Diagnosis"]["UseCoreDump"].isNull()) {
         AddParameter("Diagnosis_UseCoreDump", LocParmRoot["Diagnosis"]["UseCoreDump"].asBool());
    }
    if (!LocParmRoot["Diagnosis"]["AutoSaveLog"].isNull()) {
         AddParameter("Diagnosis_AutoSaveLog", LocParmRoot["Diagnosis"]["AutoSaveLog"].asBool());
    }
    if (!LocParmRoot["Diagnosis"]["PublishPointCloud"].isNull()) {
         AddParameter("Diagnosis_PublishPointCloud", LocParmRoot["Diagnosis"]["PublishPointCloud"].asBool());
    }
    if (!LocParmRoot["Diagnosis"]["PublishNdtCell"].isNull()) {
         AddParameter("Diagnosis_PublishNdtCell", LocParmRoot["Diagnosis"]["PublishNdtCell"].asBool());
    }
    if (!LocParmRoot["Diagnosis"]["PublishReflector"].isNull()) {
         AddParameter("Diagnosis_PublishReflector", LocParmRoot["Diagnosis"]["PublishReflector"].asBool());
    }
    if (!LocParmRoot["Diagnosis"]["HttpServerFlag"].isNull()) {
         AddParameter("Diagnosis_HttpServerFlag", LocParmRoot["Diagnosis"]["HttpServerFlag"].asBool());
    }
    if (!LocParmRoot["Diagnosis"]["UseSoftPls"].isNull()) {
         AddParameter("Diagnosis_UseSoftPls", LocParmRoot["Diagnosis"]["UseSoftPls"].asBool());
    }

    FileLocParm.close();
    return true;
}

bool ParameterObject::LoadSimuParm()
{
    std::ifstream FileSimuParm(WORK_PATH"SimuParm.json");
    Json::Reader Jreader;
    Json::Value SimuParmRoot;
    if(!FileSimuParm){
        return false;
    }
    if (!Jreader.parse(FileSimuParm, SimuParmRoot)){
        FileSimuParm.close();
        return false;
    }
    SimuParmRoot.toStyledString();

    // set the Simulate parameters
    if (!SimuParmRoot["SimuVersion"].isNull()) {
        AddParameter("SimuVersion", SimuParmRoot["SimuVersion"].asString());
    }
    if (!SimuParmRoot["Simulate"].isNull()) {
        AddParameter("Simulate", SimuParmRoot["Simulate"].asBool());
    }

    // set the map parameters
    if (!SimuParmRoot["Map"]["SizeX"].isNull()) {
        AddParameter("Map_SizeX", SimuParmRoot["Map"]["SizeX"].asDouble());
    }
    if (!SimuParmRoot["Map"]["SizeY"].isNull()) {
        AddParameter("Map_SizeY", SimuParmRoot["Map"]["SizeY"].asDouble());
    }
    if (!SimuParmRoot["Map"]["Resolution"].isNull()) {
        AddParameter("Map_Resolution", SimuParmRoot["Map"]["Resolution"].asDouble());
    }

    // set the init pose
    if (!SimuParmRoot["InitPose"]["X"].isNull()) {
        AddParameter("InitPose_X", SimuParmRoot["InitPose"]["X"].asDouble());
    }
    if (!SimuParmRoot["InitPose"]["Y"].isNull()) {
        AddParameter("InitPose_Y", SimuParmRoot["InitPose"]["Y"].asDouble());
    }
    if (!SimuParmRoot["InitPose"]["Thita"].isNull()) {
        AddParameter("InitPose_Thita", SimuParmRoot["InitPose"]["Thita"].asDouble());
    }

    FileSimuParm.close();
    return true;
}

bool ParameterObject::LoadRoboAdvParm()
{
    std::ifstream FileAdvParm(WORK_PATH"RoboAdvParm.json");
    Json::Reader Jreader;
    Json::Value AdvParmRoot;
    if(!FileAdvParm){
        return false;
    }
    if (!Jreader.parse(FileAdvParm, AdvParmRoot)){
        FileAdvParm.close();
        return false;
    }
    AdvParmRoot.toStyledString();

    // set the Localize Expand parameters
    if (!AdvParmRoot["LocalizeExpand"]["UseInInitPose"].isNull()) {
        AddParameter("LocalizeExpand_UseInInitPose", AdvParmRoot["LocalizeExpand"]["UseInInitPose"].asBool());
    }
    if (!AdvParmRoot["LocalizeExpand"]["UseInPathTracing"].isNull()) {
        AddParameter("LocalizeExpand_UseInPathTracing", AdvParmRoot["LocalizeExpand"]["UseInPathTracing"].asBool());
    }
    if (!AdvParmRoot["LocalizeExpand"]["UnitDistance"].isNull()) {
        AddParameter("LocalizeExpand_UnitDistance", AdvParmRoot["LocalizeExpand"]["UnitDistance"].asFloat());
    }
    if (!AdvParmRoot["LocalizeExpand"]["UnitAngle"].isNull()) {
        AddParameter("LocalizeExpand_UnitAngle", AdvParmRoot["LocalizeExpand"]["UnitAngle"].asFloat());
    }
    if (!AdvParmRoot["LocalizeExpand"]["RangeDistance"].isNull()) {
        AddParameter("LocalizeExpand_RangeDistance", AdvParmRoot["LocalizeExpand"]["RangeDistance"].asFloat());
    }
    if (!AdvParmRoot["LocalizeExpand"]["RangeAngle"].isNull()) {
        //转化成弧度
        float range_angle = CAngle::ToRadian(AdvParmRoot["LocalizeExpand"]["RangeAngle"].asFloat());
        AddParameter("LocalizeExpand_RangeAngle", range_angle);
    }
    if (!AdvParmRoot["LocalizeExpand"]["ThresholdCount"].isNull()) {
        AddParameter("LocalizeExpand_ThresholdCount", AdvParmRoot["LocalizeExpand"]["ThresholdCount"].asInt());
    }
    if (!AdvParmRoot["LocalizeExpand"]["ThresholdRatio"].isNull()) {
        AddParameter("LocalizeExpand_ThresholdRatio", AdvParmRoot["LocalizeExpand"]["ThresholdRatio"].asFloat());
    }
    if (!AdvParmRoot["LocalizeExpand"]["ConsistencyDistance"].isNull()) {
        AddParameter("LocalizeExpand_ConsistencyDistance", AdvParmRoot["LocalizeExpand"]["ConsistencyDistance"].asDouble());
    }
    if (!AdvParmRoot["LocalizeExpand"]["ConsistencyCount"].isNull()) {
        AddParameter("LocalizeExpand_ConsistencyCount", AdvParmRoot["LocalizeExpand"]["ConsistencyCount"].asInt());
    }

    //set the ndt_oru parameters
    if (!AdvParmRoot["LocalizeOru"]["ThresholdCount"].isNull()) {
        AddParameter("LocalizeOru_ThresholdCount", AdvParmRoot["LocalizeOru"]["ThresholdCount"].asInt());
    }
    if (!AdvParmRoot["LocalizeOru"]["ThresholdRatio"].isNull()) {
        AddParameter("LocalizeOru_ThresholdRatio", AdvParmRoot["LocalizeOru"]["ThresholdRatio"].asFloat());
    }
    if (!AdvParmRoot["LocalizeOru"]["ConsistencyTranslationX"].isNull()) {
        AddParameter("LocalizeOru_ConsistencyTranslationX", AdvParmRoot["LocalizeOru"]["ConsistencyTranslationX"].asDouble());
    }
    if (!AdvParmRoot["LocalizeOru"]["ConsistencyTranslationY"].isNull()) {
        AddParameter("LocalizeOru_ConsistencyTranslationY", AdvParmRoot["LocalizeOru"]["ConsistencyTranslationY"].asDouble());
    }
    if (!AdvParmRoot["LocalizeOru"]["ConsistencyRotationZ"].isNull()) {
        AddParameter("LocalizeOru_ConsistencyRotationZ", AdvParmRoot["LocalizeOru"]["ConsistencyRotationZ"].asDouble());
    }

    //set the ndt_omp parameters
    if (!AdvParmRoot["LocalizeOmp"]["ThresholdCount"].isNull()) {
        AddParameter("LocalizeOmp_ThresholdCount", AdvParmRoot["LocalizeOmp"]["ThresholdCount"].asInt());
    }
    if (!AdvParmRoot["LocalizeOmp"]["ThresholdRatio"].isNull()) {
        AddParameter("LocalizeOmp_ThresholdRatio", AdvParmRoot["LocalizeOmp"]["ThresholdRatio"].asFloat());
    }
    if (!AdvParmRoot["LocalizeOmp"]["ConsistencyTranslationX"].isNull()) {
        AddParameter("LocalizeOmp_ConsistencyTranslationX", AdvParmRoot["LocalizeOmp"]["ConsistencyTranslationX"].asDouble());
    }
    if (!AdvParmRoot["LocalizeOmp"]["ConsistencyTranslationY"].isNull()) {
        AddParameter("LocalizeOmp_ConsistencyTranslationY", AdvParmRoot["LocalizeOmp"]["ConsistencyTranslationY"].asDouble());
    }
    if (!AdvParmRoot["LocalizeOmp"]["ConsistencyRotationZ"].isNull()) {
        AddParameter("LocalizeOmp_ConsistencyRotationZ", AdvParmRoot["LocalizeOmp"]["ConsistencyRotationZ"].asDouble());
    }

    FileAdvParm.close();
    return true;
}

}  // namespace robo
