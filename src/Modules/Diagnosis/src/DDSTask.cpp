#include "DDSTask.h"
#include "DDS_Data.h"
#include "json/json.h"
#include "type.h"
#include "laser_t.h"
#include <fstream>



#include "SensorFamily.h"

using namespace std;

void DDSTask::DDSInit()
{
    DDSPublisher_high.init(TOPIC_HIGH_LASER);
    DDSPublisher_low.init(TOPIC_LOW_LASER);
    DDSPublisher_robotPose.init(TOPIC_ROBOT_POSE);

    if(LoadLaserParam()) {
        CreateThread(10);
    } else {
        std::cout << "DDS load laser param failed" << std::endl;
    }

    m_laserStamp1 = 0;
    m_laserStamp2 = 0;
    m_stampLastSend = 0;
}

void DDSTask::SendMessage(NAVPointCloudDDS NAVPointCloud, string TopicName)
{
    if(!TopicName.compare(TOPIC_HIGH_LASER)) {
        DDSPublisher_high.publish(NAVPointCloud);
    } else {
        DDSPublisher_low.publish(NAVPointCloud);
    }
}

void DDSTask::SupportRoutineProxy()
{
    std::lock_guard<std::mutex> lock(Laser_msg_mutex);


    DDSSendLocalizationMsg();


}
#if 0
void DDSTask::DDSSendLocalizationMsg()
{
    PoseDDS CurPoseDDS;
    CurPoseDDS.x(LocalizationMsg.position[0]);
    CurPoseDDS.y(LocalizationMsg.position[1]);
    CurPoseDDS.theta(LocalizationMsg.position[2]);

    string TopicName = TOPIC_HIGH_LASER;

    int lastLaserCount = 0;

    for(int i =0; i < LocalizationMsg.RealSensorCount; i++)
    {
        NAVPointCloudDDS NavPCloud;
        PointCloudDDS PCloudDDS;
        vector<PoseDDS> vPCloudDDS;


        float resolution = fabs(LocalizationMsg.StartAngle[i] - LocalizationMsg.EndAngle[i]) / (LocalizationMsg.LaserCount[i]);

        float first_angle = LocalizationMsg.StartAngle[i];
        float r = 0.0f; //m
        float a = 0.0f; //rad

//        std::cout << "DDSTask DDSSendLocalizationMsg, LocalizationMsg.LaserCount[i] = " << LocalizationMsg.LaserCount[i]
//                  << ", i = " << i
//                  << ", resolution = "<< resolution
//                  << ", LocalizationMsg.StartAngle[i] = " << LocalizationMsg.StartAngle[i]
//                  << ", LocalizationMsg.EndAngle[i] = " << LocalizationMsg.EndAngle[i]
//                  << ", LocalizationMsg.RealSensorCount = " << LocalizationMsg.RealSensorCount
//                  << ", MaxRange["<<i<<"] = " << MaxRange[i]
//                  <<std::endl;

        for(int j = 0; j < LocalizationMsg.LaserCount[i]; j++)
        {

            int filteredIndex = j + lastLaserCount;
            r = dis[filteredIndex] / 1000.f;
            a = first_angle + j*resolution;

            //过滤可视角度
            if(a < LocalizationMsg.VisualAngleStart[i] || a > LocalizationMsg.VisualAngleEnd[i]) {
                continue;
            }

            float pCloudToCenterX = 0.0f;
            float pCloudToCenterY = 0.0f;
            float pCloudToCenterTheta = 0.0f;

            if(fabs(r) < pow(10,-4)) {
                //实际无限远处r为0, 不满足车体需求，所以此处修改为10000
                r = 10000.0f;
            }
            else
            {
                if(r < MinRange[i]) {
                    PoseDDS pose;
                    pose.x(0.0f);
                    pose.y(0.0f);
                    pose.theta(0.0f);
                    vPCloudDDS.push_back(pose);
                    continue;
                }
                if(r>MaxRange[i])
                {
                    r = 10000.0f;
                }

            }

            //过滤距离


         /*   if(i == 0 && j > 1295 && j < 1305)
            {
                std::cout << "r = " << r  << ",LocalizationMsg.LaserCount[i] = " << LocalizationMsg.LaserCount[i] << std::endl;
            }
*/
//            if(i == 1 && j > 535 && j < 545)
//            {
//                std::cout << "r = " << r  << ",LocalizationMsg.LaserCount[i] = " << LocalizationMsg.LaserCount[i] << std::endl;
//            }

            float pCloudX = r * (float)cos(a);
            float pCloudY = r * (float)sin(a);

            float installX = LocalizationMsg.InstallPos_X[i];
            float installY = LocalizationMsg.InstallPos_Y[i];
            float installTheta = LocalizationMsg.InstallPos_Thita[i];

            //点云转到车体中心坐标系
            pCloudToCenterX = installX + pCloudX * (float)cos(installTheta) - pCloudY * (float)sin(installTheta);
            pCloudToCenterY = installY + pCloudX * (float)sin(installTheta) + pCloudY * (float)cos(installTheta) ;
            pCloudToCenterTheta = a + installTheta;

            PoseDDS pose;
            pose.x(pCloudToCenterX);
            pose.y(pCloudToCenterY);
            pose.theta(pCloudToCenterTheta);
            /*if(i == 0 && j > 1295 && j < 1305)
            {
                std::cout << "pose x = " << pose.x()  << ",y = " << pose.y() << std::endl;
            }*/
            vPCloudDDS.push_back(pose);
        }

        PCloudDDS.points(vPCloudDDS);

        if( i <= LASER_COUNT)
        {
            PCloudDDS.seq(LaserMsgTick[i]);
        }


        NavPCloud.pose(CurPoseDDS);
        NavPCloud.points(PCloudDDS);

        switch(i)
        {
        case 0:
            TopicName = TOPIC_HIGH_LASER;
            break;

        case 1:
            TopicName = TOPIC_LOW_LASER;
            break;

        default:

            break;
        }

        NAVPointCloudDDSPubSubType NavPubSubType;
        SendMessage(NavPCloud,TopicName);
        lastLaserCount += LocalizationMsg.LaserCount[i];
//        printf("DDSSendLocalizationMsg, LaserMsgTick = %lld, lastLaserCount = %d \n", LaserMsgTick, lastLaserCount);
    }

}
#endif

bool DDSTask::LoadLaserParam()
{
    std::ifstream FileLaserParm(WORK_PATH"LaserParm.json");
    Json::Reader Jreader;
    Json::Value LaserParmRoot;

    if(!FileLaserParm){
        return false;
    }

    int16_t nVersion = 210;
    m_iSensorCount = 4;
    int16_t RealSensorCount = 2;
    int16_t LaserProductor[4] = {0};
    m_bLaserState[4] = {false};

    if (Jreader.parse(FileLaserParm, LaserParmRoot))
    {
        LaserParmRoot.toStyledString();

        // 文件格式版本号
        if (!LaserParmRoot["version"].isNull()) {
            LocalizationMsg.version = LaserParmRoot["version"].asInt();
        }

        //激光器个数
        if(!LaserParmRoot["laser"].isNull()) {
            m_iSensorCount = LaserParmRoot["laser"].size();
        }
        RealSensorCount = m_iSensorCount;
        for(int j = 0; j < m_iSensorCount; j++) {
            if(!LaserParmRoot["laser"][j]["State"].isNull()) {
                m_bLaserState[j] = LaserParmRoot["laser"][j]["State"].asBool();
            }
            else {
                m_bLaserState[j] = false;
            }

            if(!m_bLaserState[j])
                RealSensorCount--;
        }
        LocalizationMsg.RealSensorCount = RealSensorCount;
    }
    else
        return false;

    //激光器参数部分

    int iLasersLineCount = 0;

    for(int i = 0; i < m_iSensorCount; i++)
    {
        if(!m_bLaserState[i]) {
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
            LocalizationMsg.StartAngle[i] = float(val_d);
        }
        else {
            LocalizationMsg.StartAngle[i] = 0.0f;
        }

        if(!LaserParmRoot["laser"][i]["EndAngle"].isNull()) {
            val_d = LaserParmRoot["laser"][i]["EndAngle"].asDouble() / 180.0 * PI;
            LocalizationMsg.EndAngle[i] = float(val_d);
        }
        else {
            LocalizationMsg.EndAngle[i] = 0.0f;
        }

        if(!LaserParmRoot["laser"][i]["LaserCount"].isNull()) {
            LocalizationMsg.LaserCount[i] = LaserParmRoot["laser"][i]["LaserCount"].asInt();

            iLasersLineCount += LocalizationMsg.LaserCount[i];

        }
        else {
            LocalizationMsg.LaserCount[i] = 0;
        }
        //LocalizationMsg.LaserCount[i] = LaserCount[i];

        if(!LaserParmRoot["laser"][i]["x"].isNull()) {
            val_d = LaserParmRoot["laser"][i]["x"].asDouble();
            LocalizationMsg.InstallPos_X[i] = float(val_d);
        }
        else {
            LocalizationMsg.InstallPos_X[i] = 0.0;
        }


        if(!LaserParmRoot["laser"][i]["y"].isNull()) {
            val_d = LaserParmRoot["laser"][i]["y"].asDouble();
            LocalizationMsg.InstallPos_Y[i] = float(val_d);
        }
        else {
            LocalizationMsg.InstallPos_Y[i] = 0.0;
        }


        if(!LaserParmRoot["laser"][i]["thita"].isNull()) {
            val_d = LaserParmRoot["laser"][i]["thita"].asDouble() / 180.0 * PI;
            LocalizationMsg.InstallPos_Thita[i] = float(val_d);
        }
        else {
            LocalizationMsg.InstallPos_Thita[i] = 0.0;
        }

        if(!LaserParmRoot["laser"][i]["MaxRange"].isNull()) {
            val_d = LaserParmRoot["laser"][i]["MaxRange"].asDouble();
            MaxRange[i] = float(val_d);
        }

        if(!LaserParmRoot["laser"][i]["MinRange"].isNull()) {
            val_d = LaserParmRoot["laser"][i]["MinRange"].asDouble();
            MinRange[i] = float(val_d);
        }

        if(!LaserParmRoot["laser"][i]["VisualRange"].isNull()) {
            LocalizationMsg.VisualRange[i] = LaserParmRoot["laser"][i]["VisualRange"].size();
        }
        else {
            LocalizationMsg.VisualRange[i] = 0;
        }

        //            float* VisualAngleStart = new float [4];
        //            float* VisualAngleEnd = new float [4];

        for(int j = 0; j < LocalizationMsg.VisualRange[i]; j++ )
        {
            if(!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].isNull()) {
                val_d = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].asDouble() / 180 * PI;
                val_f = float(val_d);
            }
            else {
                val_f = 0.0;
            }
            LocalizationMsg.VisualAngleStart[i] = val_f;

            if(!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].isNull()) {
                val_d = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].asDouble() / 180 * PI;
                val_f = float(val_d);
            }
            else {
                val_f = 0.0;
            }
            LocalizationMsg.VisualAngleEnd[i] = val_f;
        }
    }

    FileLaserParm.close();


 /*   dis = new int16_t[iLasersLineCount];
    memset(dis,0,iLasersLineCount);
    LocalizationMsg.lenth=iLasersLineCount;
    lcmintensities = new int16_t[iLasersLineCount];
    memset(lcmintensities,0,iLasersLineCount);

    nLasersLineCount = iLasersLineCount;*/


}

bool DDSTask::GetLocalizationMsg(sensor::CRawScan& pScan, CStampedPos &robotPosture)
{

    std::lock_guard<std::mutex> lock(Laser_msg_mutex);

/*    int ntotalsize = 0;
    int ncursize = 0;
   // std::cout << "GetLocalizationMsg,  LocalizationMsg.RealSensorCount = " <<  LocalizationMsg.RealSensorCount << std::endl;
    for(int i = 0; i < LocalizationMsg.RealSensorCount; i++)
    {
        if(m_bLaserState[i])
        {
            ntotalsize += (int)LocalizationMsg.LaserCount[i];
            LocalizationMsg.timestamp_data[i] = 0;
        }
    }

    // PART2 DATA
//      cout<< "part2 totalsize: "<< ntotalsize << endl;
//    for(int i = 0; i < LocalizationMsg.RealSensorCount; i++)
//    {
//        ncursize += LocalizationMsg.LaserCount[i];
//    }


    if(ntotalsize >nLasersLineCount)
    {
#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "Real LaserLineCount ", ntotalsize, "   JSON LasersLineCount", nLasersLineCount);
#endif
        return false;
    }


    ncursize=0;

    //By yu : protect for laser pointcloud is null.
    if(pScan.point_cloud.size() <= 0)
        return false;
    for(int i = 0; i < LocalizationMsg.RealSensorCount; i++)
    {
        if(pScan.point_cloud[i] == NULL) //By yu.do protect when point cloud is null.
            continue;
        if(m_bLaserState[i] && (LocalizationMsg.LaserCount[i] == pScan.point_cloud[i]->num_points))
        {
            for(int j = 0; j < LocalizationMsg.LaserCount[i]; j++)
            {
                dis[ncursize + j] = pScan.point_cloud[i]->distance[j];
                lcmintensities[ncursize + j] = pScan.point_cloud[i]->intensity[j];
            }
        }

//        if(i < LASER_COUNT)
//        {
        LaserMsgTick[i] = pScan.point_cloud[i]->timestamp_raw;
            //        }
        ncursize += LocalizationMsg.LaserCount[i];
    }


    LocalizationMsg.distance = dis;
    LocalizationMsg.intensity = lcmintensities;


    LocalizationMsg.vel[0] = pScan.odom_data.velocity.fXLinear;
    LocalizationMsg.vel[1] = pScan.odom_data.velocity.fYLinear;
    LocalizationMsg.vel[2] = pScan.odom_data.velocity.fAngular;

    LocalizationMsg.timestamp_vel = 0;

    bLocalMsgReady = true;*/
    LocalizationMsg.position[0] = robotPosture.x;
    LocalizationMsg.position[1] = robotPosture.y;
    LocalizationMsg.position[2] = robotPosture.fThita;

    m_robotStamp = robotPosture.m_dwTimeStamp;

    return true;
}


void DDSTask::DDSSendLocalizationMsg()
{

    PoseDDS CurPoseDDS;
    CurPoseDDS.x(LocalizationMsg.position[0]);
    CurPoseDDS.y(LocalizationMsg.position[1]);
    CurPoseDDS.theta(LocalizationMsg.position[2]);

    GeneralDataDDS robotPose;

    const std::vector<double> _dparams(LocalizationMsg.position,LocalizationMsg.position+3);

    robotPose.seq(m_robotStamp);
    robotPose.dparams(_dparams);

    DDSPublisher_robotPose.publish(robotPose);

    string TopicName = TOPIC_HIGH_LASER;

    int lastLaserCount = 0;

    auto pFamily = SensorFamilySingleton::GetInstance();

    for(int i = 0;i < pFamily->GetCount();i++)
    {
        auto scan = pFamily->GetSensorData(i);

         if(!pFamily->GetState(i))
             continue;

         if(!pFamily->DataReady(i))
             continue;

         NAVPointCloudDDS NavPCloud;
         PointCloudDDS PCloudDDS;
         vector<PoseDDS> vPCloudDDS;


        float m_dStartTheta = scan->parm->m_fStartAngle;
        float m_dPreTheta = fabs((scan->parm->m_fStartAngle - scan->parm->m_fEndAngle)/scan->parm->m_nLineCount);

        int size = scan->scanner->m_nPointCount;
        vector<int>  data;
        std::shared_ptr<sensor::CRawPointCloud> pCloud;
        scan->scanner->GetRawPointCloud(pCloud);

        float angular_increment = 0.0;
        if(size > 0) {
                angular_increment = ( scan->parm->m_fEndAngle - scan->parm->m_fStartAngle) / size;
        }

        long long int tmNow = GetTickCount();
        if((tmNow-m_stampLastSend)<50)
        {
            if(i==0)
            {
                if(m_laserStamp1 == pCloud->timestamp_raw)
                    continue;

            }
            if(i==1)
            {
                if(m_laserStamp2 == pCloud->timestamp_raw)
                    continue;
            }
        }

        for(int j = 0;j < size;j++)
        {

            float a = m_dStartTheta + j*angular_increment;

            float r = pCloud->distance[j];
            // 过虑掉不满足可视角度的点
            if(!scan->parm->m_AppAngleRange.Contain(a)){
                r = 0.0;
            }
            r = r/1000.f;

            float pCloudToCenterX = 0.0f;
            float pCloudToCenterY = 0.0f;
            float pCloudToCenterTheta = 0.0f;

            if(fabs(r) < pow(10,-4)) {
                //实际无限远处r为0, 不满足车体需求，所以此处修改为10000
                r = 10000.0f;
            }
            else
            {
                if(r < MinRange[i]) {
                    PoseDDS pose;
                    pose.x(0.0f);
                    pose.y(0.0f);
                    pose.theta(0.0f);
                    vPCloudDDS.push_back(pose);
                    continue;
                }
                if(r>MaxRange[i])
                {
                    r = 10000.0f;
                }

            }

            float pCloudX = r * (float)cos(a);
            float pCloudY = r * (float)sin(a);

            float installX = LocalizationMsg.InstallPos_X[i];
            float installY = LocalizationMsg.InstallPos_Y[i];
            float installTheta = LocalizationMsg.InstallPos_Thita[i];

            //点云转到车体中心坐标系
            pCloudToCenterX = installX + pCloudX * (float)cos(installTheta) - pCloudY * (float)sin(installTheta);
            pCloudToCenterY = installY + pCloudX * (float)sin(installTheta) + pCloudY * (float)cos(installTheta) ;
            pCloudToCenterTheta = a + installTheta;

            PoseDDS pose;
            pose.x(pCloudToCenterX);
            pose.y(pCloudToCenterY);
            pose.theta(pCloudToCenterTheta);
           // if(i == 0 && j > 1795 && j < 1815)
           // {
            //    std::cout << "r =  "<< r<< "  pose x = " << pose.x()  << ",y = " << pose.y() << std::endl;
           // }
            vPCloudDDS.push_back(pose);
        }
        PCloudDDS.points(vPCloudDDS);
        PCloudDDS.seq(pCloud->timestamp_raw);



        if(i==0)
            m_laserStamp1 = pCloud->timestamp_raw;

        if(i==1)
            m_laserStamp2 = pCloud->timestamp_raw;


       // std::cout << "i = " << i  << "pCloud->timestamp_raw" << pCloud->timestamp_raw << std::endl;

        NavPCloud.pose(CurPoseDDS);
        NavPCloud.points(PCloudDDS);

        switch(i)
        {
        case 0:
            TopicName = TOPIC_HIGH_LASER;
            break;

        case 1:
            TopicName = TOPIC_LOW_LASER;
            break;

        default:

            break;
        }

        NAVPointCloudDDSPubSubType NavPubSubType;
        SendMessage(NavPCloud,TopicName);
        m_stampLastSend = GetTickCount();


    }
}



