#include "HttpCommunicationGlobalData.h"

#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

GData::GData()
{
    //flag
    setParmFlag = false;
    flag = false;
	startRecordDxFlag = false;
    finishRecordDxFlag = false;

    //point cloud parm
    num_points_front = 0;
    num_points_end = 0;
    distance[7200] = {0};
    intensity[7200] = {0};

    //agv position and quality
    x = 0;
    y = 0;
    theta = 0;
    uG = 0;
    uN = 0;

    //laser parm
    laser_num = 0;

    front_laser_state = 0;
    front_laser_count = 0;
    front_line = 0;
    front_x = 0.0;
    front_y = 0.0;
    front_theta = 0.0;
    front_start_angle = 0.0;
    front_end_angle = 0.0;

    front_visual_range_size = 0;
    front_visual_angle_start = 0.0;
    front_visual_angle_end = 0.0;

    end_laser_state = 0;
    end_laser_count = 0;
    end_line = 0;
    end_x = 0.0;
    end_y = 0.0;
    end_theta = 0.0;
    end_start_angle = 0.0;
    end_end_angle = 0.0;

    end_visual_range_size = 0;
    end_visual_angle_start = 0.0;
    end_visual_angle_end = 0.0;

    // By Sam: robolocalparm
    evaluate_uG = 0;    // By Sam
    evaluate_uN = 0;    // By Sam

    //dx
    odomFlag = 0;
    odomTimeStamp = 0;
    velX = 0.0;
    velY = 0.0;
    velTheta = 0.0;
    localOdomX = 0.0;
    localOdomY = 0.0;
    localOdomTheta = 0.0;
    globalOdomX = 0.0;
    globalOdomY = 0.0;
    globalOdomTheta = 0.0;
    laserTimeStamp = 0;

    //setting parm
    setting_flag_x = false;
    setting_flag_y = false;
    setting_flag_theta = false;
    set_pos_x = 0.0;
    set_pos_y = 0.0;
    set_pos_theta = 0.0;
}

bool GData::set(std::string name, std::string value)
{
    auto pAFamily = SensorFamilySingleton::GetInstance();

    if(name == "posX") {
        pAFamily->GetSensorData(0)->parm->m_pst.x = std::stof(value);
        front_x = std::stof(value);
    }
    if(name == "posY") {
        pAFamily->GetSensorData(0)->parm->m_pst.y = std::stof(value);
        front_y = std::stof(value);
    }
    if(name == "posTheta") {
        pAFamily->GetSensorData(0)->parm->m_pst.fThita = std::stof(value);
        front_theta = std::stof(value);
    }

    setParmFlag = true;
}
