#ifndef GDATA_H
#define GDATA_H

#include <atomic>
#include "iostream"
#include "string"
#include "SensorFamily.h"
#include "blackboxhelper.hpp"

#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif

class GData
{
public:
    static GData & getObj(){
        if(!_instance())
        {
            _instance() = new GData();
        }
        return * _instance();
    }

private:
    static GData*& _instance(){
        static GData * p = new GData();
        return p;
    }

public:
    //flag
    bool setParmFlag;
    bool flag;
    bool startRecordDxFlag;
    bool finishRecordDxFlag;

    //point cloud parm
    std::atomic<int> num_points_front;
    std::atomic<int> num_points_end;
    std::atomic<unsigned short> distance[7200];
    std::atomic<unsigned short> intensity[7200];

    //agv position and quality
    std::atomic<int> x;
    std::atomic<int> y;
    std::atomic<int> theta;
    std::atomic<int> uG;
    std::atomic<int> uN;

    //laser parm
    std::atomic<int> laser_num;

    std::atomic<int> front_laser_state;    // By Sam
    std::atomic<int> front_laser_count;
    std::atomic<int> front_line;
    std::atomic<float> front_x;
    std::atomic<float> front_y;
    std::atomic<float> front_theta;
    std::atomic<float> front_start_angle;
    std::atomic<float> front_end_angle;

    std::atomic<int> front_visual_range_size;
    std::atomic<float> front_visual_angle_start;
    std::atomic<float> front_visual_angle_end;

    std::atomic<int> end_laser_state;    // By Sam
    std::atomic<int> end_laser_count;
    std::atomic<int> end_line;
    std::atomic<float> end_x;
    std::atomic<float> end_y;
    std::atomic<float> end_theta;
    std::atomic<float> end_start_angle;
    std::atomic<float> end_end_angle;

    std::atomic<int> end_visual_range_size;
    std::atomic<float> end_visual_angle_start;
    std::atomic<float> end_visual_angle_end;

    // By Sam: robolocalparm
    std::atomic<int> evaluate_uG;    // By Sam
    std::atomic<int> evaluate_uN;

    //dx
    std::atomic<unsigned int> odomFlag;
    std::atomic<unsigned int> odomTimeStamp;
    std::atomic<float> velX;
    std::atomic<float> velY;
    std::atomic<float> velTheta;
    std::atomic<float> localOdomX;
    std::atomic<float> localOdomY;
    std::atomic<float> localOdomTheta;
    std::atomic<float> globalOdomX;
    std::atomic<float> globalOdomY;
    std::atomic<float> globalOdomTheta;
    std::atomic<unsigned int> laserTimeStamp;

    //setting parm
    bool setting_flag_x;
    bool setting_flag_y;
    bool setting_flag_theta;
    std::atomic<float> set_pos_x;
    std::atomic<float> set_pos_y;
    std::atomic<float> set_pos_theta;

    //confident, oru, omp flag, for set pose
    std::atomic<int> confident_flag;
    std::atomic<int> oru_flag;
    std::atomic<int> omp_flag;

    std::atomic<int> m_setPosThreshold_x;
    std::atomic<int> m_setPosThreshold_y;
    std::atomic<float> m_setPosThreshold_theta;



public:
    // By Sam: Init parameter
    GData();

    // By Sam: Only change front laser??????
    bool set(std::string name, std::string value);
};

#endif
