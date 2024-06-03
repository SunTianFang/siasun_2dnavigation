
#ifndef NAVIGATION_TYPE_H_
#define NAVIGATION_TYPE_H_



#include <vector>
#include <map>
#include <string>
#include <sys/time.h>

#include <iostream>

#include "mapping/include/options.h"

// read json file
#include <fstream>
#include "LinuxSetting.h"
#include "MagicSingleton.h"
#include "json/json.h"

using namespace std;

// DQ add 2022.06.14
class CartoParm
{
public:


    CartoParm()
    {
        SetDefaultCartoParm();
        LoadJsonForParam();
    }
    void LoadJsonForParam()
    {

        std::ifstream   FileCartoParm(WORK_PATH"CartoParm.json");
        Json::Reader    Jreader;
        Json::Value     CartoParmRoot;


        std::string ss = std::string(WORK_PATH"CartoParm.json");

        std::cout<<ss<<std::endl;

        if ( !FileCartoParm ) {
            printf ( "Load CartoParm.json failed,can not open json\n" );
            return;
        }
        if ( !Jreader.parse ( FileCartoParm, CartoParmRoot ) ) {
            printf ( "Load CartoParm.json failed\n" );
            FileCartoParm.close();
            return;
        }
        CartoParmRoot.toStyledString();

        // MotionFilterOptions
        if ( !CartoParmRoot["ClientParm"]["MotionFilter"]["MaxTime"].isNull() ) {   //
            MotionParam.max_time_seconds = CartoParmRoot["ClientParm"]["MotionFilter"]["MaxTime"].asDouble();
        }
        if ( !CartoParmRoot["ClientParm"]["MotionFilter"]["MaxDis"].isNull() ) {    //
            MotionParam.max_distance_meters = CartoParmRoot["ClientParm"]["MotionFilter"]["MaxDis"].asDouble();
        }
        if ( !CartoParmRoot["ClientParm"]["MotionFilter"]["MaxAng"].isNull() ) {    //
            MotionParam.max_angle_radians = CartoParmRoot["ClientParm"]["MotionFilter"]["MaxAng"].asDouble();
             MotionParam.max_angle_radians =  MotionParam.max_angle_radians *3.1415/180.0  ;
        }

        // RealTimeCorrelativeScanMatcherOptions
        if ( !CartoParmRoot["DeveloperParm"]["ScanMatch"]["LinearWindows"].isNull() ) {    //
            RealTimeScanMatchParam.linear_search_window = CartoParmRoot["DeveloperParm"]["ScanMatch"]["LinearWindows"].asDouble();
        }
        if ( !CartoParmRoot["DeveloperParm"]["ScanMatch"]["AngularWindows"].isNull() ) {   //
            RealTimeScanMatchParam.angular_search_window = CartoParmRoot["DeveloperParm"]["ScanMatch"]["AngularWindows"].asDouble();
            RealTimeScanMatchParam.angular_search_window = RealTimeScanMatchParam.angular_search_window *3.1415/180.0  ;
        }
        if ( !CartoParmRoot["DeveloperParm"]["ScanMatch"]["TransDeltaCostWeight"].isNull() ) { //
            RealTimeScanMatchParam.translation_delta_cost_weight = CartoParmRoot["DeveloperParm"]["ScanMatch"]["TransDeltaCostWeight"].asDouble();
        }
        if ( !CartoParmRoot["DeveloperParm"]["ScanMatch"]["RotDeltaCostWeight"].isNull() ) {   //
            RealTimeScanMatchParam.rotation_delta_cost_weight = CartoParmRoot["DeveloperParm"]["ScanMatch"]["RotDeltaCostWeight"].asDouble();
        }

        // FastCorrelativeScanMatcherOptions2D



       if ( !CartoParmRoot["ClientParm"]["LoopClosure"]["SubmapNodeNum"].isNull() ) {  //
            SubmapsParam.num_range_data = CartoParmRoot["ClientParm"]["LoopClosure"]["SubmapNodeNum"].asInt();
        }
        if ( !CartoParmRoot["ClientParm"]["LoopClosure"]["LinearWindows"].isNull() ) {  //
            FastScanMatchParam.linear_search_window = CartoParmRoot["ClientParm"]["LoopClosure"]["LinearWindows"].asDouble();
        }
        if ( !CartoParmRoot["ClientParm"]["LoopClosure"]["AngularWindows"].isNull() ) { //
            FastScanMatchParam.angular_search_window = CartoParmRoot["ClientParm"]["LoopClosure"]["AngularWindows"].asDouble();

           FastScanMatchParam.angular_search_window = FastScanMatchParam.angular_search_window *3.1415/180.0  ;
        }

        // ConstraintBuilderOptions
        if ( !CartoParmRoot["ClientParm"]["LoopClosure"]["MaxDis"].isNull() ) { // �Ծֲ���ͼ���лػ�����ʱ�ܳ�Ϊ�ػ�����������
            ConstraintBuilderParam.max_constraint_distance = CartoParmRoot["ClientParm"]["LoopClosure"]["MaxDis"].asDouble();
        }
        if ( !CartoParmRoot["ClientParm"]["LoopClosure"]["MinScore"].isNull() ) {   // �Ծֲ���ͼ���лػ�����ʱ�ܳ�Ϊ�ػ������ͷ�����ֵ
            ConstraintBuilderParam.min_score = CartoParmRoot["ClientParm"]["LoopClosure"]["MinScore"].asDouble();
        }

        // LocalTrajectoryBuilderOptions2D
        if ( !CartoParmRoot["DeveloperParm"]["ScanMatch"]["Enable"].isNull() ) {   // �Ƿ�����CSM����֡����׼
            LocalTrajectoryBuilderParam.use_online_correlative_scan_matching = CartoParmRoot["DeveloperParm"]["ScanMatch"]["Enable"].asBool();
        }
        if ( !CartoParmRoot["ClientParm"]["LaserRange"]["Min"].isNull() ) { // ���ⷶΧmin
            LocalTrajectoryBuilderParam.min_range = CartoParmRoot["ClientParm"]["LaserRange"]["Min"].asDouble();
        }
        if ( !CartoParmRoot["ClientParm"]["LaserRange"]["Max"].isNull() ) { // ���ⷶΧmax

            LocalTrajectoryBuilderParam.max_range = CartoParmRoot["ClientParm"]["LaserRange"]["Max"].asDouble();

     //       std::cout<<"LocalTrajectoryBuilderParam.max_range "<<LocalTrajectoryBuilderParam.max_range<<std::endl;

        }

        // PoseGraphOptions
        if ( !CartoParmRoot["ClientParm"]["Optimize"]["EveryXNode"].isNull() ) {    // ÿ�����ٽڵ�ִ��һ�κ����Ż�
            PoseGraphParam.optimize_every_n_nodes = CartoParmRoot["ClientParm"]["Optimize"]["EveryXNode"].asInt();
        }
        // bool
        if ( !CartoParmRoot["ClientParm"]["Optimize"]["EnableOdom"].isNull() ) {    // �Ƿ���������
            bEnableOdom = CartoParmRoot["ClientParm"]["Optimize"]["EnableOdom"].asBool();
        }

     /*   // set the DeveloperParm parameters
        if ( !CartoParmRoot["DeveloperParm"]["Odom"]["Enable"].isNull() ) {
            m_cartoParam.Odom_Enable = CartoParmRoot["DeveloperParm"]["Odom"]["Enable"].asBool();
        }
        if ( !CartoParmRoot["DeveloperParm"]["Odom"]["TransWeight"].isNull() ) {
            m_cartoParam.Odom_TransWeight = CartoParmRoot["DeveloperParm"]["Odom"]["TransWeight"].asDouble();
        }
        if ( !CartoParmRoot["DeveloperParm"]["Odom"]["RotWeight"].isNull() ) {
            m_cartoParam.Odom_RotWeight = CartoParmRoot["DeveloperParm"]["Odom"]["RotWeight"].asDouble();
        }

        if ( !CartoParmRoot["DeveloperParm"]["ScanMatch"]["Enable"].isNull() ) {
            m_cartoParam.Odom_RotWeight = CartoParmRoot["DeveloperParm"]["ScanMatch"]["Enable"].asBool();
        }
        if ( !CartoParmRoot["DeveloperParm"]["ScanMatch"]["LinearWindows"].isNull() ) {
            m_cartoParam.Odom_RotWeight = CartoParmRoot["DeveloperParm"]["ScanMatch"]["LinearWindows"].asDouble();
        }
        if ( !CartoParmRoot["DeveloperParm"]["ScanMatch"]["AngularWindows"].isNull() ) {
            m_cartoParam.Odom_RotWeight = CartoParmRoot["DeveloperParm"]["ScanMatch"]["AngularWindows"].asDouble();
        }
        if ( !CartoParmRoot["DeveloperParm"]["ScanMatch"]["TransDeltaCostWeight"].isNull() ) {
            m_cartoParam.Odom_RotWeight = CartoParmRoot["DeveloperParm"]["ScanMatch"]["TransDeltaCostWeight"].asDouble();
        }
        if ( !CartoParmRoot["DeveloperParm"]["ScanMatch"]["RotDeltaCostWeight"].isNull() ) {
            m_cartoParam.Odom_RotWeight = CartoParmRoot["DeveloperParm"]["ScanMatch"]["RotDeltaCostWeight"].asDouble();
        }
    */

        printf ( "Load CartoParm.json successed\n" );
    }

    struct proto::MotionFilterOptions GetMotionParam()
    {
        return MotionParam;
    }

    struct proto::RealTimeCorrelativeScanMatcherOptions GetRealTimeScanMatchParam()
    {
        return RealTimeScanMatchParam;
    }

    struct proto::FastCorrelativeScanMatcherOptions2D GetFastScanMatchParam()
    {
        return FastScanMatchParam;
    }

    struct proto::ConstraintBuilderOptions GetConstraintBuilderParam()
    {
        return ConstraintBuilderParam;
    }

    struct proto::LocalTrajectoryBuilderOptions2D GetLocalTrajectoryBuilderParam()
    {
        return LocalTrajectoryBuilderParam;
    }

    struct proto::PoseGraphOptions GetPoseGraphParam()
    {
        return PoseGraphParam;
    }

    struct proto::OptimizationProblemOptions GetOptimizationProblemParam()
    {
        return OptimizationProblemParam;
    }

    struct proto::SubmapsOptions2D GetSubmapsParam()
    {
        return SubmapsParam;
    }

    bool GetEnableOdom()
    {
        return bEnableOdom;
    }
    void SetFastScanMatchParam(double linear_window,double angle_window)
    {
        FastScanMatchParam.linear_search_window  = linear_window;
        FastScanMatchParam.angular_search_window  = angle_window;

    }
    void SetConstraintBuilderParam(double max_constraint_distance,double min_score)
    {
        ConstraintBuilderParam.max_constraint_distance  = max_constraint_distance;
        ConstraintBuilderParam.min_score                = min_score;

    }
    void SetMotionFilterParam(double max_distance_meters,double max_angle_radians,double max_time_seconds)
    {
        MotionParam.max_time_seconds     = max_time_seconds;
        MotionParam.max_distance_meters  = max_distance_meters;
        MotionParam.max_angle_radians    = max_angle_radians;
    }

    void SetPoseGraphParam(int optimize_every_n_nodes)
    {
        PoseGraphParam.optimize_every_n_nodes   = optimize_every_n_nodes;

    }
    void SetSubmapsParam(int num_range_data)
    {
        SubmapsParam.num_range_data          = num_range_data;

    }

    /*struct TrajectoryOptions GetTrajectoryParam()
    {
        return TrajectoryParam;
    }*/

private:

    struct proto::MotionFilterOptions                   MotionParam;            // �˶��˲�

    struct proto::RealTimeCorrelativeScanMatcherOptions RealTimeScanMatchParam; // ��׼����

    struct proto::FastCorrelativeScanMatcherOptions2D   FastScanMatchParam;     // �ػ�����

    struct proto::ConstraintBuilderOptions              ConstraintBuilderParam; // �Ծֲ���ͼ���лػ�����

    struct proto::LocalTrajectoryBuilderOptions2D       LocalTrajectoryBuilderParam;    // ����

    struct proto::PoseGraphOptions                      PoseGraphParam;         // �����Ż�

    struct proto::OptimizationProblemOptions            OptimizationProblemParam;       // ����

    struct proto::SubmapsOptions2D                      SubmapsParam;           // ��ͼ

    bool                                                 bEnableOdom;            // �Ƿ�ʹ������

    // struct TrajectoryOptions                            TrajectoryParam;
public:
    void SetDefaultCartoParm()
    {
        // �˶��˲�����������һ�����в���
        MotionParam.max_time_seconds                         = 15;       // �˶��˲�����ʱ��
        MotionParam.max_distance_meters                      = 0.2;       // �˶��˲���������
        MotionParam.max_angle_radians                        = 5.0*3.1415/180.0;       // �˶��˲������Ƕ�

        RealTimeScanMatchParam.linear_search_window          = 0.1;    // ��׼��������Χ
        RealTimeScanMatchParam.angular_search_window         = 15.0*3.1415/180.0;      // ƥ�������Ƕȷ�Χ
        RealTimeScanMatchParam.translation_delta_cost_weight = 1e-1;    // ƥ������λ�Ƴͷ�
        RealTimeScanMatchParam.rotation_delta_cost_weight    = 1e-1;    // ƥ��������ת�ͷ�

        FastScanMatchParam.linear_search_window              = 5;       // �ػ���������Χ
        FastScanMatchParam.angular_search_window             = 30.0*3.1415/180.0;      // �ػ������Ƕ�

        ConstraintBuilderParam.max_constraint_distance       = 15;      // �Ծֲ���ͼ���лػ�����ʱ�ܳ�Ϊ�ػ�����������
        ConstraintBuilderParam.min_score                     = 0.55;    // �Ծֲ���ͼ���лػ�����ʱ�ܳ�Ϊ�ػ������ͷ�����ֵ

        LocalTrajectoryBuilderParam.use_online_correlative_scan_matching = true; // �Ƿ�����CSM����֡����׼
        LocalTrajectoryBuilderParam.min_range                = 0.2;     // ���ⷶΧmin
        LocalTrajectoryBuilderParam.max_range                = 30;      // ���ⷶΧmax

        PoseGraphParam.optimize_every_n_nodes                = 90;      // ÿ�����ٽڵ�ִ��һ�κ����Ż�

        OptimizationProblemParam.odometry_translation_weight = 1e5;     // ����λ��Ȩ��
        OptimizationProblemParam.odometry_rotation_weight    = 1e5;     // ������תȨ��

        SubmapsParam.num_range_data                          = 90;      // һ����ͼ�������״����ݵĸ�����һ��

        bEnableOdom                                          = false;    // �Ƿ���������
    }
};

using CartoParmSingleton = MagicSingleton<CartoParm>;




typedef struct
{
    float   x;
    float   y;
    float   theta;
} Pose;

typedef struct
{

        double loop_linear_search_window;
        double loop_angular_search_window;
        double createmap_range;

} LaserNaviConfig;



typedef struct{
  double resolution;
  int nwidth;
  int nheight;
  double x0;
  double y0;
}map_info;



typedef struct
{
        int x;
        int y;
        int griddata;

}grid_data;

typedef struct{

        timeval stamp;
        Pose local_pose;
        std::vector<Pose> vtlaser;
        bool isfirstframe;
        int nodeId;
        double realtimescore;

}slam_result;


typedef struct{

        int node_id;
        timeval stamp;
        Pose global_pose;


}opt_result;





struct submap_entry
{

        int submap_index;
    Pose pose;
};

typedef struct{

        double stamp;
        std::vector<submap_entry> submaps;

}submap_list;

typedef struct{

  std::vector<unsigned char> pixels;

  int num_y_cells;
  int num_x_cells;
  double resolution;


  double max_x;
  double max_y;

  Pose slice_pose;
 }submap_data;




typedef struct
{
    int node_id;
    timeval stamp;
    Pose pos;

}node_data;



enum emWarnInfo
{
  WARN_EVERYTHING_IS_OK,
  WARN_LONG_TIME_NOT_RECIEVE_ODOM,
  WARN_LONG_TIME_NOT_RECIEVE_LASER,
  WARN_CANNOT_OPEN_MAP,
  WARN_WRONG_MAP_FILE_FORMAT,
  WARN_SAVE_OPEN_FAILURE,
  WARN_SAVE_HAVENOT_DATA,
};


struct Vector3
{
        float x;
        float y;
        float z;
};

;
struct imu_msg
{
        double stamp;
        Vector3  linear_velocity;
        Vector3  angular_acceleration;
};



struct odometry_msg{

        double stamp;  //sec
        Pose pos;
};

struct laserscan_msg{

        unsigned long long stamp;
        float angle_min;        // start angle of the scan [rad]
        float angle_max;        // end angle of the scan [rad]
        float angle_increment;  // angular distance between measurements [rad]
        float time_increment;   // time between measurements [seconds] - if your scanner
                         // is moving, this will be used in interpolating position
                         // of 3d points
        float scan_time;        // time between scans [seconds]

        float range_min;        // minimum range value [m]
        float range_max ;       // maximum range value [m]

        vector<float> ranges;
        vector<float> intensities;


        std::string sensor_id;
        Pose laser_tf ;

};

typedef  void (*SlamResultCbFunc)(slam_result *slam, std::vector<opt_result> *opt);
typedef  void (*LocationCbFunc)(int result,Pose *globalpos,laserscan_msg *msg,map_info *mapinfo,vector<double> &initlocreults);

typedef  void (*MapchangeCbFund)(std::vector<grid_data> *pchangeinfo);

typedef  void (*BuildOverFunc)(void);

typedef  void (*OptOverFunc)(std::vector<opt_result> *opt);

#endif
