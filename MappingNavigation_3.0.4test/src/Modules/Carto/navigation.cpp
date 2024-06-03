#include "navigation.h"

#include <sys/time.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <fstream>



#include "transform.h"
#include "rigid_transform.h"
#include "mymath.h"
#include "msg_conversion.h"

#include "mymath.h"


////////////////////////////////////////////////
//  实现cartographer 建图应用 接口的设计
//   Author: lishen
//   Date:   2022.4
///////////////////////////////////////////////


CCartoSlam* CCartoSlam::pSingle = nullptr ;


NodeOptions CreateNodeOptions(LaserNaviConfig config) {
    NodeOptions options;
    options.map_builder_options =
        ::mapping::CreateMapBuilderOptions(config);
    options.map_frame = std::string("map");

    options.createmap_range = config.createmap_range;

    return options;
}

TrajectoryOptions CreateTrajectoryOptions(void)
{
    // ??? CartoParm ???????
    auto pCartoParm = CartoParmSingleton::GetInstance();


    TrajectoryOptions options;

    options.use_odometry = true;
    options.num_laser_scans = 1;
    options.num_multi_echo_laser_scans = 0;
    options.num_subdivisions_per_laser_scan = 1;                  ///////////////////
    options.num_point_clouds = 0;


    CHECK_GE(options.num_laser_scans + options.num_multi_echo_laser_scans +
               options.num_point_clouds,
           1)
      << "Configuration error: 'num_laser_scans', "
         "'num_multi_echo_laser_scans' and 'num_point_clouds' are "
         "all zero, but at least one is required.";


    proto::TrajectoryBuilderOptions trajectory_builder_options;

    trajectory_builder_options.pure_localization = false;

    proto::LocalTrajectoryBuilderOptions2D trajectory_builder_2d_options;


    trajectory_builder_2d_options.min_range = pCartoParm->GetLocalTrajectoryBuilderParam().min_range;   //??????
    trajectory_builder_2d_options.max_range = pCartoParm->GetLocalTrajectoryBuilderParam().max_range;
    trajectory_builder_2d_options.min_z     = -0.8;
    trajectory_builder_2d_options.max_z     = 2.0;
    trajectory_builder_2d_options.missing_data_ray_length       = 5.0;
    trajectory_builder_2d_options.num_accumulated_range_data    = 1;                ///////gai
    trajectory_builder_2d_options.voxel_filter_size             = 0.025;
    trajectory_builder_2d_options.use_online_correlative_scan_matching  = pCartoParm->GetLocalTrajectoryBuilderParam().use_online_correlative_scan_matching;     //////
    trajectory_builder_2d_options.use_imu_data                          = false;
    trajectory_builder_2d_options.imu_gravity_time_constant             = 10.0;


    proto::AdaptiveVoxelFilterOptions adaptive_voxel_filter_options;
    adaptive_voxel_filter_options.max_length = 0.2;
    adaptive_voxel_filter_options.min_num_points = 300;
    adaptive_voxel_filter_options.max_range = 50.0;
    trajectory_builder_2d_options.adaptive_voxel_filter_options = adaptive_voxel_filter_options;

    proto::AdaptiveVoxelFilterOptions loop_closure_adaptive_voxel_filter_options;
    loop_closure_adaptive_voxel_filter_options.max_length = 0.5;
    loop_closure_adaptive_voxel_filter_options.min_num_points = 150;
    loop_closure_adaptive_voxel_filter_options.max_range = 50.0;
    trajectory_builder_2d_options.loop_closure_adaptive_voxel_filter_options = loop_closure_adaptive_voxel_filter_options;

    proto::CeresScanMatcherOptions2D cere_options;
    cere_options.occupied_space_weight = 20.0;;
    cere_options.translation_weight = 10.0;
    cere_options.rotation_weight = 1.0;

    proto::RealTimeCorrelativeScanMatcherOptions realtime_scanmatch_option;

    realtime_scanmatch_option.linear_search_window          = pCartoParm->GetRealTimeScanMatchParam().linear_search_window;//??
    realtime_scanmatch_option.angular_search_window         = pCartoParm->GetRealTimeScanMatchParam().angular_search_window;
    realtime_scanmatch_option.translation_delta_cost_weight = pCartoParm->GetRealTimeScanMatchParam().translation_delta_cost_weight;
    realtime_scanmatch_option.rotation_delta_cost_weight    = pCartoParm->GetRealTimeScanMatchParam().rotation_delta_cost_weight;
    realtime_scanmatch_option.angular_resolution = 0.3*3.1415/180.0;

  //  std::cout<<"realtime_scanmatch_option.linear_search_window"<<realtime_scanmatch_option.linear_search_window<<std::endl;

   // std::cout<<"realtime_scanmatch_option.angular_search_window"<<realtime_scanmatch_option.angular_search_window<<std::endl;




    trajectory_builder_2d_options.real_time_correlative_scan_matcher_options = realtime_scanmatch_option;

    trajectory_builder_2d_options.ceres_scan_matcher_options = cere_options;


    proto::MotionFilterOptions motion_filter_options;

    motion_filter_options.max_time_seconds      = pCartoParm->GetMotionParam().max_time_seconds;
    motion_filter_options.max_distance_meters   = pCartoParm->GetMotionParam().max_distance_meters;     // gai  0.2
    motion_filter_options.max_angle_radians     = pCartoParm->GetMotionParam().max_angle_radians;       // 10du     // 0.004538;

    trajectory_builder_2d_options.motion_filter_options = motion_filter_options;


    proto::SubmapsOptions2D submaps_options;

    submaps_options.num_range_data = pCartoParm->GetSubmapsParam().num_range_data;  //90;       ///////////modifiy


    //std::cout<<"submaps_options.num_range_data"<<submaps_options.num_range_data<<std::endl;


    proto::GridOptions2D grid_options;

    grid_options.grid_type  = proto::GridOptions2D::PROBABILITY_GRID;
    grid_options.resolution = 0.05;

    submaps_options.grid_options_2d = grid_options;


    proto::RangeDataInserterOptions range_data_insert_options;

    range_data_insert_options.range_data_inserter_type = proto::RangeDataInserterOptions::RangeDataInserterType::PROBABILITY_GRID_INSERTER_2D;

    proto::ProbabilityGridRangeDataInserterOptions2D prob_gird_range_data_insert_options;

    prob_gird_range_data_insert_options.hit_probability     = 0.55;
    prob_gird_range_data_insert_options.miss_probability    = 0.49;
    prob_gird_range_data_insert_options.insert_free_space   = true;

    CHECK_GT(prob_gird_range_data_insert_options.hit_probability, 0.5);
    CHECK_LT(prob_gird_range_data_insert_options.miss_probability, 0.5);

    range_data_insert_options.probability_grid_range_data_inserter_options_2d = prob_gird_range_data_insert_options;

    submaps_options.range_data_inserter_options = range_data_insert_options;

    CHECK_GT(submaps_options.num_range_data, 0);

    trajectory_builder_2d_options.submaps_options = submaps_options;

    trajectory_builder_options.trajectory_builder_2d_options = trajectory_builder_2d_options;



    // trajectory_builder_options.overlapping_submaps_trimmer_2d.fresh_submaps_count = 0;
    // trajectory_builder_options.overlapping_submaps_trimmer_2d.min_covered_area = 0;
    // trajectory_builder_options.overlapping_submaps_trimmer_2d.min_added_submaps_count = 0;

    options.trajectory_builder_options = trajectory_builder_options;

    return options;
}

CCartoSlam::CCartoSlam(void)
{


    pnode = nullptr;

    m_metersPerPixel = 0.05;

    m_createmap_range = 15;

    m_status = INVALID;
    m_pslamCbFunc = NULL;

}

/*
CCartoSlam::CCartoSlam(const NodeOptions& _node_options,
       mapping::MapBuilderInterface *map_builder  ,const TrajectoryOptions& _trajectory_options)
      :node(_node_options,map_builder),
      trajectory_options(_trajectory_options)
{


    m_metersPerPixel = 0.05;

    m_createmap_range = 15;

    node.SetSlamPoseCbFunc((LocalSlamPoseCbFunc)(&GetLocalSlamPose));

    m_createmap_range =_node_options.createmap_range;

    pthis = this;

    m_status = INVALID;

}
*/


CCartoSlam::~CCartoSlam(void)
{

    if(pnode!=nullptr)
        delete pnode;
}
void CCartoSlam::CreateNode()
{
    LaserNaviConfig config;

    auto pCartoParm = CartoParmSingleton::GetInstance();

    config.createmap_range = 15;
    config.loop_linear_search_window = pCartoParm->GetFastScanMatchParam().linear_search_window;
    config.loop_angular_search_window = pCartoParm->GetFastScanMatchParam().angular_search_window;


    static NodeOptions node_options = CreateNodeOptions(config);


    static mapping::MapBuilder map_builder(node_options.map_builder_options);

    pnode = new Node(node_options, &map_builder);
    pnode->SetSlamPoseCbFunc((LocalSlamPoseCbFunc)(&GetLocalSlamPose));

}


void CCartoSlam::RegisterSlamCallBack(SlamResultCbFunc pFunc)
{
    m_pslamCbFunc = pFunc;

}

void CCartoSlam::RegisterBuildOverCallBack(BuildOverFunc pFunc)
{

    if ( pnode == NULL ) {
        return ;
    }
    pnode->RegisterBuildOverCallBack(pFunc);

}

 void CCartoSlam::RegisterOptOverCallBack(OptOverFunc pFunc)
 {
     if ( pnode == NULL ) {
         return ;
     }
     pnode->RegisterOptOverCallBack(pFunc);
 }


void CCartoSlam::HandleLaserData( laserscan_msg *pmsg)
{
    if ( pnode == NULL ) {
        return ;
    }
    std::lock_guard<std::mutex> lock(build_mtx);

    transform::Rigid2d  loc;
    sensor_msgs::LaserScan msg;

   // long int sec = (long int)(pmsg->stamp/1000.0);  //ms   to   sec
   // long int usec = (long int)(((double)pmsg->stamp/1000.0-sec)*1000000ll);

    long int sec = (long int)(pmsg->stamp/1000.0);  //ms   to   sec
    long int usec = (pmsg->stamp%1000)*1000;

    msg.header.stamp.SetTime(sec , usec);

    msg.angle_min = pmsg-> angle_min;
    msg.angle_max = pmsg->angle_max;
    msg.angle_increment = pmsg->angle_increment;

    msg.time_increment = pmsg->time_increment;

    msg.scan_time = pmsg->scan_time;

    msg.range_min = pmsg->range_min;
    msg.range_max = pmsg->range_max;

    msg.ranges =  pmsg->ranges;
    msg.intensities = pmsg->intensities;
    msg.sensor_id = pmsg->sensor_id;

    Eigen::Matrix<double, 3, 1> tf(pmsg->laser_tf.x, pmsg->laser_tf.y,pmsg->laser_tf.theta) ;
    msg.laser_tf = tf;


    if(m_status == MAP )
    {
        transform::Rigid2d offset;
        pnode->HandleLaserScanMessage(false,msg,loc,msg.header.stamp,offset);
    }

}
void CCartoSlam::HandleLaserData( sensor_msgs::LaserScan &msg)
{

    std::lock_guard<std::mutex> lock(build_mtx);
    bool res_loc;

    transform::Rigid2d  loc;

    if ( pnode == NULL ) {
        return ;
    }


    if(m_status == MAP )
    {

        transform::Rigid2d offset;
        pnode->HandleLaserScanMessage(false,msg,loc,msg.header.stamp,offset);

    }


}

void CCartoSlam::HandleLaserData(sensor::PointCloud &pointclouds)
{

    std::lock_guard<std::mutex> lock(build_mtx);

    if ( pnode == NULL ) {
        return ;
    }

    if(m_status == MAP )
    {
        transform::Rigid2d offset;
        pnode->HandleLaserScanMessage(pointclouds);
    }
}





void CCartoSlam::HandleEncoderData(sensor_msgs::OdometryProto &msg)
{
    std::lock_guard<std::mutex> lock(build_mtx);
    if ( pnode == NULL ) {
        return ;
    }

    if(m_status == MAP )
    {
        msg.theta = common::NormalizeAngleDifference( msg.theta);
        pnode->HandleOdometryMessage(0,(std::string)("odom"),msg);
    }


}

void CCartoSlam::HandleEncoderData(odometry_msg *pmsg)
{
    if ( pnode == NULL ) {
        return ;
    }
     std::lock_guard<std::mutex> lock(build_mtx);

     //printf("odometry stamp = %f\n", pmsg->stamp);
     sensor_msgs::OdometryProto msg;

     msg.stamp = pmsg->stamp;
     msg.x = pmsg->pos.x;
     msg.y = pmsg->pos.y;
     msg.theta = pmsg->pos.theta;

     if(m_status == MAP )
         pnode->HandleOdometryMessage(0,(std::string)("odom"),msg);


}


void CCartoSlam::GetLocalPose(Pose &localpos)
{
    //std::lock_guard<std::mutex> lock(build_mtx);
    localpos = local_pose;
}

void CCartoSlam::GetLocalSlamPose(int *isaddnode,const common::Time *ptime,const transform::Rigid3d *plocal_pose,sensor::RangeData *prangedata,bool *bupdatemap,bool *bfirstframe,mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> *pnodepose,double *prealtimescore)
{

    Pose xyt;

    double theta = transform::GetYaw((*plocal_pose).rotation());
    Eigen::Matrix<double, 3, 1> xyz = (*plocal_pose).translation();

    xyt.x = xyz(0);
    xyt.y = xyz(1);
    xyt.theta = theta;

    //printf(" x = %f, y=%f, theta= %f, bfirstframe = %d\n",xyt.x, xyt.y,xyt.theta,*bfirstframe);

    pSingle->local_pose = xyt;

    // std::cout<<"x = "<<xyt.x<<"y="<<xyt.y<<"theta="<<xyt.theta<<std::endl;
    vector<Pose> vtlaserpoiont;


    float c = cos(xyt.theta);
    float s = sin(xyt.theta);


    for(int i=0 ;i < (*prangedata).returns.size();i++)
    {
        Pose pt;
        float dx = ((*prangedata).returns.at(i))(0)-xyt.x;
        float dy = ((*prangedata).returns.at(i))(1)-xyt.y;

        pt.x = c*dx + s*dy;
        pt.y = -s*dx + c*dy;
        pt.theta =  ((*prangedata).returns.at(i))(2);

        vtlaserpoiont.push_back(pt);


    }


    slam_result result;


    result.stamp.tv_sec = ptime->Sec();
    result.stamp.tv_usec = ptime->Usec();

    result.local_pose = xyt;
    result.nodeId = *isaddnode;
    result.isfirstframe = *bfirstframe;

    result.vtlaser= vtlaserpoiont;
    result.realtimescore= *prealtimescore;

    std::vector<opt_result> vtopt;


    for (const auto& node_id_data : *pnodepose)
    {

        common::optional<mapping::TrajectoryNodePose::ConstantPoseData> constant_pose_data = node_id_data.data.constant_pose_data;

        mapping::TrajectoryNodePose::ConstantPoseData tmp = 	constant_pose_data.value();

        transform::Rigid3d global_pose = node_id_data.data.global_pose;


        double theta = transform::GetYaw(global_pose.rotation());
        Eigen::Matrix<double, 3, 1> xyz = global_pose.translation();


        Eigen::Matrix<double, 3, 1> mm = transform::RotationQuaternionToAngleAxisVector(global_pose.rotation());
        Eigen::Matrix<double, 3, 1> tt = global_pose.translation();


        opt_result opt;

        opt.node_id = node_id_data.id.node_index;
        opt.stamp.tv_sec = tmp.time.Sec();
        opt.stamp.tv_usec = tmp.time.Usec();

        Pose ps;
        ps.x = xyz(0);
        ps.y = xyz(1);
        ps.theta = theta;

        opt.global_pose = ps;

        vtopt.push_back(opt);


    }

     if(pSingle->m_pslamCbFunc != NULL)
        pSingle->m_pslamCbFunc(&result, &vtopt);


}


void CCartoSlam::AddLaserID(std::string laser_id)
{
    // std::lock_guard<std::mutex> lock(build_mtx);

    if ( pnode == NULL ) {
        return ;
    }
    pnode->AddLaserID(laser_id);

}



bool CCartoSlam::createMap()
{

    std::lock_guard<std::mutex> lock(build_mtx);

    if ( pnode == NULL ) {

        CreateNode();
    }
    if ( pnode == NULL ) {
        return false;
    }
    if(pnode->IsTrajectoryActive())
    {
        pnode->FinishTrajectory(0);
    }

    if(!pnode->IsInit())
    {
        pnode->Reset();
    }

    Eigen::Vector2f pos(0.0, 0.0);
    m_status = MAP;
    TrajectoryOptions trajectory_options = CreateTrajectoryOptions();
    trajectory_options.trajectory_builder_options.trajectory_builder_2d_options.submaps_options.grid_options_2d.resolution = m_metersPerPixel;
    trajectory_options.trajectory_builder_options.pure_localization = false;

    pnode->AddTrajectory(trajectory_options,pos,false,nullptr);



    return true;
}




int CCartoSlam::saveMap(const string filename)
{
    // std::lock_guard<std::mutex> lock(build_mtx);
    if ( pnode == NULL ) {
        return false;
    }

    if(m_status==MAP)
    {

        if(pnode->IsTrajectoryActive())
        {
            pnode->FinishTrajectory(0);
           // pnode->RunFinalOptimization();
            return pnode->SaveMap(filename,m_metersPerPixel,m_createmap_range);
        }
        else
        {
            return pnode->SaveMap(filename,m_metersPerPixel,m_createmap_range);
        }
    }
    else
        return 1;


}

int CCartoSlam::saveMap(const string filename,std::vector<sensor_msgs::LaserScan> &vtLasers)
{
    if ( pnode == NULL ) {
        return -1;
    }
   //std::lock_guard<std::mutex> lock(build_mtx);
   return pnode->SaveMap(filename,vtLasers,0.05,15);
}


int CCartoSlam::RunFinalOptimization()
{
    // std::lock_guard<std::mutex> lock(build_mtx);
    if ( pnode == NULL ) {
        return -1;
    }
    if(m_status==MAP)
    {

        if(pnode->IsTrajectoryActive())
        {
          //  pnode->FinishTrajectory(0);
            pnode->RunFinalOptimization();
          return 1;
        }
        else
        {
            return 1;
        }


    }
    else
        return 1;


}

int CCartoSlam::StopMapping()
{

    // std::lock_guard<std::mutex> lock(build_mtx);
    if ( pnode == NULL ) {
        return -1;
    }
    if(m_status==MAP)
    {

        if(pnode->IsTrajectoryActive())
        {
            pnode->FinishTrajectory(0);

            pnode->RunFinalOptimization();  //?????????
          return 1;
        }
        else
        {
            return 1;
        }


    }
    else
        return 1;


}

void CCartoSlam::Reset()
{
    // std::lock_guard<std::mutex> lock(build_mtx);
    if ( pnode == NULL ) {
        return ;
    }
   // if(!pnode->IsInit())
    {
        pnode->Reset();
    }
}

void CCartoSlam::GetSubmapList(SubmapList &submap_list)
{
     //std::lock_guard<std::mutex> lock(build_mtx);
     if ( pnode == NULL ) {
         return ;
     }
    pnode->GetSubmapList(submap_list);
}

void CCartoSlam::GetSubmapData(const mapping::SubmapId submap_id,mapping::SubmapTexture *texture)
{
    if ( pnode == NULL ) {
        return ;
    }
    pnode->GetSubmapData(submap_id,texture);
}

void CCartoSlam::GetSubmapData_upload(const mapping::SubmapId submap_id,mapping::SubmapTexture_upload *texture, vector<double>& blackcell_index)
{
    if ( pnode == NULL ) {
        return ;
    }
    pnode->GetSubmapData_upload(submap_id,texture, blackcell_index);
}

mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> CCartoSlam::GetNodeData(void)
{
     std::lock_guard<std::mutex> lock(build_mtx);

     if ( pnode != NULL ) {
            return pnode->GetNodeData();

     }

}

mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::SubmapData> CCartoSlam::GetSubmapDataUnderLock() const
{
    return pnode->GetSubmapDataUnderLock();
}

std::vector<mapping::PoseGraphInterface::Constraint> CCartoSlam::GetConstraints() const
{
    // std::lock_guard<std::mutex> lock(build_mtx);
    if ( pnode != NULL ) {
        return pnode->GetConstraints();
    }
}
mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> CCartoSlam::GetTrajectoryNodePoses()
{
    // std::lock_guard<std::mutex> lock(build_mtx);
    if ( pnode != NULL ) {
        return pnode->GetTrajectoryNodePoses();
    }
}
mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::MySubmapData> CCartoSlam::GetMySubmapData() const
{
   //  std::lock_guard<std::mutex> lock(build_mtx);
    if ( pnode != NULL ) {
        return pnode->GetMySubmapData();
    }

}

int CCartoSlam::GetNumFinishedNodes()
{
     std::lock_guard<std::mutex> lock(build_mtx);
     if ( pnode != NULL ) {
        return pnode->GetNumFinishedNodes();
     }
}

map<int, vector<int>> CCartoSlam::GetPossibleConstraintPairs()
{
    if ( pnode != NULL ) {
     return pnode->GetPossibleConstraintPairs();
    }
}


void CCartoSlam::GetNodeData(std::vector<node_data> &nodeDatas)
{
     //std::lock_guard<std::mutex> lock(build_mtx);

    mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose>  trajectory_nodes_= GetTrajectoryNodePoses();
    for (const auto& node_id_data : trajectory_nodes_)
    {

        mapping::TrajectoryNodePose::ConstantPoseData constant_pose_data = 	node_id_data.data.constant_pose_data.value();

       // if (node_id_data.data.constant_pose_data != nullptr)
        {

            node_data data;

            data.node_id = node_id_data.id.node_index;
            data.stamp.tv_sec = constant_pose_data.time.Sec();
            data.stamp.tv_usec = constant_pose_data.time.Usec();

            double theta = transform::GetYaw(node_id_data.data.global_pose.rotation());
            Eigen::Matrix<double, 3, 1> xyz = node_id_data.data.global_pose.translation();

            data.pos.x = xyz(0);
            data.pos.y = xyz(1);
            data.pos.theta = theta;

            nodeDatas.push_back(data);
        }
    }

}
void CCartoSlam::GetSubmapData(int submap_id,submap_data *pdata)
{
    mapping::SubmapTexture texture;
    mapping::SubmapId _submapid;

    _submapid.trajectory_id = 0;
    _submapid.submap_index = submap_id;


    GetSubmapData((const mapping::SubmapId)_submapid,&texture);

    pdata->pixels =  texture.pixels;
    pdata->num_y_cells = texture.num_y_cells;
    pdata->num_x_cells = texture.num_x_cells;
    pdata->resolution =  texture.resolution;


    pdata->max_x = texture.max_x;
    pdata->max_y = texture.max_y;

    //transform::Rigid3d texture.slice_pose  // to pdata. pose;

    double theta = transform::GetYaw(texture.slice_pose.rotation());
    Eigen::Matrix<double, 3, 1> xyz = texture.slice_pose.translation();

    pdata->slice_pose.x = xyz(0);
    pdata->slice_pose.y =  xyz(1);
    pdata->slice_pose.theta = theta;
}

// dq upload_submaps
void CCartoSlam::GetSubmapData_upload(int submap_id, transform::Rigid3d &global_pose, submap_data *pdata,vector<double>& blackcell_index)
{
    mapping::SubmapTexture_upload texture;
    mapping::SubmapId _submapid;

    _submapid.trajectory_id = 0;
    _submapid.submap_index = submap_id;


    GetSubmapData_upload((const mapping::SubmapId)_submapid,&texture,blackcell_index);
    //double theta = transform::GetYaw(texture.slice_pose.rotation());
    Eigen::Matrix<double, 3, 1> xyz = texture.slice_pose.translation();

    transform::Rigid3d global_max = global_pose*texture.slice_pose;
    Eigen::Matrix<double, 3, 1> global_max_xy = global_max.translation();

    pdata->slice_pose.x = xyz(0);
    pdata->slice_pose.y =  xyz(1);
    pdata->slice_pose.theta = transform::GetYaw(global_max.rotation());;
    pdata->num_x_cells = texture.height;
    pdata->num_y_cells = texture.width;
    //std::cout<<"max_y: "<<blackcell_index.back()<<std::endl;
    //pdata->max_y = blackcell_index.back();
    pdata->max_y = global_max_xy(1);
    //blackcell_index.pop_back();
    //pdata->max_x = blackcell_index.back();
    pdata->max_x =  global_max_xy(0);
    //std::cout<<"max_x: "<<blackcell_index.back()<<std::endl;
    //blackcell_index.pop_back();
    std::cout<<"global_max_xy(0): "<<global_max_xy(0)<<", global_max_xy(1): "<<global_max_xy(1)<<std::endl;
}

bool CCartoSlam::StartLocate(Pose &initPos,std::unique_ptr<mapping::Grid2D> gridMap,double timestamp)
{
    Eigen::Vector2f pos(initPos.x, initPos.y);
    std::lock_guard<std::mutex> lock(build_mtx);

    if ( pnode == NULL ) {

        CreateNode();
    }
    if ( pnode == NULL ) {
        return false;
    }

    if(pnode->IsTrajectoryActive())
    {
        pnode->FinishTrajectory(0);
    }

    if(!pnode->IsInit())
    {
        pnode->Reset();
    }

    m_status = MAP;
     TrajectoryOptions trajectory_options = CreateTrajectoryOptions();
    trajectory_options.trajectory_builder_options.trajectory_builder_2d_options.submaps_options.grid_options_2d.resolution = m_metersPerPixel;

    trajectory_options.trajectory_builder_options.pure_localization = true;


    pnode->AddTrajectory(trajectory_options,pos,false,nullptr);

    const common::Time time(timestamp);
    Eigen::Matrix<double, 3, 1> tr( (double)initPos.x,  (double)initPos.y,0);
    Eigen::Quaternion<double> quat=transform::RollPitchYaw(0.0, 0.0,initPos.theta) ;

    //const transform::Rigid3d pose(tr, quat);

   // pnode->SetInitNodePose(time,pose);

    transform::Rigid2d pose = transform::Rigid2d({(double)initPos.x, (double)initPos.y}, 0.0);

    return true;
}
bool CCartoSlam::StopLocate()
{
    std::lock_guard<std::mutex> lock(build_mtx);
    if(m_status==MAP)
    {
         std::cout<< "CCartoSlam::StopLocate "<<std::endl;
        if(pnode->IsTrajectoryActive())
        {
            pnode->FinishTrajectory(0);

            return true;
        }
    }
    return false;
}

bool CCartoSlam::StartExpandMap(Pose &initPos,std::unique_ptr<mapping::Grid2D> gridMap,double timestamp)
{
    Eigen::Vector2f pos(initPos.x, initPos.y);
    std::lock_guard<std::mutex> lock(build_mtx);

    if ( pnode == NULL ) {

        CreateNode();
    }
    if ( pnode == NULL ) {
        return false;
    }

    if(pnode->IsTrajectoryActive())
    {
        pnode->FinishTrajectory(0);
    }

    if(!pnode->IsInit())
    {
        pnode->Reset();
    }

    m_status = MAP;
     TrajectoryOptions trajectory_options = CreateTrajectoryOptions();
    trajectory_options.trajectory_builder_options.trajectory_builder_2d_options.submaps_options.grid_options_2d.resolution = m_metersPerPixel;
    trajectory_options.trajectory_builder_options.pure_localization = false;

    pnode->AddTrajectory(trajectory_options,pos,true,std::move(gridMap));

    const common::Time time(timestamp);
    Eigen::Matrix<double, 3, 1> tr( (double)initPos.x,  (double)initPos.y,0);
    Eigen::Quaternion<double> quat=transform::RollPitchYaw(0.0, 0.0,initPos.theta) ;

    const transform::Rigid3d pose(tr, quat);

    pnode->SetInitNodePose(time,pose);


    return true;
}
/*void CCartoSlam::SetFirstSubmap(const transform::Rigid2d &global_submap_pose,std::unique_ptr<mapping::Grid2D> grid)
{
   pnode->SetFirstSubmap(global_submap_pose,std::move(grid));
    //std::shared_ptr<mapping::Submap2D> submap1= std::make_shared<mapping::Submap2D>(mapping::Submap2D(origin, std::move(grid)));
}*/


void CCartoSlam::SetInitNodePose(unsigned long long  timestamp,const Pose &initPos )
{

    long int sec = (long int)(timestamp/1000.0);  //ms   to   sec
    long int usec = (timestamp%1000)*1000;

    const common::Time time(sec,usec);
    Eigen::Matrix<double, 3, 1> tr( (double)initPos.x,  (double)initPos.y,0);
    Eigen::Quaternion<double> quat=transform::RollPitchYaw(0.0, 0.0,initPos.theta) ;

    const transform::Rigid3d pose(tr, quat);

    if(pnode!=NULL)
         pnode->SetInitNodePose(time,pose);
}
void CCartoSlam::SetInitNodePose(double  timestamp,const Pose &initPos )
{

    long int sec = (long int)timestamp;
    long int usec = (long int)((timestamp-sec)*1000000ll);


    const common::Time time(sec,usec);
    Eigen::Matrix<double, 3, 1> tr( (double)initPos.x,  (double)initPos.y,0);
    Eigen::Quaternion<double> quat=transform::RollPitchYaw(0.0, 0.0,initPos.theta) ;

    const transform::Rigid3d pose(tr, quat);

    if(pnode!=NULL)
         pnode->SetInitNodePose(time,pose);
}

void CCartoSlam::Clear()
{
    if(pnode!=NULL)
         pnode->Clear();
}
bool CCartoSlam::LoadBinary(char *filename)
{

    if ( pnode == NULL ) {

        CreateNode();
    }
    if ( pnode == NULL ) {
        return false;
    }

    if(pnode->IsTrajectoryActive())
    {
        pnode->FinishTrajectory(0);
    }

    if(!pnode->IsInit())
    {
        pnode->Reset();
    }

   // bool res = pnode->LoadBinary(fp);

   // fclose(fp);

    return true;
}

bool CCartoSlam::StartExpandMapnew(Pose &initPos,std::unique_ptr<mapping::Grid2D> gridMap,double timestamp)
{
    Eigen::Vector2f pos(initPos.x, initPos.y);
    std::lock_guard<std::mutex> lock(build_mtx);

    std::cout<<"StartExpandMapnew\n";
    std::cout << "initPos"<<initPos.x<<" "<<initPos.y<<" "<<initPos.theta<<std::endl;

    if ( pnode == NULL ) {

        CreateNode();
    }
    if ( pnode == NULL ) {
        return false;
    }
    if(pnode->IsTrajectoryActive())
    {  
        pnode->FinishTrajectory(0);
    }

    m_status = MAP;

    TrajectoryOptions trajectory_options = CreateTrajectoryOptions();
    trajectory_options.trajectory_builder_options.trajectory_builder_2d_options.submaps_options.grid_options_2d.resolution = m_metersPerPixel;
    trajectory_options.trajectory_builder_options.pure_localization = false;

    pnode->AddTrajectory(trajectory_options,pos,false,std::move(gridMap));

    const common::Time time(timestamp);
    Eigen::Matrix<double, 3, 1> tr( (double)initPos.x,  (double)initPos.y,0);
    Eigen::Quaternion<double> quat=transform::RollPitchYaw(0.0, 0.0,initPos.theta) ;

    const transform::Rigid3d pose(tr, quat);

    pnode->SetInitNodePose(time,pose);


    return true;
}
bool CCartoSlam::OpenExpandDx(FILE *fp)
{
    std::lock_guard<std::mutex> lock(build_mtx);

    if ( pnode == NULL ) {

        CreateNode();
    }
    if ( pnode == NULL ) {
        return false;
    }

    pnode->Reset();

    return pnode->LoadBinary(fp);


}
 bool CCartoSlam::SaveBinary(FILE *fp)
 {
     if ( pnode == NULL ) {
         return false;
     }

     bool bRet = pnode->SaveBinary(fp);

     return bRet;
 }
 bool CCartoSlam::RotateMap(double angle)
 {
     if(pnode!=NULL)
     {
          return pnode->RotateMap(angle);
     }
     else
         return false;
 }
 void CCartoSlam::SetFrozen(bool flag)
 {
     if(pnode!=NULL)
          pnode->SetFrozen(flag);
 }
 int  CCartoSlam::GetFrozenNodeNum()
 {
     if(pnode!=NULL)
          pnode->GetFrozenNodeNum();
 }
 int  CCartoSlam::GetFrozenSubmapNum()
 {
     if(pnode!=NULL)
          pnode->GetFrozenSubmapNum();
 }
