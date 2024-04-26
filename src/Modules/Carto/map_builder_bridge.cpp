
#include "map_builder_bridge.h"

#include "make_unique.h"
#include "pose_graph.h"

#include "rigid_transform.h"

#include "msg_conversion.h"
#include "id.h"
#include "pose_graph_interface.h"








using transform::Rigid3d;

namespace  {




double tfCos(double x) { return cos(x); }
double tfSin(double x) { return sin(x); }
double tfTan(double x) { return tan(x); }


constexpr float kPointCloudComponentFourMagic = 1.;



Eigen::Quaterniond ToEigen(const sensor_msgs::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

Rigid3d ToRigid3d( const sensor_msgs::Pose& pose) {
  return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                 ToEigen(pose.orientation));
}


Eigen::Vector3d ToEigen(const sensor_msgs::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}




// For sensor_msgs::LaserScan.
bool HasEcho(float) { return true; }

float GetFirstEcho(float range) { return range; }



Eigen::Quaternion<double> createQuaternionMsgFromYaw(double yaw)
{
 
 	

	double roll = 0.0;
	double pitch = 0.0;
	
    double halfYaw = double(yaw) * double(0.5);  
    double halfPitch = double(pitch) * double(0.5);  
    double halfRoll = double(roll) * double(0.5);  
    double cosYaw = tfCos(halfYaw);
    double sinYaw = tfSin(halfYaw);
    double cosPitch = tfCos(halfPitch);
    double sinPitch = tfSin(halfPitch);
    double cosRoll = tfCos(halfRoll);
    double sinRoll = tfSin(halfRoll);

    double x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw; //x
 	double y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw; //y
    double z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw; //z
    double w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw; //formerly yzx
	Eigen::Quaternion<double>  q(w,x,y,z);

 	return q;



} 
// For sensor_msgs::LaserScan and sensor_msgs::MultiEchoLaserScan.
template <typename LaserMessageType>

void LaserScanToPointCloudWithIntensities(const LaserMessageType &msg ,sensor::PointCloudWithIntensities &point_cloud ,common::Time &timestamp) 
{
  CHECK_GE(msg.range_min, 0.f);
  CHECK_GE(msg.range_max, msg.range_min);
  if (msg.angle_increment > 0.f) {
    CHECK_GT(msg.angle_max, msg.angle_min);
  } else {
    CHECK_GT(msg.angle_min, msg.angle_max);
  }
 //PointCloudWithIntensities point_cloud;
  float angle = msg.angle_min;




  	for (size_t i = 0; i < msg.ranges.size(); ++i) 
  	{
		const auto& echoes = msg.ranges[i];

	   if (HasEcho(echoes)) 
       {
		  const float first_echo = GetFirstEcho(echoes);




			  if (msg.range_min <= first_echo && first_echo <= msg.range_max) 
		      {
				const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
				Eigen::Vector4f point;
				point << rotation * (first_echo * Eigen::Vector3f::UnitX()),
				    i * msg.time_increment;
				point_cloud.points.push_back(point);
				if (msg.intensities.size() > 0) 
		        {
				  CHECK_EQ(msg.intensities.size(), msg.ranges.size());
				  const auto& echo_intensities = msg.intensities[i];
				  CHECK(HasEcho(echo_intensities));
				  point_cloud.intensities.push_back(GetFirstEcho(echo_intensities));
				} 
		        else 
		        {
				  point_cloud.intensities.push_back(0.f);
				}
			  }
		}
		angle += msg.angle_increment;
	}

	timestamp = msg.header.stamp;
	
	if (!point_cloud.points.empty()) 
	{
		const double duration = point_cloud.points.back()[3];
		//timestamp += common::FromSeconds(duration);

		timestamp = timestamp + duration;
		
		
		for (Eigen::Vector4f& point : point_cloud.points) 
        {
		  point[3] -= duration;            //  s

	
		}
	}


}




void ToPointCloudWithIntensities(const sensor_msgs::LaserScan &msg,sensor::PointCloudWithIntensities &pointCound, common::Time &time) {
    LaserScanToPointCloudWithIntensities(msg, pointCound, time);
}


}  // namespace






//构造函数里只是做了一下实例化赋值：
MapBuilderBridge::MapBuilderBridge(
    const NodeOptions& node_options,
    mapping::MapBuilderInterface *map_builder)
    : node_options_(node_options),
      map_builder_(map_builder)
{
		is_trajectory_active = false;
    last_trajectory_active = false;
		localSlamData = nullptr;
		m_bupdatemap = false;
    m_printfwarn = true;

}



//添加一条Trajectory
int MapBuilderBridge::AddTrajectory(
    const std::set<mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& trajectory_options,
        Eigen::Vector2f &submap_initpos,bool bExpandMap,
        std::unique_ptr<mapping::Grid2D> gridMap)
{



  const int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options.trajectory_builder_options,
      std::bind(&MapBuilderBridge::OnLocalSlamResult, this,
                  std::placeholders::_1, ::std::placeholders::_2,
                  std::placeholders::_3, ::std::placeholders::_4,
                  std::placeholders::_5,std::placeholders::_6),submap_initpos,bExpandMap,std::move(gridMap));
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";
   

  num_subdivisions_per_laser_scan_ = trajectory_options.num_subdivisions_per_laser_scan;

  trajectory_options_ = trajectory_options;
 
  is_trajectory_active = true;

  m_printfwarn = true;

  vt_laser_msg.clear();

    
  return trajectory_id;
}

void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {

  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  // Make sure there is a trajectory with 'trajectory_id'.
 
  m_printfwarn = false;

  if(!is_trajectory_active)
 	return;

  map_builder_->FinishTrajectory(trajectory_id);
  LOG(INFO) << "jj  Finishing trajectory with ID '" << trajectory_id << "'...";

 
}



void MapBuilderBridge::Reset()
{
  common::MutexLocker lock(&mutex_);
  map_builder_->Reset();
  sensor_to_previous_subdivision_time_.clear();

}


void MapBuilderBridge::RunFinalOptimization() {
  LOG(INFO) << "Running final trajectory optimization...";
  map_builder_->pose_graph()->RunFinalOptimization();



}

mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> MapBuilderBridge::GetNodeData()
{
    return map_builder_->GetTrajectoryNodePoses();


}


// add by lishen
int MapBuilderBridge::SaveMap(const string filename,std::vector<sensor_msgs::LaserScan> &vtLasers,double _metersPerPixel,double creatmaprange)
{
	LOG(INFO) << "SaveMap...";
    std::cout<<"SaveMap..."<<std::endl;

    return map_builder_->pose_graph()->SaveMap(filename,_metersPerPixel,vtLasers,creatmaprange);

}
// add by lishen
int MapBuilderBridge::SaveMap(const string filename,double _metersPerPixel,double creatmaprange)
{
    LOG(INFO) << "SaveMap...";


    return map_builder_->pose_graph()->SaveMap(filename,_metersPerPixel,vt_laser_msg,creatmaprange);

}


// add by lishen
void MapBuilderBridge::OnLocalSlamResult(
    const int trajectory_id, const common::Time time,
    const Rigid3d local_pose,
    sensor::RangeData range_data_in_local,
    const std::unique_ptr<const mapping::
                              TrajectoryBuilderInterface::InsertionResult>,double realtimescore) {


	//printf("printf ( %f  %f   )\n", local_pose.translation().x,local_pose.translation().y);

  	std::shared_ptr< TrajectoryState::LocalSlamData> local_slam_data =
      std::make_shared<TrajectoryState::LocalSlamData>(
          TrajectoryState::LocalSlamData{time, local_pose,
                                         std::move(range_data_in_local)});
  	common::MutexLocker lock(&mutex_);

  //trajectory_state_data_[trajectory_id] = std::move(local_slam_data);


  	localSlamData = (local_slam_data);


    int isaddnode = trajectory_id;
    bool is_first = false;

    if(last_trajectory_active==false&& isaddnode)
     {
       is_first=true;
       last_trajectory_active = true;
     } 


   

    mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> node_poses = map_builder_->GetTrajectoryNodePoses();


	//if(!m_bupdatemap)
	{
		double theta = transform::GetYaw(local_pose.rotation());
		Eigen::Matrix<double, 3, 1> xyz = local_pose.translation();	
    m_plocalposCbFunc(&isaddnode,&time,&local_pose,&local_slam_data->range_data_in_local, &m_bupdatemap,&is_first,&node_poses,&realtimescore);
	}
   
}



// add by lishen
std::shared_ptr<const MapBuilderBridge::TrajectoryState::LocalSlamData> MapBuilderBridge::GetLocalSlamData(int trajectory_id){

  common::MutexLocker lock(&mutex_);


	return localSlamData;


}


/*
void MapBuilderBridge::HandleOdometryMessage(
    const std::string& sensor_id, const sensor_msgs::Odometry *msg) 
{
  std::unique_ptr<sensor::OdometryData> odometry_data = ToOdometryData(msg);

  if (odometry_data != nullptr) {
    trajectory_builder_->AddSensorData(
        sensor_id,
        sensor::OdometryData{odometry_data->time, odometry_data->pose});
  }
}




std::unique_ptr<sensor::OdometryData> MapBuilderBridge::ToOdometryData(const sensor_msgs::Odometry *msg) 
{
  const common::Time time = FromRos(msg->header.stamp);


  Eigen::Matrix<double, 3, 1> tf_vt(0.0, 0.0 ,0.0);
  Eigen::Quaternion<double> tf_quat(1.0, 0.0, 0.0, 0.0);

 
  Rigid3d sensor_to_tracking(tf_vt, tf_quat);


  return common::make_unique<sensor::OdometryData>(
      sensor::OdometryData{
          time, ToRigid3d(msg->pose.pose) * sensor_to_tracking.inverse()});
}
*/

// add by lishen
std::unique_ptr<sensor::OdometryData> MapBuilderBridge::ToOdometryData(
    const sensor_msgs::OdometryProto *msg) {
  const common::Time time = msg->stamp;


  Eigen::Matrix<double, 3, 1> tf_vt(0.0, 0.0 ,0.0);
  Eigen::Quaternion<double> tf_quat(1.0, 0.0, 0.0, 0.0);

 
 
  Rigid3d sensor_to_tracking(tf_vt, tf_quat);

  Eigen::Matrix<double, 3, 1> pose;
  pose(0) = msg->x;
  pose(1) = msg->y;
  pose(2) = 0.0;


  Eigen::Quaternion<double> quat;
  quat = createQuaternionMsgFromYaw(msg->theta);



	//Eigen::Matrix<double, 3, 1> m = transform::RotationQuaternionToAngleAxisVector(quat);
 
  //std::cout << m;


  Rigid3d encoder(pose, quat);

 
  return common::make_unique<sensor::OdometryData>(
     sensor::OdometryData{

          time, encoder * sensor_to_tracking.inverse()});
}


void MapBuilderBridge::HandleOdometryMessage(
    const std::string& sensor_id, const sensor_msgs::OdometryProto *msg) 
{
  if(!is_trajectory_active)
 	return;

// printf("HandleOdometryMessage \n");
  std::unique_ptr<sensor::OdometryData> odometry_data = ToOdometryData(msg);




//	double ti = time0.tv_sec*1000.0+(double)time0.tv_usec/1000.0;

	


  if (odometry_data != nullptr) {
    map_builder_->AddSensorData(
        sensor_id,
        sensor::OdometryData{odometry_data->time, odometry_data->pose});
  }
	

}



void MapBuilderBridge::HandleLaserScanMessage(sensor::PointCloud &pointclouds)
{

    if(!is_trajectory_active)
      return;



}


void MapBuilderBridge::HandleLaserScanMessage(sensor_msgs::LaserScan *msg) {

  if(!is_trajectory_active)
 	return;

// add by lishen

	double lasertime = 	msg->header.stamp.ToSecond();




    //timeval time0;
    //timeval time1;
    //gettimeofday(&time0,NULL);


   // vt_laser_msg.push_back(*msg);   //gai ????


  sensor::PointCloudWithIntensities point_cloud;
  common::Time time;
//将激光原始数据变成点云数据 点云中各点时间为  :最后一个点时间0，其他点为相对于最后一点的时间差，如最前一点约0.018 m



  ToPointCloudWithIntensities(*msg, point_cloud, time );
	
  HandleLaserScan(msg->header.stamp,msg->sensor_id,msg->laser_tf, time,point_cloud);


    //gettimeofday(&time1,NULL);

    //double deltime = (time1.tv_sec - time0.tv_sec)*1000 + (double)(time1.tv_usec -time0.tv_usec)/1000 ;
 	
	//if(deltime>10)	
   // printf(" &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&encoder2  deltime = %f\n", deltime ); 

}


void MapBuilderBridge::HandleLaserScan(const common::Time msgtime,
    const std::string& sensor_id, Eigen::Matrix<double, 3, 1> laser_tf,const common::Time time,
    const sensor::PointCloudWithIntensities& points)
{


    //std::cout<<"enter HandleLaserScan\n"<<std::endl;

  if (points.points.empty()) {
    return;
  }
  CHECK_LE(points.points.back()[3], 0);


 // std::cout<<"num_subdivisions_per_laser_scan_ = "<<num_subdivisions_per_laser_scan_<<std::endl;

  // TODO(gaschler): Use per-point time instead of subdivisions.

//num_subdivisions_per_laser_scan = 10
  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) 
  {
    const size_t start_index =
        points.points.size() * i / num_subdivisions_per_laser_scan_;
    const size_t end_index =
        points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
    sensor::TimedPointCloud subdivision(                                         //typedef std::vector<Eigen::Vector4f> TimedPointCloud;
        points.points.begin() + start_index, points.points.begin() + end_index);
    if (start_index == end_index) {

        std::cout << "start_index == end_index"<<std::endl;
      continue;
    }

	///  subdivision   10

    const double time_to_subdivision_end = subdivision.back()[3];
    // `subdivision_time` is the end of the measurement so sensor::Collator will
    // send all other sensor data first.
    const common::Time subdivision_time =
               time + (time_to_subdivision_end);
    auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
    if (it != sensor_to_previous_subdivision_time_.end() &&
        it->second >= subdivision_time) {


      //if laser no timestamp 

      if(m_printfwarn==true)
      {

        printf("WARNING: laser timestamp ???     ")  ; 

        LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                   << sensor_id << " because previous subdivision time "
                   << it->second << " is not before current subdivision time "
                   << subdivision_time;  

        std::cout << "Ignored subdivision of a LaserScan message from sensor "
                   << sensor_id << " because previous subdivision time "
                   << it->second << " is not before current subdivision time "
                   << subdivision_time<<std::endl;



        m_printfwarn = false;
      }

      continue;
    }
    sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
    for (Eigen::Vector4f& point : subdivision) {
      point[3] -= time_to_subdivision_end;
    }
    CHECK_EQ(subdivision.back()[3], 0);




    HandleRangefinder(sensor_id, laser_tf,msgtime, subdivision_time,subdivision);


	double subtime = 	subdivision_time.ToSecond();

    //printf("subdivision_time = %f  \n",subtime);



  }

	

}

void MapBuilderBridge::HandleRangefinder(
    const std::string& sensor_id, Eigen::Matrix<double, 3, 1> laser_tf, common::Time msgtime, const common::Time time,
    const sensor::TimedPointCloud& ranges)

{


  Eigen::Matrix<double, 3, 1> tf_vt(laser_tf(0), laser_tf(1) ,0.0);
 // Eigen::Quaternion<double> tf_quat(1.0, 0.0, 0.0, 0.0);

 
  Eigen::Quaterniond tf_quat = transform::RollPitchYaw(0, 0,laser_tf(2)) ;

  Rigid3d sensor_to_tracking(tf_vt, tf_quat);

 

  Eigen::Vector3f origin = sensor_to_tracking.translation().cast<float>();



  string str = sensor_to_tracking.DebugString();





  // change pointcloud  to robot frame

  //将 点云转为机器人坐标系    origin 为激光在机器人坐标系位置
    map_builder_->AddSensorData(
        sensor_id, sensor::TimedPointCloudData{
                      msgtime, time, sensor_to_tracking.translation().cast<float>(),
                       sensor::TransformTimedPointCloud(
                           ranges, sensor_to_tracking.cast<float>())});
   


}



std::unique_ptr<sensor::ImuData> MapBuilderBridge::ToImuData(
    const sensor_msgs::Imu  &msg) {


  if(!is_trajectory_active)
 	return nullptr;

//确保IMU工作正常
  CHECK_NE(msg.linear_acceleration_covariance[0], -1)
      << "Your IMU data claims to not contain linear acceleration measurements "
         "by setting linear_acceleration_covariance[0] to -1. slam "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  CHECK_NE(msg.angular_velocity_covariance[0], -1)
      << "Your IMU data claims to not contain angular velocity measurements "
         "by setting angular_velocity_covariance[0] to -1. slam "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";



const common::Time time = msg.header.stamp;
  Eigen::Matrix<double, 3, 1> tf_vt(0.0, 0.0 ,0.0);     //  &&&&& IMU pose
  Eigen::Quaternion<double> tf_quat(1.0, 0.0, 0.0, 0.0);


  Rigid3d sensor_to_tracking(tf_vt, tf_quat);

  return common::make_unique<sensor::ImuData>(
      sensor::ImuData{
          time,
          sensor_to_tracking.rotation() * ToEigen(msg.linear_acceleration),
          sensor_to_tracking.rotation() * ToEigen(msg.angular_velocity)});



}
//最终，将先加速度和角加速度传入trajectory_builder_->AddSensorData做处理


void MapBuilderBridge::HandleImuMessage(const std::string& sensor_id,
                                    const sensor_msgs::Imu  &msg) {
  std::unique_ptr<sensor::ImuData> imu_data = ToImuData(msg);
  if (imu_data != nullptr) {
    map_builder_->AddSensorData(
        sensor_id,
       sensor::ImuData{imu_data->time, imu_data->linear_acceleration,
                               imu_data->angular_velocity});
  }
}



// add by lishen
void MapBuilderBridge::GetSubmapList(SubmapList &submap_list)
{

     //common::MutexLocker lock(&mutex_);

  	submap_list.header.stamp = common::Time::now();
  //	submap_list.header.frame_id = node_options_.map_frame;

	mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::SubmapPose> submap_poses;

  for (const auto& submap_id_pose :
       map_builder_->pose_graph()->GetAllSubmapPoses()) {


    SubmapEntry submap_entry;
    submap_entry.trajectory_id = submap_id_pose.id.trajectory_id;
    submap_entry.submap_index = submap_id_pose.id.submap_index;
    submap_entry.submap_version = submap_id_pose.data.version;
    submap_entry.pose = (submap_id_pose.data.pose);
    submap_list.submap.push_back(submap_entry);
  }



  //return ;
}


mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> MapBuilderBridge::GetTrajectoryNodePoses()
{

  // common::MutexLocker lock(&mutex_);
   return map_builder_->GetTrajectoryNodePoses();
}



mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::SubmapData> MapBuilderBridge::GetSubmapDataUnderLock() const
{
     return map_builder_->GetSubmapDataUnderLock();
}
// add by lishen
std::vector<mapping::PoseGraphInterface::Constraint> MapBuilderBridge::GetConstraints() const
{
     return map_builder_->GetConstraints();
}
// add by lishen
mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::MySubmapData> MapBuilderBridge::GetMySubmapData() const
{
    return map_builder_->GetMySubmapData();
}

// add by lishen
void MapBuilderBridge::GetSubmapData(const mapping::SubmapId submap_id,mapping::SubmapTexture *texture)
{
	 common::MutexLocker lock(&mutex_);
	return map_builder_->GetSubmapData(submap_id,texture);

}

void MapBuilderBridge::GetSubmapData_upload(const mapping::SubmapId submap_id,mapping::SubmapTexture_upload *texture, vector<double>& blackcell_index)
{
    // common::MutexLocker lock(&mutex_);
    return map_builder_->GetSubmapData_upload(submap_id,texture,blackcell_index);

}

int MapBuilderBridge::GetNumFinishedNodes()
{
    return map_builder_->GetNumFinishedNodes();
}
// add by lishen
map<int, vector<int>> MapBuilderBridge::GetPossibleConstraintPairs()
{
    common::MutexLocker lock(&mutex_);

    struct IdDis
    {
        int id;
        float dis;
    };

    mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> trajectory_nodes_ = map_builder_->GetTrajectoryNodePoses();

    map<int, vector<int>> consPair;

    mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::MySubmapData> submap_data_=map_builder_->GetMySubmapData();

    std::vector<mapping::PoseGraphInterface::Constraint> vtCartoConstraint = map_builder_->GetConstraints();




    std::cout<<"trajectory_nodes_.size() "<<trajectory_nodes_.size()<<"submap "<< submap_data_.size()<<std::endl;



    for (mapping::PoseGraphInterface::Constraint constraint : vtCartoConstraint)
    {

        transform::Rigid3d global_pose1 = trajectory_nodes_.at(constraint.node_id).global_pose;

        std::cout<<"constraint node "<<constraint.node_id.node_index<<"submap "<< constraint.submap_id.submap_index<<std::endl;


        for (const auto& submap_id_data : submap_data_)
        {

            if( constraint.submap_id == submap_id_data.id)
            {
                         //  std::cout<<"constraint find submap "<<submap_id_data.id<< std::endl;
                 vector<IdDis> submapIds;

                 for(const auto& node_id_insubmap : submap_id_data.data.node_ids)
                 {

                     transform::Rigid3d global_pose2 = trajectory_nodes_.at(node_id_insubmap).global_pose;
                     double x1 = global_pose1.translation().x();
                     double y1 = global_pose1.translation().y();
                     double x2 = global_pose2.translation().x();
                     double y2 = global_pose2.translation().y();

                     double dis = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));

                      std::cout<<"constraint find node in submap "<<node_id_insubmap.node_index<< std::endl;

                     if(dis>25)
                          continue;
                    if(constraint.node_id.node_index ==node_id_insubmap.node_index )
                    {
                           submapIds.clear();
                           break;
                    }


                   if(constraint.node_id.node_index <=node_id_insubmap.node_index )
                       continue;

                   IdDis tmp;
                   tmp.id = node_id_insubmap.node_index;
                   tmp.dis = dis;
                   if(submapIds.size()<=0)
                   {
                         submapIds.push_back(tmp);
                   }
                   else
                   {
                         vector<IdDis>::iterator iter;
                         bool insert = false;
                         for(iter = submapIds.begin();iter!=submapIds.end();iter++)
                         {
                              IdDis cur= *iter;
                              if(dis<(cur.dis))
                              {
                                   submapIds.insert(iter,tmp);
                                   insert = true;
                                   break;
                               }
                         }
                         if(!insert)
                             submapIds.push_back(tmp);
                   }

             }

              if(submapIds.size()>0)
              {
                  vector<int> ids;
                  for(auto sids :submapIds)
                      ids.push_back(sids.id);
                  consPair[constraint.node_id.node_index] = ids;
              }
            }

         }

    }

    return consPair;
}
// add by lishen
void MapBuilderBridge::SetInitNodePose(const common::Time time,const transform::Rigid3d &pose)
{

    map_builder_->SetInitNodePose(time,pose);
}
// add by lishen
void MapBuilderBridge::SetFirstSubmap(const transform::Rigid2d &global_submap_pose,std::unique_ptr<mapping::Grid2D> grid)
{

    map_builder_->pose_graph()->SetFirstSubmap(global_submap_pose,std::move(grid));

}
// add by lishen
void MapBuilderBridge::Clear()
{
    map_builder_->ClearSubmapAndNode();
}
bool MapBuilderBridge::LoadBinary(FILE *fp)
{

    map_builder_->pose_graph()->Reset();
    return map_builder_->pose_graph()->LoadBinary(fp);
}

bool MapBuilderBridge::SaveBinary(FILE *fp)
{
   return map_builder_->pose_graph()->SaveBinary(fp);
}
bool MapBuilderBridge::RotateMap(double angle)
{
    return map_builder_->pose_graph()->RotateMap(angle);
}

void MapBuilderBridge::SetFrozen(bool flag)
{
    map_builder_->pose_graph()->SetFrozen(flag);
}

int MapBuilderBridge::GetFrozenNodeNum()
{
     map_builder_->pose_graph()->GetFrozenNodeNum();
}

int MapBuilderBridge::GetFrozenSubmapNum()
{
     map_builder_->pose_graph()->GetFrozenSubmapNum();
}



