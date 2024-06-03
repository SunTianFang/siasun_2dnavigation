
#include "node.h"

#include <chrono>
#include <string>
#include <vector>
#include "Eigen/Core"

#include "make_unique.h"
#include "port.h"
#include "mytime.h"
#include "point_cloud.h"
#include "rigid_transform.h"
#include "transform.h"
#include "msg_conversion.h"


#include "mylogging.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>



#include "submap_2d.h"


using namespace myLogging;
using transform::Rigid3d;





Node::Node(
    const NodeOptions& node_options,
    mapping::MapBuilderInterface *map_builder)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, std::move(map_builder))
{


  common::MutexLocker lock(&mutex_);


  is_active_trajectory_ = false;
  is_init_ready = true;



}

Node::~Node() { 

  common::MutexLocker lock(&mutex_);
  if (is_active_trajectory_)
      FinishTrajectory(0); 

}



void Node::AddLaserID(std::string laser_id)
{
	vtLaserID.push_back(laser_id);
}


int Node::AddTrajectory(const TrajectoryOptions& options,Eigen::Vector2f &submap_initpos,bool bExpandMap,std::unique_ptr<mapping::Grid2D> gridMap) {

 
   common::MutexLocker lock(&mutex_);

	typedef mapping::TrajectoryBuilderInterface::SensorId SensorId;
 	typedef SensorId::SensorType SensorType;
  	std::set<SensorId> expected_sensor_ids ;

	
   	printf("AddTrajectory\n");	

	if (options.use_odometry){ 
	expected_sensor_ids.insert(
        SensorId{SensorType::ODOMETRY, std::string("odom")});  
	}
	if (options.trajectory_builder_options.trajectory_builder_2d_options.use_imu_data){
	expected_sensor_ids.insert(
        SensorId{SensorType::IMU,"imu"}); 
	}

	for(int i=0;i<vtLaserID.size();i++)
	{
		expected_sensor_ids.insert(
        	SensorId{SensorType::RANGE,std::string(vtLaserID.at(i))});  

	}


 
  const int trajectory_id = map_builder_bridge_.AddTrajectory(expected_sensor_ids, options,submap_initpos,bExpandMap,std::move(gridMap));


  is_active_trajectory_ = true;
  is_init_ready = false;



  return trajectory_id;
}



bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
 	return true;
 
}


//前面检查了一下是否可以关掉，指定id是否存在，是否已经被Finished了等情况后，如果一切正常，则停止订阅Topic、清除id及其他与该trajectory相关的量。最后调用map_builder_bridge_中的FinishTrajectory函数。
bool Node::FinishTrajectoryUnderLock(
    const int trajectory_id) {
 


  printf("FinishTrajectoryUnderLock trajectory_id=%d\n",trajectory_id);
  if (!is_active_trajectory_) {
    const std::string error = "Trajectory " + std::to_string(trajectory_id) +
                              " has already been finished.";
    LOG(ERROR) << error;

    return false;
  }



  map_builder_bridge_.FinishTrajectory(trajectory_id);

  is_active_trajectory_ = false;





  return true;
}

// add by lishen
void Node::Reset()
{

	common::MutexLocker lock(&mutex_);
	map_builder_bridge_.Reset();
    is_init_ready = true;
} 




bool Node::FinishTrajectory(const int trajectory_id) {
  common::MutexLocker lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id);
}

void Node::RunFinalOptimization() 
{

    common::MutexLocker lock(&mutex_);

     std::cout<<"RunFinalOptimization "<<is_active_trajectory_<<std::endl;
  
   // CHECK(!is_active_trajectory_);

    if(is_active_trajectory_)
          map_builder_bridge_.RunFinalOptimization();
}
// add by lishen
int Node::SaveMap(const string filename, double _metersPerPixel,double creatmaprange)
{
    common::MutexLocker lock(&mutex_);
  
    CHECK(!is_active_trajectory_);
   
	   return map_builder_bridge_.SaveMap(filename,_metersPerPixel,creatmaprange);

}
// add by lishen
int Node::SaveMap(const string filename, std::vector<sensor_msgs::LaserScan> &vtLasers,double _metersPerPixel,double creatmaprange)
{
    common::MutexLocker lock(&mutex_);

    CHECK(!is_active_trajectory_);

       return map_builder_bridge_.SaveMap(filename,vtLasers,_metersPerPixel,creatmaprange);

}



void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const sensor_msgs::OdometryProto &msg) {



  common::MutexLocker lock(&mutex_);


	if(is_active_trajectory_==false)
		return;




 // auto odometry_data_ptr = map_builder_bridge_.ToOdometryData(&msg);
 // if (odometry_data_ptr != nullptr) {
 //   extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
 // }

  map_builder_bridge_.HandleOdometryMessage(sensor_id, &msg);





}

 std::shared_ptr<const MapBuilderBridge::TrajectoryState::LocalSlamData> Node::GetLocalSlamData(int trajectory_id)
{

	common::MutexLocker lock(&mutex_);
	return map_builder_bridge_.GetLocalSlamData(trajectory_id);

}

mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> Node::GetNodeData(void)
{

    common::MutexLocker lock(&mutex_);
    return map_builder_bridge_.GetNodeData();
}

void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::Imu &msg) {



  common::MutexLocker lock(&mutex_);

	if(is_active_trajectory_==false)
		return;


 // if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
  //  return;
 // }

  auto imu_data_ptr = map_builder_bridge_.ToImuData(msg);

 // if (imu_data_ptr != nullptr) {
  //  extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
 // }
  map_builder_bridge_.HandleImuMessage(sensor_id, msg);
}
//订阅之后的处理是在Node::HandleLaserScanMessage，查看该代码就可以发现最后依然交给了map_builder_bridge_去处理


  void Node::HandleLaserScanMessage(sensor::PointCloud &pointclouds)
  {

      common::MutexLocker lock(&mutex_);



      if(is_active_trajectory_==false)
          return;

       map_builder_bridge_.HandleLaserScanMessage(pointclouds);

  }

void Node::HandleLaserScanMessage(bool bUpdatemap,
                                   sensor_msgs::LaserScan  &msg,transform::Rigid2d locpos,common::Time loctime,transform::Rigid2d &offset) {

// add by lishen
  	common::MutexLocker lock(&mutex_);

	if(is_active_trajectory_==false)
		return;

  //  std::cout<<"HandleLaserScanMessage \n";

	map_builder_bridge_.SetUpdateMapFlag(bUpdatemap);

    map_builder_bridge_.HandleLaserScanMessage(&msg);


 // if (map_builder_bridge_.ShouldMoveSubmaps()) 

/*	if(bUpdatemap)
   {

		int trajectory_id = 0;
		std::shared_ptr<const MapBuilderBridge::TrajectoryState::LocalSlamData> localdata = map_builder_bridge_.GetLocalSlamData(trajectory_id);

		if(localdata!=nullptr)
		{
			 
			common::Time slamtime = localdata->time;

			// double delay = common::Time::now() - loctime; //

			//if(delay>0.3)
			//	return;


			double theta = transform::GetYaw(localdata->local_pose.rotation());
			Eigen::Matrix<double, 3, 1> xyz = localdata->local_pose.translation();
		
			transform::Rigid2d  slam = transform::Rigid2d({xyz(0), xyz(1)},theta);



			// std::cout<<"locpos   "<<locpos.DebugString()<<std::endl;

			//	printf(" slam  %f  %f  %f\n",xyz(0), xyz(1),theta);


			offset= locpos*slam.inverse();


		  // std::cout<<"node offset    "<<offset.DebugString()<<std::endl;
		     //map_builder_bridge_.MoveSubmaps(offset);
		}

  }
*/

}
void Node::GetSubmapList(SubmapList &submap_list)                    
{ 
    //common::MutexLocker lock(&mutex_);
	map_builder_bridge_.GetSubmapList(submap_list);
  
}



void Node::GetSubmapData(const ::mapping::SubmapId submap_id,::mapping::SubmapTexture *texture) 
{
	common::MutexLocker lock(&mutex_);
	return map_builder_bridge_.GetSubmapData(submap_id,texture);

} 

void Node::GetSubmapData_upload(const ::mapping::SubmapId submap_id,::mapping::SubmapTexture_upload *texture, vector<double>& blackcell_index)
{
    //common::MutexLocker lock(&mutex_);
    return map_builder_bridge_.GetSubmapData_upload(submap_id,texture, blackcell_index);
}

// add by lishen
mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::SubmapData> Node::GetSubmapDataUnderLock() const
{
    return map_builder_bridge_.GetSubmapDataUnderLock();
}
// add by lishen
std::vector<mapping::PoseGraphInterface::Constraint> Node::GetConstraints() const
{
    return map_builder_bridge_.GetConstraints();
}
// add by lishen
mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose>  Node::GetTrajectoryNodePoses()
{
    return map_builder_bridge_.GetTrajectoryNodePoses();
}
// add by lishen
mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::MySubmapData> Node::GetMySubmapData() const
{
    return map_builder_bridge_.GetMySubmapData();
}
// add by lishen
int Node::GetNumFinishedNodes()
{
    return map_builder_bridge_.GetNumFinishedNodes();
}
// add by lishen
map<int, vector<int>> Node::GetPossibleConstraintPairs()
{
   return map_builder_bridge_.GetPossibleConstraintPairs();
}
// add by lishen
void Node::SetFirstSubmap(const transform::Rigid2d &global_submap_pose,std::unique_ptr<mapping::Grid2D> grid)
{
    map_builder_bridge_.SetFirstSubmap(global_submap_pose,std::move(grid));
    //std::shared_ptr<mapping::Submap2D> submap1= std::make_shared<mapping::Submap2D>(mapping::Submap2D(origin, std::move(grid)));
}
// add by lishen
void Node::SetInitNodePose(const common::Time time,const transform::Rigid3d &pose)
{
    map_builder_bridge_.SetInitNodePose(time,pose);
}
// add by lishen
void Node::Clear()
{
    map_builder_bridge_.Clear();
}
// add by lishen
bool Node::LoadBinary(FILE *fp)
{
    return map_builder_bridge_.LoadBinary(fp);
}

bool Node::SaveBinary(FILE *fp)
{
    return map_builder_bridge_.SaveBinary(fp);
}
bool Node::RotateMap(double angle)
{
     return map_builder_bridge_.RotateMap(angle);
}
void Node::SetFrozen(bool flag)
{
    map_builder_bridge_.SetFrozen(flag);
}
int  Node::GetFrozenNodeNum()
{
    return map_builder_bridge_.GetFrozenNodeNum();
}
int  Node::GetFrozenSubmapNum()
{
    return map_builder_bridge_.GetFrozenSubmapNum();
}

