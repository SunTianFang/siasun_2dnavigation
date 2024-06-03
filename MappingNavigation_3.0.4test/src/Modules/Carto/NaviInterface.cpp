#include "NaviInterface.h"


#include "navigation.h"
#include <stdint.h>

#include <unistd.h>
#include <stdio.h>
#include <fstream>


#include <sys/time.h>

#if 1
#include <signal.h>
#include <execinfo.h>
#define BACKTRACE_BUF_SIZE          50

void ShmSigSegv_handler(int signal)
{
    int j, nptrs;
    void *buffer[BACKTRACE_BUF_SIZE];
    char **strings;

#if 1
    nptrs = backtrace(buffer, BACKTRACE_BUF_SIZE);
#else
    nptrs = backtrace_arm(buffer, BACKTRACE_BUF_SIZE);
#endif

    printf("signal_test exit:  backtrace() returned %d addresses, sig:%d\n", nptrs, signal);

    switch (signal) {
    case SIGFPE: // 8:
        printf("The signal type: Floating point exception!\n");
        break;
    case SIGILL: // 4:
        printf("The signal type: Illegal instruction!\n");
        break;
    case SIGSEGV: // 11:
        printf("The signal type: Segmentation fault!\n");
        break;
    case SIGBUS: // 7:
        printf("The signal type: Bus error!\n");
        break;
    case SIGABRT: // 6:
        printf("The signal type: Aborted!\n");
        break;
    case SIGTRAP: // 5:
        printf("The signal type: Trace/breakpoint trap!\n");
        break;
    case SIGSYS: // 31:
        printf("The signal type: Bad system call!\n");
        break;
    case SIGTERM: // 15:
        printf("The signal type: Terminated!\n");
        break;
    case SIGINT: // 2:
        printf("The signal type: Interrupt!\n");
        break;
    case SIGQUIT: // 3:
        printf("The signal type: Quit!\n");
        break;
    case SIGKILL: // 9:
        printf("The signal type: Killed!\n");
        break;
    case SIGHUP: // 1:
        printf("The signal type: Hangup!\n");
        break;
    case SIGALRM: // 14:
        printf("The signal type: Alarm clock!\n");
        break;
    case SIGVTALRM: // 26:
        printf("The signal type: Virtual timer expired!\n");
        break;
    case SIGPROF: // 27:
        printf("The signal type: Profiling timer expired!\n");
        break;
    case SIGIO: // 29:
        printf("The signal type: I/O possible!\n");
        break;
    case SIGPIPE: // 13:
        printf("The signal type: Broken pipe!\n");
        break;
    case SIGXCPU: // 24:
        printf("The signal type: CPU time limit exceeded!\n");
        break;
    case SIGXFSZ: // 25:
        printf("The signal type: File size limit exceeded!\n");
        break;
    case SIGUSR1: // 10:
        printf("The signal type: User defined signal 1!\n");
        break;
    case SIGUSR2: // 12:
        printf("The signal type: User defined signal 2!\n");
        break;
    default:
        printf("The signal type: Unkown signal:%d!\n", signal);
        break;
    }

    /* The call backtrace_symbols_fd(buffer, nptrs, STDOUT_FILENO)
    would produce similar output to the following: */

    strings = backtrace_symbols(buffer, nptrs);
    if (strings == NULL) {
        printf("backtrace_symbols");
        exit(EXIT_FAILURE);
    }

    for (j = 0; j < nptrs; j++) {
        printf("%s\n", strings[j]);
    }

    free(strings);
    exit(-1);
}

void SignalHandlerRegister_test()
{
    signal(SIGSEGV, ShmSigSegv_handler); // 段错误
    signal(SIGILL, ShmSigSegv_handler); // 非法指令
    signal(SIGBUS, ShmSigSegv_handler); // Bus error
    signal(SIGFPE, ShmSigSegv_handler); // 算术异常
    signal(SIGABRT, ShmSigSegv_handler); // 终止进程
    signal(SIGTRAP, ShmSigSegv_handler); // 跟踪、断点陷阱
    signal(SIGSYS, ShmSigSegv_handler); // 错误的系统调用
    signal(SIGTERM, ShmSigSegv_handler); // 终止进程
    signal(SIGINT, ShmSigSegv_handler); // 终端中断
    signal(SIGQUIT, ShmSigSegv_handler); // 终端退出
    signal(SIGKILL, ShmSigSegv_handler); // 杀死进程（必杀）
    signal(SIGHUP, ShmSigSegv_handler); // 挂起
    //signal(SIGALRM, ShmSigSegv_handler); // 定时器信号
    signal(SIGVTALRM, ShmSigSegv_handler); // 虚拟定时器超时
    signal(SIGPROF, ShmSigSegv_handler); // 性能分析定时器超时
    signal(SIGIO, ShmSigSegv_handler); // io时可能产生
    signal(SIGPIPE, ShmSigSegv_handler); // 管道断开
    signal(SIGXCPU, ShmSigSegv_handler); // 突破对cpu时间的限制
    signal(SIGXFSZ, ShmSigSegv_handler); // 突破对文件大小的限制
    signal(SIGUSR1, ShmSigSegv_handler); // 用户自定义信号1
    signal(SIGUSR2, ShmSigSegv_handler); // 用户自定义信号2
    signal(SIGSTKFLT, ShmSigSegv_handler); // 栈错误
    signal(SIGPWR, ShmSigSegv_handler); // 电量行将耗尽
    // signal(SIGCHLD, ShmSigSegv_handler); // 子进程退出：忽略
    // signal(SIGCONT, ShmSigSegv_handler); // 继续执行：若停止则继续执行
    // signal(SIGSTOP, ShmSigSegv_handler); // 停止执行（必停）：暂停执行
    // signal(SIGTSTP, ShmSigSegv_handler); // 停止：暂停执行
    // signal(SIGTTIN, ShmSigSegv_handler); // Stopped(tty input)：暂停执行
    // signal(SIGTTOU, ShmSigSegv_handler); // Stopped(tty output)：暂停执行
    // signal(SIGURG, ShmSigSegv_handler); // io紧急数据：忽略
    // signal(SIGWINCH, ShmSigSegv_handler); // 终端窗口尺寸发生变化：忽略
}





#endif




CCartoSlam *pNaviObj = NULL;
void Navi_Init(LaserNaviConfig config)
{

	
    //static NodeOptions node_options = CreateNodeOptions(config);
    //static TrajectoryOptions trajectory_options = CreateTrajectoryOptions();
		
   // static mapping::MapBuilder map_builder(node_options.map_builder_options);

    //static CNavigation navigation(node_options, &map_builder,trajectory_options);

    pNaviObj = new CCartoSlam();

    //SignalHandlerRegister();

}



void Navi_HandleLaserData(laserscan_msg *msg)
{

    if(pNaviObj!=NULL)
        pNaviObj->HandleLaserData(msg);

}

void Navi_HandleEncoderData(odometry_msg *msg)
{

    if(pNaviObj!=NULL)
        pNaviObj->HandleEncoderData(msg);


}

bool Navi_CreateMap(void)
{

	if(pNaviObj==NULL)
		return false;
	return pNaviObj->createMap();


}
int Navi_SaveMap(const string filename)
{

	if(pNaviObj==NULL)
		return -1;
	return pNaviObj->saveMap(filename);


}


bool Navi_StopMapping(void)
{
    if(pNaviObj==NULL)
        return false;
    return pNaviObj->StopMapping();
}


void Navi_AddLaserID(std::string laser_id)
{
	if(pNaviObj!=NULL)
		pNaviObj->AddLaserID(laser_id);

}


void Navi_RegisterSlamCallBack(SlamResultCbFunc pFunc)
{
	if(pNaviObj!=NULL)
		pNaviObj->RegisterSlamCallBack(pFunc);

}




void Navi_GetSubmapList(submap_list &submaps)
{
	if(pNaviObj!=NULL)
	{
		SubmapList _submaplist;
		pNaviObj->GetSubmapList(_submaplist);
	
		for(int i=0;i<_submaplist.submap.size();i++)
		{
			SubmapEntry submap = _submaplist.submap.at(i);
			submap_entry _submap;

			_submap.submap_index = (int)submap.submap_index;

			//change  transform::Rigid3d pose; to pose
			double theta = transform::GetYaw(submap.pose.rotation());
			Eigen::Matrix<double, 3, 1> xyz = submap.pose.translation();

			_submap.pose.x = xyz(0);
			_submap.pose.y =  xyz(1);
			_submap.pose.theta = theta;
		}


	}
}
void Navi_GetSubmapData(int submap_id,submap_data *pdata)
{
	if(pNaviObj!=NULL)
	{

		mapping::SubmapTexture texture;
		mapping::SubmapId _submapid;

		_submapid.trajectory_id = 0;
		_submapid.submap_index = submap_id;


		pNaviObj->GetSubmapData((const mapping::SubmapId)_submapid,&texture); 

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
}

void Navi_GetNodeData(std::vector<node_data> &nodeDatas)
{
    if(pNaviObj!=NULL)
    {

        mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose>  trajectory_nodes_= pNaviObj->GetTrajectoryNodePoses();
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



}

map<int, vector<int>> Navi_GetPossibleConstraintPairs()
{
    if(pNaviObj!=NULL)
    {
        return pNaviObj->GetPossibleConstraintPairs();
    }

}

/*
void Navi_SaveMapBy(const string filename,double _metersPerPixel, std::vector<sensor_msgs::LaserScan> &vt_laser_msg,double creatmaprange)
{


    //const auto& submap_data = optimization_problem_->submap_data();
    const auto& node_data = optimization_problem_->node_data();

    int kInitialProbabilityGridSize = 100;
    double resolution = _metersPerPixel;
    Eigen::Vector2d max = 0.5 * kInitialProbabilityGridSize * resolution * Eigen::Vector2d::Ones();

    mapping::ProbabilityGrid probmap( mapping::MapLimits(resolution, max.x(),max.y(),CellLimits(kInitialProbabilityGridSize,kInitialProbabilityGridSize)));


    proto::ProbabilityGridRangeDataInserterOptions2D options;
    options.hit_probability = 0.550000 ;
    options.miss_probability = 0.490000 ;
    options.insert_free_space = true;


    ProbabilityGridRangeDataInserter2D range_data_inserter_(options);

    int totalnum = node_data.size();
    int num = 0;
    int lastpercent = 0;

    transform::TransformInterpolationBuffer transform_interpolation_buffer;

   int hhh = 0;


    totalnum = vt_laser_msg.size();

    int begin = 0;

    if(totalnum>=2)
        begin = 1;


    for(int k = begin; k<vt_laser_msg.size();k++)
    {

        sensor_msgs::LaserScan  msg = vt_laser_msg.at(k);
        num++;

      //Eigen::Matrix<double, 3, 1> tf_vt(0.33, 0.0 ,0.0);
      Eigen::Quaternion<double> tf_quat(1.0, 0.0, 0.0, 0.0);

      transform::Rigid3d sensor_to_tracking(msg.laser_tf, tf_quat);

    sensor::PointCloudWithIntensities point_cloud;
    common::Time time;
        //将激光原始数据变成点云数据 点云中各点时间为  :最后一个点时间0，其他点为相对于最后一点的时间差，如最前一点约0.018 m


    LaserScanToPointCloudWithIntensities(msg, point_cloud, time );



        sensor::RangeData all_range_data;


        transform::Rigid3d tracking_to_map ;

        const common::Time msgtime = msg.header.stamp;

        if (!transform_interpolation_buffer.Has(msgtime)) {
             continue;
        }

        tracking_to_map =
                    transform_interpolation_buffer.Lookup(msgtime);



          for (size_t i = 0; i < point_cloud.points.size(); ++i) {



            const transform::Rigid3f sensor_to_map =
                (tracking_to_map * sensor_to_tracking).cast<float>();

            //std::cout<<"tracking_to_map"<<tracking_to_map.DebugString().c_str()<<std::endl;
            //std::cout<<"sensor_to_tracking"<<sensor_to_tracking.DebugString().c_str()<<std::endl;


            //points_batch->points.push_back(sensor_to_map *
            //	                           point_cloud.points[i].head<3>());

            // We use the last transform for the origin, which is approximately correct.
            //origin = sensor_to_map * Eigen::Vector3f::Zero();



                const Eigen::Vector3f hit_in_local = sensor_to_map *point_cloud.points[i].head<3>();

                const Eigen::Vector3f origin_in_local =sensor_to_map * Eigen::Vector3f::Zero();

                const Eigen::Vector3f delta = hit_in_local - origin_in_local;
                const float range = delta.norm();




                if (range >= 0.2)
                {
                  if (range <= creatmaprange)
                  {

                           all_range_data.returns.push_back(hit_in_local);
                  }
                  else
                  {

                         all_range_data.misses.push_back(origin_in_local + 5.0 / range * delta);
                  }
                }



          }

            if(all_range_data.returns.size() >0)
            {
                all_range_data.origin = tracking_to_map.cast<float>().translation();//mutable_trajectory_node.global_pose.cast<float>().translation();
                range_data_inserter_.Insert(all_range_data, &probmap);
            }

            //   std::ostringstream progress_info;
        //	progress_info << "Saving: " << std::fixed << std::setprecision(1)
          //        << 100.* ((double)(num)/(double)(totalnum))
            //      << "%...";
            //std::cout << "\r\x1b[K" << progress_info.str() << std::flush;


    }
     std::cout<<"save over   "<<std::endl;

    return probmap.saveMap(filename);

}
*/




