#pragma once




////////////////////////////////////////////////
//  实现cartographer 建图应用 接口的设计
//   Author: lishen
//   Date:   2022.4
///////////////////////////////////////////////

#include "type.h"
#include "node.h"
#include "map_builder.h"


#include <mutex>


using namespace std;


typedef void *LPVOID;

typedef enum  emNaviStatus
{
	INVALID,
	LOCATION,
        MAP,
	//UPDATEMAP,
};

/*
struct stSendIdTime
{
	Pose pos;
	common::Time time;
	int id;
};
*/

NodeOptions CreateNodeOptions(LaserNaviConfig config);
TrajectoryOptions CreateTrajectoryOptions(void);




class CCartoSlam
{
private:



    SlamResultCbFunc 	m_pslamCbFunc;

    Node *pnode;


    int 	m_status;

    double  m_createmap_range;

    double    m_metersPerPixel;

   // double laser_check_interval_;
   // double odom_check_interval;

   // common::Time last_laser_received_ts_;
   // common::Time last_odom_received_ts_;

    //TrajectoryOptions trajectory_options;

    Pose local_pose;

    std::mutex  build_mtx;

    void CreateNode();

public:

     static  CCartoSlam* pSingle ;
/*
    static LCMTask& GetLcmInstance(){
        static LCMTask instance;
        return instance;
    }*/
    CCartoSlam();

    ~CCartoSlam(void);

     static CCartoSlam* GetInstance() {
        if (pSingle == nullptr) {

            if (pSingle == nullptr) {
                pSingle = new CCartoSlam();
            }
        }
        return pSingle;
    }

    static void DesInstance() {
        if(pSingle) {
            delete pSingle;
            pSingle = nullptr;
        }
    }


    //CNavigation(const NodeOptions& _node_options,mapping::MapBuilderInterface *map_builder ,const TrajectoryOptions& _trajectory_options);


    void Publish();

    static void GetLocalSlamPose(int *isaddnode,const common::Time *ptime,const transform::Rigid3d *plocal_pose,sensor::RangeData *prangedata,bool *bupdatemap,bool *bfirstframe,mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> *pnodepose,double *prealtimescore);

    void RegisterSlamCallBack(SlamResultCbFunc pFunc);
    void RegisterBuildOverCallBack(BuildOverFunc pFunc);
    void RegisterOptOverCallBack(OptOverFunc pFunc);
	
    void HandleLaserData( sensor_msgs::LaserScan &msg);

    void HandleLaserData( laserscan_msg *pmsg);

    void HandleEncoderData(sensor_msgs::OdometryProto &msg);
    void HandleEncoderData(odometry_msg *pmsg);

    bool createMap(void);

    int saveMap(const string filename);

    int saveMap(const string filename,std::vector<sensor_msgs::LaserScan> &vtLasers);

    int StopMapping();

    void Reset();

    void AddLaserID(std::string laser_id);


    void GetSubmapList(SubmapList &submap_list) ;

    void GetSubmapData(const mapping::SubmapId submap_id,mapping::SubmapTexture *texture) ;
    void GetSubmapData_upload(const mapping::SubmapId submap_id, mapping::SubmapTexture_upload *texture, vector<double> &blackcell_index);

    mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> GetNodeData(void);


    void HandleLaserData(sensor::PointCloud &pointclouds);

    void GetLocalPose(Pose &localpos);

    int RunFinalOptimization();
    mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::SubmapData> GetSubmapDataUnderLock() const;

    std::vector<mapping::PoseGraphInterface::Constraint> GetConstraints() const;
    mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose> GetTrajectoryNodePoses() ;
    mapping::MapById<mapping::SubmapId, mapping::PoseGraphInterface::MySubmapData> GetMySubmapData() const;

   int GetNumFinishedNodes();

   map<int, vector<int>> GetPossibleConstraintPairs();
   void GetNodeData(std::vector<node_data> &nodeDatas);
   void GetSubmapData(int submap_id,submap_data *pdata);
   void GetSubmapData_upload(int submap_id, transform::Rigid3d &global_pose, submap_data *pdata, vector<double> &blackcell_index);
   bool StartExpandMap(Pose &initPos,std::unique_ptr<mapping::Grid2D> gridMap,double timestamp);

   bool StartLocate(Pose &initPos,std::unique_ptr<mapping::Grid2D> gridMap,double timestamp);

   bool StopLocate();
   void SetInitNodePose(unsigned long long  timestamp,const Pose &initPos);

   void SetInitNodePose(double  timestamp,const Pose &initPos );
   void Clear();

   bool LoadBinary(char *filename);

   bool StartExpandMapnew(Pose &initPos,std::unique_ptr<mapping::Grid2D> gridMap,double timestamp);

   bool OpenExpandDx(FILE *fp);

   bool SaveBinary(FILE *fp);

   bool RotateMap(double angle);

   void SetFrozen(bool flag);

   int  GetFrozenNodeNum();

   int GetFrozenSubmapNum();
};





