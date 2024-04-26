#include "ZTypes.h"
#include "LCMTask.h"
#include "Localization_Msg.h"
#include "json/json.h"
#include "AffinePosture.h"

#include "robot_control_t.h"
#include "robot_control_t_new.h"

#include "RoboManager.h"
#include "RoboLocProto.h"   // add for lcm
#include "Project.h"
#include "calibration.h"
#include "rigid_transform.h"

#include "LaserAutoMapping.h"

#pragma GCC push_options
#pragma GCC optimize ("O0")

bool LCMTask::m_bNaviToPad = false;  // static 变量需要类外定义, 分配内存


SIASUN_PTHREAD_PROC_DECL LCMRecvTaskMinitor( LPVOID arg )
{
    LCMTask& mlcm = LCMTask::GetLcmInstance();
    while( 1 )
    {
        lcm_handle( mlcm.GetLcmHandle() );
        Sleep(50);
    }
    mlcm.LCMUnInit();
    return NULL;
}

// 导航给 pad 发送心跳线程, 1s发送一次 add 2022.06.06
SIASUN_PTHREAD_PROC_DECL NaviSendHeartToPad( LPVOID arg )
{
    while( 1 )
    {
        LCMTask::GetLcmInstance().SendHeart ();
        Sleep(1000);
    }

    return NULL;
}

SIASUN_PTHREAD_PROC_DECL LCMSendTask( LPVOID arg );


void FeatureMatchInfoHandle(const lcm_recv_buf_t *rbuf, const char * channel,
                            const FeatureMatchInfoLcm_ *FeatureMatchInfoData,    void * user);

void CloudAdjustHandle(const lcm_recv_buf_t *rbuf, const char * channel,
                            const LcmCloudAdjusted *CloudAdjustData,    void * user);

void PadMsgHandle ( const lcm_recv_buf_t *rbuf, const char *channel,
                        const robot_control_t *PadControlData, void *user );

void PadMsgHandle_new ( const lcm_recv_buf_t *rbuf, const char *channel,
                        const robot_control_t_new *PadControlData, void *user );

int LCMTask::LCMInit( void )
{
   // printf("LCMTask::LCMInit\n");
    m_bPadReceived = false;
    nState_LocalMsg = 2;
    LocalMsgTick = GetTickCount();
    CreateThread(50);
    return 0;
}

int LCMTask::LCMRecInit()
{
    Scan_= NULL;
    FeatureMatchInfoLcm__subscribe(lcm, "FeatureMatchInfoLcm_",&FeatureMatchInfoHandle,NULL);
    LcmCloudAdjusted_subscribe(lcm, "LcmCloudAdjusted",&CloudAdjustHandle,NULL);

    SiaSun_AfxBeginThread(LCMRecvTaskMinitor,(LPVOID)this, &m_LcmThread);

}
int LCMTask::LCMInitSubscrible(void *user)
{
    //robot_control_t_subscribe ( lcm, "NAVI_SERVICE_COMMAND", &PadMsgHandle, user );
    robot_control_t_new_subscribe ( lcm, "NAVI_SERVICE_COMMAND", &PadMsgHandle_new, user );
    SiaSun_AfxBeginThread(LCMRecvTaskMinitor,(LPVOID)this, &m_LcmThread);

    // 导航给 pad 发送心跳、状态
    SiaSun_AfxBeginThread ( NaviSendHeartToPad, (LPVOID)this, &m_HeartThread );
}

//  by cgd   delete grid in map
bool DeleteArea ( double topLeft_x, double topLeft_y, double downRight_x, double downRight_y )
{
    const char* fileStr = "ProbMap.txt";
    // 打开地图
    ifstream infile;
    infile.open ( WORK_PATH"ProbMap.txt", ios::in );
    if ( !infile ) {
        printf ( "infile open failed !\n" );
        return false;
    }

    ofstream offile_p;
    ofstream offile_y;
    string str_p = SHAREFILES_PATH"PM.pgm";
    string str_y = SHAREFILES_PATH"PM.yaml";
    vector<int> temp;

    offile_p.open(str_p,ios::out);
    offile_y.open(str_y,ios::out);
    if(!offile_p ||!offile_y)
        return false;

    double  range;
    double  resolution;
    int     nwidth;
    int     nheight;
    double  meterPer;
    double  origin_position_x;
    double  origin_position_y;
    bool    is_map_data_valid = true;

    infile >> range;               // 未使用
    infile >> resolution;          // 分辨率
    infile >> nheight;              // 高度
    infile >> nwidth;               // 宽度
    infile >> meterPer;             // 分辨率
    infile >> origin_position_x;    // 左下角 x
    infile >> origin_position_y;    // 左下角 y

    if ( nheight <= 0 || nwidth <= 0 || meterPer < 0.01 )
    {
        printf ( "ERR: Wrong Map File Format" );
        return false;
    }

    int *mapData = new int [ nheight * nwidth ];    // 地图数据
    int row      = 0;                               // 地图行下标
    int col      = 0;                               // 地图列下标

    // 读取地图数据
    while ( !infile.eof() )
    {
        int data;

        infile >> data;

        mapData [row * nwidth + col] = data;

        col++;
        if ( col == nwidth )
        {
            col = 0;
            row++;
        }
    }

    infile.close();


    // 删除区域
    int col_min = (topLeft_x   - origin_position_x) / resolution;   // 左上角 x 下标
    int col_max = (downRight_x - origin_position_x) / resolution;   // 右下角 x 下标
    int row_min = (downRight_y - origin_position_y) / resolution;   // 右下角 y 下标
    int row_max = (topLeft_y   - origin_position_y) / resolution;   // 左上角 y 下标
    printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );

    col_min = max(0,col_min);
    col_max = min(col_max,nwidth);
    row_min = max(0,row_min);
    row_max = min(row_max,nheight);

   // printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );
  //  printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );


    for ( int y = row_min; y < row_max; y++ )
        for ( int x = col_min; x < col_max; x++ )
            mapData [y * nwidth + x] = 255; // 将删去区域概率置为255, 显示白色

    // 保存地图
    ofstream outFile;
    outFile.open ( WORK_PATH"ProbMap.txt", ios::out );
    if ( !outFile ) {
        printf ( "outFile open failed !\n" );
        return false;
    }

    outFile << 15;
    outFile << ' ';
    outFile << resolution;
    outFile << ' ';
    outFile << nheight;
    outFile << ' ';
    outFile << nwidth;
    outFile << ' ';
    outFile << meterPer;
    outFile << ' ';
    outFile << origin_position_x;
    outFile << ' ';
    outFile << origin_position_y;

    for ( int i = 0; i < nwidth * nheight; i++)
    {
        if ( i % nwidth == 0 )
        {
            outFile << std::endl;
        }

        outFile << mapData[i];
        outFile << ' ';
    }

    offile_p<<"P2\n";
    offile_p<<nwidth<<" "<<nheight<<std::endl;
    offile_p<<"2\n";

    int j=0;
    for(int h = 0;h<nheight;h++)
    {
        for (int w = 0;w<nwidth; w++)
        {
            int index = nheight*nwidth-(h*nwidth+nwidth-1-w);
            int data = 0;
            if(mapData[index]>127)
              data = 2;
			else if (mapData[index] == 127)
			  data = 1;
            else
              data = 0;

              j++;
              offile_p<<data<<" ";
              if(j%nheight ==0)
                  offile_p<<std::endl;
        }
    }

    offile_y<<"image: PM.pgm\n";
    offile_y<<"resolution: "<<"0.05"<<std::endl;
    offile_y<<"origin: ["<<origin_position_x<<", "<<origin_position_y<<", 0.0]\n";
    offile_y<<"occupied_thresh: 0.5\n";
    offile_y<<"free_thresh: 0.5\n";
    offile_y<<"negate: 0\n";
    std::cout<<"YAML FILE SAVED, PLEASE EDIT PATH TO MAP!"<<std::endl;


    offile_y.close();
    offile_p.close();

    outFile.close();

    delete [] mapData;

    return true;
}

//dq 同时删除多个区域
bool DeleteMultipleAreas ( int8_t *iparams, double *dparams )
{
    const char* fileStr = "ProbMap.txt";
    // 打开地图
    ifstream infile;
    infile.open ( WORK_PATH"ProbMap.txt", ios::in );
    if ( !infile ) {
        printf ( "infile open failed !\n" );
        return false;
    }
    ofstream offile_p;
    ofstream offile_y;
    string str_p = SHAREFILES_PATH"PM.pgm";
    string str_y = SHAREFILES_PATH"PM.yaml";
    vector<int> temp;

    offile_p.open(str_p,ios::out);
    offile_y.open(str_y,ios::out);
    if(!offile_p ||!offile_y)
        return false;

    double  range;
    double  resolution;
    int     nwidth;
    int     nheight;
    double  meterPer;
    double  origin_position_x;
    double  origin_position_y;
    bool    is_map_data_valid = true;
    double topLeft_x = 0;
    double downRight_x = 0;
    double downRight_y = 0;
    double topLeft_y = 0;

    infile >> range;               // 未使用
    infile >> resolution;          // 分辨率
    infile >> nheight;              // 高度
    infile >> nwidth;               // 宽度
    infile >> meterPer;             // 分辨率
    infile >> origin_position_x;    // 左下角 x
    infile >> origin_position_y;    // 左下角 y

    if ( nheight <= 0 || nwidth <= 0 || meterPer < 0.01 )
    {
        printf ( "ERR: Wrong Map File Format" );
        return false;
    }
    std::cout<<"nheight: "<<nheight<<", nwidth: "<<nwidth<<std::endl;
    //int *mapData = new int [ nheight * nwidth ];    // 地图数据
    vector<int> mapData;
    mapData.resize(nheight * nwidth+100);
    int row      = 0;                               // 地图行下标
    int col      = 0;                               // 地图列下标
    // 读取地图数据
    while ( !infile.eof() )
    {
        int data;

        infile >> data;

        mapData [row * nwidth + col] = data;

        col++;
        if ( col == nwidth )
        {
            col = 0;
            row++;
        }
    }

    infile.close();


    // 删除区域
    std::cout<<"<<<<<<<<<<<<<<<<删除区域>>>>>>>>>>>>>>>>"<<std::endl;
    for(int i = 0; i < iparams[0]; i++)
    {
        topLeft_x = dparams[4*i];
        topLeft_y = dparams[4*i+1];
        downRight_x = dparams[4*i+2];
        downRight_y = dparams[4*i+3];
        int col_min = (topLeft_x   - origin_position_x) / resolution;   // 左上角 x 下标
        int col_max = (downRight_x - origin_position_x) / resolution;   // 右下角 x 下标
        int row_min = (downRight_y - origin_position_y) / resolution;   // 右下角 y 下标
        int row_max = (topLeft_y   - origin_position_y) / resolution;   // 左上角 y 下标
        printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );

        col_min = max(0,col_min);
        col_max = min(col_max,nwidth);
        row_min = max(0,row_min);
        row_max = min(row_max,nheight);

        // printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );
        //  printf ( "col_min = %d, col_max = %d, row_min = %d, row_max = %d\n", col_min, col_max, row_min, row_max );


        for ( int y = row_min; y < row_max; y++ )
            for ( int x = col_min; x < col_max; x++ )
            {
                int clear_index = y * nwidth + x;
                if(clear_index > 0 && clear_index < nheight * nwidth)
                mapData [y * nwidth + x] = 255; // 将删去区域概率置为255, 显示白色
            }
    }
    // 保存地图
    ofstream outFile;
    outFile.open ( WORK_PATH"ProbMap.txt", ios::out );
    if ( !outFile ) {
        printf ( "outFile open failed !\n" );
        mapData.clear();
        return false;
    }

    outFile << 15;
    outFile << ' ';
    outFile << resolution;
    outFile << ' ';
    outFile << nheight;
    outFile << ' ';
    outFile << nwidth;
    outFile << ' ';
    outFile << meterPer;
    outFile << ' ';
    outFile << origin_position_x;
    outFile << ' ';
    outFile << origin_position_y;

    for ( int i = 0; i < nwidth * nheight; i++)
    {
        if ( i % nwidth == 0 )
        {
            outFile << std::endl;
        }

        outFile << mapData[i];
        outFile << ' ';
    }

    offile_p<<"P2\n";
    offile_p<<nwidth<<" "<<nheight<<std::endl;
    offile_p<<"2\n";

    int j=0;
    for(int h = 0;h<nheight;h++)
    {
        for (int w = 0;w<nwidth; w++)
        {
            int index = nheight*nwidth-(h*nwidth+nwidth-1-w);
            int data = 0;
            if(mapData[index]>127)
              data = 2;
			else if (mapData[index] == 127)
			  data = 1;
            else
              data = 0;


              j++;
              offile_p<<data<<" ";
              if(j%nheight ==0)
                  offile_p<<std::endl;
        }
    }

    offile_y<<"image: PM.pgm\n";
    offile_y<<"resolution: "<<"0.05"<<std::endl;
    offile_y<<"origin: ["<<origin_position_x<<", "<<origin_position_y<<", 0.0]\n";
    offile_y<<"occupied_thresh: 0.5\n";
    offile_y<<"free_thresh: 0.5\n";
    offile_y<<"negate: 0\n";
    std::cout<<"YAML FILE SAVED, PLEASE EDIT PATH TO MAP!"<<std::endl;


    offile_y.close();
    offile_p.close();


    outFile.close();

    //delete [] mapData;
    mapData.clear();

    return true;
}

bool ReserveArea(int index, int wide_limits_num_x_cells, int topLeft_x, int topLeft_y, int downRight_x, int downRight_y)
{

    int y = index%wide_limits_num_x_cells;
    int x = index/wide_limits_num_x_cells;


    int s1 = (x-topLeft_x)*(x-downRight_x);
    int s2 = (y-topLeft_y)*(y-downRight_y);

    if(s1<=0 && s2<=0)
    {
        return false;

    }
    return true;
}

//  by lishen  delete grid in map (fast match)
bool DeleteFasthMapArea(double topLeft_x, double topLeft_y, double downRight_x, double downRight_y)
{
    // limits_.resolution_

    struct griddata
    {

      int offsetx;
      int offsety;
      int wide_limits_num_x_cells;
      int wide_limits_num_y_cells;
      float min_score;
      float max_score;
      int occpuiedCellNum;
      std::vector<unsigned char> cells_;
    };


    FILE *fp_open = fopen(WORK_PATH"Gridmap.map", "rb");
    if (fp_open == NULL)
        return false;

    int submapnum = 0;
    int submapID ;
    double resolution;
    double max_x;
    double max_y;
    int num_x_cells;
    int num_y_cells;
    int branch_and_bound_depth;

    std::vector<griddata> mapStack;

    if (fread(&submapnum, sizeof(int), 1, fp_open) != 1)
        return false;

    if (fread(&submapID, sizeof(int), 1, fp_open) != 1)
        return false;

    if (fread(&resolution, sizeof(double), 1, fp_open) != 1)
     return false;

    if (fread(&max_x, sizeof(double), 1, fp_open) != 1)
     return false;

    if (fread(&max_y, sizeof(double), 1, fp_open) != 1)
     return false;

    if (fread(&num_x_cells, sizeof(int), 1, fp_open) != 1)
     return false;

    if (fread(&num_y_cells, sizeof(int), 1, fp_open) != 1)
     return false;

    if (fread(&branch_and_bound_depth, sizeof(int), 1, fp_open) != 1)
     return false;

    if(branch_and_bound_depth<=0)
        return false;

    int branch_and_bound_depth2 = 0;
    if (fread(&branch_and_bound_depth2, sizeof(int), 1, fp_open) != 1)
     return false;

    if(branch_and_bound_depth2<=0)
        return false;


    for(int j=0;j<branch_and_bound_depth2;j++)
    {

        griddata  data;

        if (fread(&data.offsetx, sizeof(int), 1, fp_open) != 1)
            return false;

        if (fread(&data.offsety, sizeof(int), 1, fp_open) != 1)
            return false;

        if (fread(&data.wide_limits_num_x_cells, sizeof(int), 1, fp_open) != 1)
            return false;

        if (fread(&data.wide_limits_num_y_cells, sizeof(int), 1, fp_open) != 1)
            return false;

        if (fread(&data.min_score, sizeof(float), 1, fp_open) != 1)
            return false;

        if (fread(&data.max_score, sizeof(float), 1, fp_open) != 1)
            return false;

        if (fread(&data.occpuiedCellNum, sizeof(int), 1, fp_open) != 1)
            return false;


        data.cells_.resize(data.wide_limits_num_x_cells * data.wide_limits_num_y_cells,0x00);

        int topLeftx  =   (max_x-topLeft_x)/resolution;
        int topLefty  =   (max_y-topLeft_y)/resolution;
        int downRightx =   (max_x-downRight_x)/resolution;
        int downRighty =   (max_y-downRight_y)/resolution;

        for(int i=0;i<data.occpuiedCellNum;i++)
        {
            int index;
            if (fread(&index, sizeof(int), 1, fp_open) != 1)
                return false;

            unsigned char   value;
            if (fread(&value, sizeof(unsigned char), 1, fp_open) != 1)
                return false;
            if(index>=0 && index<data.wide_limits_num_x_cells * data.wide_limits_num_y_cells && ReserveArea(index, data.wide_limits_num_x_cells, topLeftx, topLefty, downRightx, downRighty))
            {
                   data.cells_.at(index) = value;

             }
        }

        int occpuiedCellNum = 0;
        for(int i=0;i<data.cells_.size();i++)
        {
            if(data.cells_.at(i)!=0x00)
                occpuiedCellNum++;
        }

        data.occpuiedCellNum = occpuiedCellNum;
        mapStack.push_back(data);

    }
    fclose(fp_open);



    FILE *fpsave = fopen(WORK_PATH"Gridmap.map", "wb");
    if (fpsave == NULL)
        return false;

    if (fwrite(&submapnum, sizeof(int), 1, fpsave) != 1)
        return false;

    if (fwrite(&submapID, sizeof(int), 1, fpsave) != 1)
        return false;

    if (fwrite(&resolution, sizeof(double), 1, fpsave) != 1)
     return false;
    if (fwrite(&max_x, sizeof(double), 1, fpsave) != 1)
     return false;
    if (fwrite(&max_y, sizeof(double), 1, fpsave) != 1)
     return false;
    if (fwrite(&num_x_cells, sizeof(int), 1, fpsave) != 1)
     return false;
    if (fwrite(&num_y_cells, sizeof(int), 1, fpsave) != 1)
     return false;
    if (fwrite(&branch_and_bound_depth, sizeof(int), 1, fpsave) != 1)
     return false;

    if (fwrite(&branch_and_bound_depth2, sizeof(int), 1, fpsave) != 1)
     return false;

    if(mapStack.size()<branch_and_bound_depth2)
        return false;


    for(int i=0;i<branch_and_bound_depth2;i++)
    {

        griddata data = mapStack.at(i);

        if (fwrite(&data.offsetx, sizeof(int), 1, fpsave) != 1)
         return false;

        if (fwrite(&data.offsety, sizeof(int), 1, fpsave) != 1)
         return false;

        if (fwrite(&data.wide_limits_num_x_cells, sizeof(int), 1, fpsave) != 1)
         return false;

        if (fwrite(&data.wide_limits_num_y_cells, sizeof(int), 1, fpsave) != 1)
         return false;

        if (fwrite(&data.min_score, sizeof(float), 1, fpsave) != 1)
         return false;

        if (fwrite(&data.max_score, sizeof(float), 1, fpsave) != 1)
            return false;

        if (fwrite(&data.occpuiedCellNum, sizeof(int), 1, fpsave) != 1)
            return false;

        for(int i=0;i<data.cells_.size();i++)
        {
            if(data.cells_.at(i)!=0x00)
            {
                if (fwrite(&i, sizeof(int), 1, fpsave) != 1)
                    return false;
                if (fwrite(&data.cells_.at(i), sizeof(unsigned char), 1, fpsave) != 1)
                    return false;
            }
        }

    }
    fclose(fpsave);

}

// by DQ PadMsgHandle_new
void PadMsgHandle_new ( const lcm_recv_buf_t *rbuf, const char *channel,
                        const robot_control_t_new *PadControlData, void *user )
{
    // 更改工作模式
#ifdef NAV_APP
    robo::CRoboManager *pManager = (robo::CRoboManager*)user;
    bool                lcmResult;
   // printf ( "pad  to commandid = %d\n", PadControlData->commandid );
    switch ( PadControlData->commandid )
    {

        case 5: // pad开始建图
        {
			// dq VISION
             system("rm -rf /userdata/CarryBoy/NAV/ImgRecord/*");
             std::cout<<"rm -rf /userdata/CarryBoy/NAV/ImgRecord/*"<<std::endl;
            // system("mkdir /userdata/CarryBoy/NAV/ImgRecord");
            // double seconds = 0;
            // scanf("%d",&seconds);
            // char chars[100];
            // sprintf(chars,"shutdown -s -t %d",seconds);
            // system(chars);
            std::cout<<"开始建图"<<std::endl;

            int workMode = RoboManagerSingleton::GetInstance()->GetCurWorkMode();
            if(RLP_MODE_LOCALIZATION == workMode )
            {

                auto pMapping = LaserAutoMappingSingleton::GetInstance();

                pMapping->SetMappingMode(mapping::Mode_BuildMap);

                pMapping->SetExpandFrozen(false);

                pManager->HandlePadMsg ( RLP_MODE_AUTOMAPPING );
                LCMTask::GetLcmInstance().SendNaviCommand ( 111 );  // 开始建图应答
            }


        }
            break;
        case 6: // pad结束建图

            std::cout<<"pad结束建图"<<std::endl;
            pManager->HandlePadMsg ( RLP_MODE_STOPMAPPING );
            LCMTask::GetLcmInstance().SendNaviCommand ( 112 );  // 结束建图应答
            break;
        case 9: // pad删除区域
            if ( PadControlData -> ndparams == 4 ) {
                lcmResult = DeleteArea( PadControlData -> dparams[0],
                                        PadControlData -> dparams[1],
                                        PadControlData -> dparams[2],
                                        PadControlData -> dparams[3] );

                if ( lcmResult == true ) {
                    LCMTask::GetLcmInstance().SendNaviCommandEx ( 52, 1 );  // pad删除区域应答
                }
                else {
                    LCMTask::GetLcmInstance().SendNaviCommandEx ( 52, 0 );  // pad删除区域应答
                }
            }
            else {
                printf ( " commandid 9, ndparams num is error !!!!!\n" );
            }
            break;
        case 10: // pad删除多区域
            if ( PadControlData -> niparams > 0 && PadControlData -> iparams[0] > 0) {
                std::cout<<"MultipleAreas have been recieved!!!  Num: "<<PadControlData -> niparams<<", "<< PadControlData -> iparams[0] <<std::endl;
                lcmResult = DeleteMultipleAreas(PadControlData -> iparams, PadControlData -> dparams);

                if ( lcmResult == true ) {
                    LCMTask::GetLcmInstance().SendNaviCommandEx ( 52, 1 );  // pad删除区域应答
                }
                else {
                    LCMTask::GetLcmInstance().SendNaviCommandEx ( 52, 0 );  // pad删除区域应答
                }
            }
            else {
                printf ( " commandid 10, ndparams num is error !!!!!\n" );
            }
            break;

        case 20:   // pad 向 导航请求心跳, pad 请求一次, 导航应答一次
            LCMTask::m_bNaviToPad    = true;
            break;
        case 21:   // 录制dx

             std::cout<<"&&&&&&&&&&&&&&&&&&&录制dx\n";

            if ( PadControlData -> niparams == 1 ) {
                printf ( "commandid 21, iparams[0] = %d\n", PadControlData -> iparams[0] );
                if ( PadControlData -> iparams[0] == 1 ) {
                    pManager->HandlePadMsg ( RLP_MODE_MAPPING );
                }
                else if ( PadControlData -> iparams[0] == 2 ) {
                    pManager->HandlePadMsg ( RLP_MODE_LOCALIZATION );
                }
            }
            else {
                printf ( " commandid 21, niparams num is error !!!!!\n" );
            }

            break;


        case 22:    // 导航结束建图状态中, 结束优化后, pad 界面是否保存地图
            if ( PadControlData -> niparams == 1 ) {
                vector<double> dparams;
                if ( PadControlData -> iparams[0] == 1 ) {
                    pManager->HandleSaveMapMsg ( 1 ,dparams);
                }
                else if ( PadControlData -> iparams[0] == 2 ) {
                    pManager->HandleSaveMapMsg ( 2 ,dparams);
                }
                else if ( PadControlData -> iparams[0] == 3 ) {
                    //rotate map and save map

                    std::cout<<"rotate map"<<std::endl;
                    if(PadControlData -> ndparams == 1)
                    {
                        dparams.push_back(PadControlData -> dparams[0]);
                        pManager->HandleSaveMapMsg ( 3 ,dparams);
                    }

                }
            }
            else {
                printf ( " commandid 22, niparams num is error !!!!!\n" );
            }
            break;
        case 23:    // pad 请求 导航将 Probmap.txt 地图恢复
            system("cp ProbMap_bak.txt ProbMap.txt");
            system("cp Gridmap_bak.map Gridmap.map");
            LCMTask::GetLcmInstance().SendNaviCommand ( 54 );
            break;

        case 24:    //by DQ 从pad获取模板
            if( PadControlData -> niparams >= 0)
            {
                if(!pManager->ChangeObjects(PadControlData -> iparams,PadControlData -> dparams))
                {
                    LCMTask::GetLcmInstance().SendNaviCommandEx (55, 0);
                    printf ( " Change Objects Failed!!! error!!!!!\n" );
                }
                else
                    LCMTask::GetLcmInstance().SendNaviCommandEx (55, 1);  // pad添加模板区域应答
            }
            break;
/*
        case 25:    //by DQ 从pad获取区域
            if( PadControlData -> niparams >= 0)
            {
                if(!pManager->ChangePlan(PadControlData -> iparams,PadControlData -> dparams))
                {
                    LCMTask::GetLcmInstance().SendNaviCommandEx (56, 0);
                    printf ( " Change Plan Failed!!! error!!!!!\n" );
                }
                else
                    LCMTask::GetLcmInstance().SendNaviCommandEx (56, 1);  // pad添加区域应答
            }
            break;

        case 26:    //by DQ 向pad发送模板
            if( PadControlData -> niparams >= 0)
            {
                int count;
                vector<vector<float>> points;
                vector<CPosture> psts;
                if(!pManager->GetStaticObjects(count, points, psts))
                {
                    LCMTask::GetLcmInstance().SendStaticObjects (55, count, points, psts);
                    printf ( " Get StaticObjects Failed!!! error!!!!!\n" );
                }
                else
                    LCMTask::GetLcmInstance().SendStaticObjects (55, count, points, psts);  // pad删除区域应答
            }
            break;
        case 27:    //by DQ 向pad发送区域
            if( PadControlData -> niparams >= 0)
            {
                int count;
                vector<vector<float>> plans;
                vector<int> type;
                if(!pManager->GetPlans(count, plans, type))
                {
                    LCMTask::GetLcmInstance().SendPlans (55, count, plans, type);
                    printf ( " Get Plans Failed!!! error!!!!!\n" );
                }
                else
                    LCMTask::GetLcmInstance().SendPlans (55, count, plans, type);  // pad删除区域应答
            }
            break;
        case 28:    //by DQ 向pad发送地图、模板、区域
            if( PadControlData -> nsparams >= 0)
            {
                int objs_num;
                int plans_num;
                vector<vector<float>> points;
                vector<CPosture> psts;
                vector<vector<float>> plans;
                vector<int> type;
                pManager->GetMap(PadControlData -> sparams);
                if(!pManager->GetPlans(plans_num, plans, type))
                {
                    printf ( " Get Plans Failed!!! error!!!!!\n" );
                }
                else if(!pManager->GetStaticObjects(objs_num, points, psts))
                {
                    printf ( " Get StaticObjects Failed!!! error!!!!!\n" );
                }
                else
                    LCMTask::GetLcmInstance().SendStaticObjects (55, objs_num, points, psts);
                    LCMTask::GetLcmInstance().SendPlans (55, plans_num, plans, type);
            }
            break;
            */
        case 28:
            LCMTask::GetLcmInstance().SendVersion(60);
            break;

        case 29:
        {
            LCMTask::GetLcmInstance().m_bPadReceived = true;
            std::cout<<"receive m_bPadReceived\n ";
            break;
        }

        case 30:
            {
                std::cout<<"$$$$$$$$$$$$$$$ WriteLaserParam $$$$$$$$$$$$$$$$\n";
                bool res = LCMTask::GetLcmInstance().WriteLaserParam();
                LCMTask::GetLcmInstance().SendWriteLaserParamResut(res);
            }

            break;
        case 31:
            {
               std::cout<<"Mode_ExpandMap"<<std::endl;
               int workMode = RoboManagerSingleton::GetInstance()->GetCurWorkMode();
               if(RLP_MODE_LOCALIZATION == workMode )
               {
                    auto pMapping = LaserAutoMappingSingleton::GetInstance();

                    pMapping->SetMappingMode(mapping::Mode_ExpandMap);

                    pMapping->SetExpandFrozen(true);

                    pManager->HandlePadMsg ( RLP_MODE_AUTOMAPPING );
                    LCMTask::GetLcmInstance().SendNaviCommand ( 111 );  // 开始建图应答
               }
            }
          break;

         case 32:   //expandmap
            {
                auto pMapping = LaserAutoMappingSingleton::GetInstance();

                  std::cout<<"expandmap"<<std::endl;
                if ( PadControlData -> niparams == 1 ) {
                    if ( PadControlData -> iparams[0] == 1 ) {
                          std::cout<<"Frozen"<<std::endl;
                         pMapping->SetExpandFrozen(true);
                    }
                    if ( PadControlData -> iparams[0] == 0 ) {
                        //
                          std::cout<<"not Frozen"<<std::endl;
                         pMapping->SetExpandFrozen(false);
                    }
                }

                pMapping->ExpandMap();
            }
            break;
            case 33:   //send submap
               {
                   auto pMapping = LaserAutoMappingSingleton::GetInstance();
                   std::cout<<"all send submap"<<std::endl;
                   if ( PadControlData ->niparams> 0  ) {

                        pMapping->SendSubMaps(PadControlData ->niparams,PadControlData -> iparams);

                   }
               }
            break;
            case 34:
            {
                if ( PadControlData ->niparams== 1)
                {
                    if(PadControlData ->iparams[0] == 63)
                    {
                        auto pMapping = LaserAutoMappingSingleton::GetInstance();
                        pMapping->SetPadAnswer();

                         std::cout<<"PAD answer"<<std::endl;
                    }
                }
            }
            break;

    }
#endif
}

// 接收lcm, pad控制命令 add by cgd
void PadMsgHandle ( const lcm_recv_buf_t *rbuf, const char *channel,
                        const robot_control_t *PadControlData, void *user )
{
    // 更改工作模式
//#ifdef NAV_APP
#if 0
    robo::CRoboManager *pManager = (robo::CRoboManager*)user;
    bool                lcmResult;

    //printf ( "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!pad  to commandid = %d\n", PadControlData->commandid );
    switch ( PadControlData->commandid )
    {
        #if 1
        case 5: // pad开始建图
            pManager->HandlePadMsg ( RLP_MODE_AUTOMAPPING );
            LCMTask::GetLcmInstance().SendNaviCommand ( 111 );  // 开始建图应答
            break;
        case 6: // pad结束建图
            pManager->HandlePadMsg ( RLP_MODE_STOPMAPPING );
            LCMTask::GetLcmInstance().SendNaviCommand ( 112 );  // 结束建图应答
            break;
        case 9: // pad删除区域
            if ( PadControlData -> ndparams == 4 ) {
                lcmResult = DeleteArea( PadControlData -> dparams[0],
                                        PadControlData -> dparams[1],
                                        PadControlData -> dparams[2],
                                        PadControlData -> dparams[3] );

                if ( lcmResult == true ) {
                    LCMTask::GetLcmInstance().SendNaviCommandEx ( 52, 1 );  // pad删除区域应答
                }
                else {
                    LCMTask::GetLcmInstance().SendNaviCommandEx ( 52, 0 );  // pad删除区域应答
                }
            }
            else {
                printf ( " commandid 20, ndparams num is error !!!!!\n" );
            }
            break;
        case 20:   // pad 向 导航请求心跳, pad 请求一次, 导航应答一次
            LCMTask::m_bNaviToPad    = true;
            break;
        case 21:   // 录制dx
            if ( PadControlData -> niparams == 1 ) {
                printf ( "commandid 21, iparams[0] = %d\n", PadControlData -> iparams[0] );
                if ( PadControlData -> iparams[0] == 1 ) {
                    pManager->HandlePadMsg ( RLP_MODE_MAPPING );
                }
                else if ( PadControlData -> iparams[0] == 2 ) {
                    pManager->HandlePadMsg ( RLP_MODE_LOCALIZATION );
                }
            }
            else {
                printf ( " commandid 21, niparams num is error !!!!!\n" );
            }


            break;
        case 22:    // 导航结束建图状态中, 结束优化后, pad 界面是否保存地图
            if ( PadControlData -> niparams == 1 ) {
                if ( PadControlData -> iparams[0] == 1 ) {
                    pManager->HandleSaveMapMsg ( 1 );
                }
                else if ( PadControlData -> iparams[0] == 2 ) {
                    pManager->HandleSaveMapMsg ( 2 );
                }
            }
            else {
                printf ( " commandid 22, niparams num is error !!!!!\n" );
            }
            break;
        case 23:    // pad 请求 导航将 Probmap.txt 地图恢复
            system("cp ProbMap_bak.txt ProbMap.txt");
            system("cp Gridmap_bak.map Gridmap.map");
            LCMTask::GetLcmInstance().SendNaviCommand ( 54 );
            break;

        #endif



    }
#endif
}

void FeatureMatchInfoHandle(const lcm_recv_buf_t *rbuf, const char * channel,
                            const FeatureMatchInfoLcm_ *FeatureMatchInfoData,    void * user)
{
    LCMTask::GetLcmInstance().SetFeatureMatchInfo(FeatureMatchInfoData);
}

void CloudAdjustHandle(const lcm_recv_buf_t *rbuf, const char * channel,
                            const LcmCloudAdjusted *CloudAdjustData,    void * user)
{
    LCMTask::GetLcmInstance().SetScan(CloudAdjustData);
}

const CFeatureMatchInfo &LCMTask::GetFeatureMatchInfo()
{
    std::lock_guard<std::mutex> locker(feature_mutex);
    return featureMatchInfo_;
}

void LCMTask::SetFeatureMatchInfo(const FeatureMatchInfoLcm_ *FeatureMatchInfoData)
{
    std::lock_guard<std::mutex> locker(feature_mutex);
    featureMatchInfo_.Clear();

    featureMatchInfo_.type_=FeatureMatchInfoData->type_ ;
    featureMatchInfo_.result_=FeatureMatchInfoData->result_ ;
    featureMatchInfo_. pst_.x=FeatureMatchInfoData->NewPstfX_;
    featureMatchInfo_. pst_.y=FeatureMatchInfoData->NewPstfY_;
    featureMatchInfo_. pst_.fThita=FeatureMatchInfoData->NewPstfAngle_  ;
    featureMatchInfo_.matchRatio_ =FeatureMatchInfoData->matchRatio_   ;
    featureMatchInfo_.matchNum_ =FeatureMatchInfoData->matchNum_   ;
    featureMatchInfo_.initPst_.x =FeatureMatchInfoData->InitPstfX_   ;
    featureMatchInfo_.initPst_.y =FeatureMatchInfoData->InitPstfY_   ;
    featureMatchInfo_.initPst_.fThita = FeatureMatchInfoData->InitPstfAngle_   ;

    for(int i  = 0; i < FeatureMatchInfoData->nPoint_ ; i+=2)
    {
        CPnt pnt;
        pnt.x = FeatureMatchInfoData->Point_[i];
        pnt.y = FeatureMatchInfoData->Point_[i+1];
        featureMatchInfo_.vecReflectors_.push_back(pnt);
    }

    for(int i  = 0; i < FeatureMatchInfoData->nLine_ ; i+=4)
    {
        CLine line;
        line.m_ptStart.x = FeatureMatchInfoData->Line_[i];
        line.m_ptStart.y = FeatureMatchInfoData->Line_[i+1];
        line.m_ptEnd.x = FeatureMatchInfoData->Line_[i+2];
        line.m_ptEnd.y = FeatureMatchInfoData->Line_[i+3];

        featureMatchInfo_.vecLines_.push_back(line);
    }
    for(int i  = 0 ; i <  FeatureMatchInfoData->nPointPair_; i+=2)
    {
        CPointMatchPair pair;
        pair.m_ptLocalToWorld.x = FeatureMatchInfoData->PointPair_[i];
        pair.m_ptLocalToWorld.y = FeatureMatchInfoData->PointPair_[i+1];
        featureMatchInfo_.vecPointPair_.push_back(pair);
    }

    for(int i  = 0 ; i <  FeatureMatchInfoData->nLinePair_; i+=4 )
    {
        CLineMatchPair linepair;
        linepair.m_lnWorld.m_ptStart.x = FeatureMatchInfoData->LinePair_[i];
        linepair.m_lnWorld.m_ptStart.y = FeatureMatchInfoData->LinePair_[i+1];
        linepair.m_lnWorld.m_ptEnd.x = FeatureMatchInfoData->LinePair_[i+2];
        linepair.m_lnWorld.m_ptEnd.y = FeatureMatchInfoData->LinePair_[i+3];
        featureMatchInfo_.vecLinePair_.push_back(linepair);
    }

    iLocType = featureMatchInfo_.type_;
}

bool LCMTask::SendCloudAdjusted(const Eigen::Affine3d initPos,const ndt_oru::CStampedPointCloud cloudIn)
{
#ifndef DesktopRun
    if((GetTickCount()-cloudLastTime_ )> 5000)
#endif
    {
        CPosture InitPosture = AffineToPosture(initPos);
        LcmCloudAdjusted LcmCloudAdjustedTemp;
        LcmCloudAdjustedTemp.nSize = cloudIn.size();
        LcmCloudAdjustedTemp.ptX_ = new float[LcmCloudAdjustedTemp.nSize];
        LcmCloudAdjustedTemp.ptY_ = new float[LcmCloudAdjustedTemp.nSize];
        LcmCloudAdjustedTemp.InitPstfX_ = InitPosture.x;
        LcmCloudAdjustedTemp.InitPstfY_ = InitPosture.y;
        LcmCloudAdjustedTemp.InitPstfAngle_ =  InitPosture.fThita;
        vecIntensity.clear();
        for(int i = 0; i < LcmCloudAdjustedTemp.nSize ; i++)
        {
            const Eigen::Vector3d &pt = cloudIn.at(i);
            LcmCloudAdjustedTemp.ptX_[i] = pt(0);
            LcmCloudAdjustedTemp.ptY_[i] = pt(1);
            if(cloudIn.vecIntensity.at(i) > 0)
            {
                vecIntensity.push_back(i);
            }
        }
        int IntensitySize = vecIntensity.size();
        LcmCloudAdjustedTemp.nIntensitySize = IntensitySize;
        LcmCloudAdjustedTemp.Intensity_ = new int32_t[IntensitySize];
        for(int i  = 0 ; i < IntensitySize ; i++)
        {
            LcmCloudAdjustedTemp.Intensity_[i] = vecIntensity.at(i);
        }
        LcmCloudAdjusted_publish(lcm, "LcmCloudAdjusted",&LcmCloudAdjustedTemp);

        delete []LcmCloudAdjustedTemp.Intensity_;
        delete []LcmCloudAdjustedTemp.ptX_;
        delete []LcmCloudAdjustedTemp.ptY_;
        cloudLastTime_ = GetTickCount();
    }
}

void LCMTask::SetScan(const LcmCloudAdjusted *CloudAdjustData)
{
//    std::cout<<"SetScan"<<std::endl;
    std::lock_guard<std::mutex> locker(Scan_mutex);
    if(Scan_)
    {
        delete Scan_;
        Scan_ = NULL;
    }
    CPosture InitPosture;
    InitPosture.x = CloudAdjustData->InitPstfX_;
    InitPosture.y = CloudAdjustData->InitPstfY_;
    InitPosture.fThita = CloudAdjustData->InitPstfAngle_;
    Scan_ = new CScan(int(CloudAdjustData->nSize));
    for (int i = 0; i < CloudAdjustData->nSize; i++)
    {
        Scan_->m_pPoints[i].x = CloudAdjustData->ptX_[i];
        Scan_->m_pPoints[i].y = CloudAdjustData->ptY_[i];
        Scan_->m_pPoints[i].r =  (float)sqrt(CloudAdjustData->ptX_[i] * CloudAdjustData->ptX_[i] + CloudAdjustData->ptY_[i] * CloudAdjustData->ptY_[i]);//单位m
        Scan_->m_pPoints[i].a =  NormAngle2(atan2(CloudAdjustData->ptY_[i],CloudAdjustData->ptX_[i]));//单位弧度
        Scan_->m_pPoints[i].m_nIntensity = 0;

    }
    for(int j  = 0 ; j <  CloudAdjustData->nIntensitySize ; j++)
    {
        Scan_->m_pPoints[CloudAdjustData->Intensity_[j]].m_nIntensity = 255; //wt_test_intensity 20230221
    }

    Scan_->InvTransform(InitPosture);
    Scan_->m_pstScanner = InitPosture;
}

//after use  must delete
CScan *LCMTask::GetCopyScan()
{
    std::lock_guard<std::mutex> locker(Scan_mutex);
    CScan * RetScan = NULL;
    if(Scan_)
     RetScan = Scan_->Duplicate();
    return RetScan;
}

int LCMTask::LCMUnInit( void )
{
    printf( "[%s]lcm: %p\n", __func__, lcm );

    lcm_destroy(lcm);

    return 0;
}

bool LCMTask::SendMatchInfo(const CMatchInfo *MatchInfo)
{
    if(MatchInfo) //如果results为null
    {
        switch(MatchInfo->type_)
        {
            case CMatchInfo::LOC_NDT:
                break;
            case CMatchInfo::LOC_FEATURE:
                return SendFeatureMatchInfo(MatchInfo);
            break;
            case CMatchInfo::LOC_TEMPLATE:
                break;
        }
    }
    return false;
}

bool LCMTask::SendFeatureMatchInfo(const CMatchInfo *MatchInfo)
{
#ifndef DesktopRun
    if(MatchInfo && (GetTickCount() - featureLastTime_) > 1000)
#endif
     {
        CFeatureMatchInfo *pFeatureMatchInfo = ((CFeatureMatchInfo*)(MatchInfo));
        FeatureMatchInfoLcm_ FeatureMatchInfoLcm;
        FeatureMatchInfoLcm.type_ = pFeatureMatchInfo->type_;
        FeatureMatchInfoLcm.result_ = pFeatureMatchInfo->result_;
        FeatureMatchInfoLcm.NewPstfX_  = pFeatureMatchInfo-> pst_.x;
        FeatureMatchInfoLcm.NewPstfY_  = pFeatureMatchInfo-> pst_.y;
        FeatureMatchInfoLcm.NewPstfAngle_  = pFeatureMatchInfo-> pst_.fThita;
        FeatureMatchInfoLcm.matchRatio_  = pFeatureMatchInfo->matchRatio_ ;
        FeatureMatchInfoLcm.matchNum_  = pFeatureMatchInfo->matchNum_ ;
        FeatureMatchInfoLcm.InitPstfX_  = pFeatureMatchInfo->initPst_.x ;
        FeatureMatchInfoLcm.InitPstfY_  = pFeatureMatchInfo->initPst_.y ;
        FeatureMatchInfoLcm.InitPstfAngle_  = pFeatureMatchInfo->initPst_.fThita ;

        FeatureMatchInfoLcm.nPoint_  = pFeatureMatchInfo->vecReflectors_.size()*2 ;
        int j = 0;
        FeatureMatchInfoLcm.Point_ = new float[FeatureMatchInfoLcm.nPoint_];
        for(int i  = 0; i < pFeatureMatchInfo->vecReflectors_.size() ; i ++)
        {
            FeatureMatchInfoLcm.Point_[j++] = (pFeatureMatchInfo->vecReflectors_.at(i).x);
            FeatureMatchInfoLcm.Point_[j++] = (pFeatureMatchInfo->vecReflectors_.at(i).y);
        }
        FeatureMatchInfoLcm.nLine_  = pFeatureMatchInfo->vecLines_.size()*4 ;
        j = 0;
        FeatureMatchInfoLcm.Line_ = new float[FeatureMatchInfoLcm.nLine_];
        for(int i  = 0 ; i <  pFeatureMatchInfo->vecLines_.size(); i++ )
        {
            FeatureMatchInfoLcm.Line_[j++] =(pFeatureMatchInfo->vecLines_.at(i).m_ptStart.x);
            FeatureMatchInfoLcm.Line_[j++] =(pFeatureMatchInfo->vecLines_.at(i).m_ptStart.y);
            FeatureMatchInfoLcm.Line_[j++] =(pFeatureMatchInfo->vecLines_.at(i).m_ptEnd.x);
            FeatureMatchInfoLcm.Line_[j++] =(pFeatureMatchInfo->vecLines_.at(i).m_ptEnd.y);
        }
        FeatureMatchInfoLcm.nPointPair_  = pFeatureMatchInfo->vecPointPair_.size()*2 ;
        j = 0;
        FeatureMatchInfoLcm.PointPair_ = new float[ FeatureMatchInfoLcm.nPointPair_];
        for(int i  = 0 ; i <  pFeatureMatchInfo->vecPointPair_.size(); i++ )
        {
            FeatureMatchInfoLcm.PointPair_[j++] =(pFeatureMatchInfo->vecPointPair_.at(i).m_ptLocalToWorld.x);
            FeatureMatchInfoLcm.PointPair_[j++] =(pFeatureMatchInfo->vecPointPair_.at(i).m_ptLocalToWorld.y);
        }
        FeatureMatchInfoLcm.nLinePair_  = pFeatureMatchInfo->vecLinePair_.size()*4;
        j = 0;
        FeatureMatchInfoLcm.LinePair_ = new float[FeatureMatchInfoLcm.nLinePair_];
        for(int i  = 0 ; i <  pFeatureMatchInfo->vecLinePair_.size(); i++ )
        {
            FeatureMatchInfoLcm.LinePair_[j++] =(pFeatureMatchInfo->vecLinePair_.at(i).m_lnWorld.m_ptStart.x);
            FeatureMatchInfoLcm.LinePair_[j++] =(pFeatureMatchInfo->vecLinePair_.at(i).m_lnWorld.m_ptStart.y);
            FeatureMatchInfoLcm.LinePair_[j++] =(pFeatureMatchInfo->vecLinePair_.at(i).m_lnWorld.m_ptEnd.x);
            FeatureMatchInfoLcm.LinePair_[j++] =(pFeatureMatchInfo->vecLinePair_.at(i).m_lnWorld.m_ptEnd.y);
        }

        FeatureMatchInfoLcm__publish(lcm, "FeatureMatchInfoLcm_",&FeatureMatchInfoLcm);

        delete []FeatureMatchInfoLcm.Point_;
        delete []FeatureMatchInfoLcm.Line_;
        delete []FeatureMatchInfoLcm.PointPair_;
        delete []FeatureMatchInfoLcm.LinePair_;
        featureLastTime_ = GetTickCount();
    }

    return true;
}


void LCMTask::LCMSendLocalizationMsg()
{
    //cout<<"send msg"<<endl;
    Localization_Msg_publish(lcm, "LOCALIZATIONMSG",&LocalizationMsg);
    delete[] dis;
    delete[] lcmintensities;
    //cout<<"send msg 1"<<endl;
}

void LCMTask::SupportRoutineProxy()
{
    switch(nState_LocalMsg)
    {
    case 0 :
        nState_LocalMsg = 1;
        break;
    case 1:
        {
            if(bLocalMsgReady)
            {
                LCMSendLocalizationMsg();
                LocalMsgTick = GetTickCount();
                bLocalMsgReady = false;
                nState_LocalMsg = 2;
            }
        }
        break;
    case 2:
        if(GetTickCount() - LocalMsgTick > 200)
            nState_LocalMsg = 0;
        break;
    default:
        LocalMsgTick = GetTickCount();
        nState_LocalMsg = 2;
        break;
    }
}

bool LCMTask::GetLocalizationMsg(sensor::CRawScan& pScan)
{
     if(bLocalMsgReady)
         return false;
    std::ifstream FileLaserParm(WORK_PATH"LaserParm.json");
    Json::Reader Jreader;
    Json::Value LaserParmRoot;

    if(!FileLaserParm){
        return false;
    }

    int16_t nVersion = 210;
    int16_t SensorCount = 4;
    int16_t RealSensorCount = 2;
    int16_t LaserProductor[4] = {0};
    bool LaserState[4] = {false};

    if (Jreader.parse(FileLaserParm, LaserParmRoot))
    {
        LaserParmRoot.toStyledString();

        // 文件格式版本号
        if (!LaserParmRoot["version"].isNull()) {
            LocalizationMsg.version = LaserParmRoot["version"].asInt();
        }

        //激光器个数
        if(!LaserParmRoot["laser"].isNull()) {
            SensorCount = LaserParmRoot["laser"].size();
        }
        RealSensorCount = SensorCount;
        for(int j = 0; j < SensorCount; j++) {
            if(!LaserParmRoot["laser"][j]["State"].isNull()) {
                LaserState[j] = LaserParmRoot["laser"][j]["State"].asBool();
            }
            else {
                LaserState[j] = false;
            }

            if(!LaserState[j])
                RealSensorCount--;
        }
        LocalizationMsg.RealSensorCount = RealSensorCount;
    }
    else
        return false;

//    int16_t* LaserCount = new int16_t[RealSensorCount];
//    float* startangle =new float [RealSensorCount];
//    float* endangle = new float [RealSensorCount];
//    float* installposx = new float [RealSensorCount];
//    float* installposy = new float [RealSensorCount];
//    float* installpost = new float [RealSensorCount];
//    int16_t* visualRange = new int16_t[RealSensorCount];
//    int16_t* timestmp_data = new int16_t[RealSensorCount];

    //激光器参数部分
    for(int i = 0; i < SensorCount; i++)
    {
        if(!LaserState[i]) {
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
        //            delete VisualAngleStart;
        //            delete VisualAngleEnd;
    }
//    LocalizationMsg.LaserCount = LaserCount;
//    LocalizationMsg.StartAngle = startangle;
//    LocalizationMsg.EndAngle = endangle;
//    LocalizationMsg.VisualRange = visualRange;
//    LocalizationMsg.InstallPos_X = installposx;
//    LocalizationMsg.InstallPos_Y = installposy;
//    LocalizationMsg.InstallPos_Thita = installpost;

    FileLaserParm.close();

    int ntotalsize = 0;
    int ncursize = 0;
    for(int i = 0; i < SensorCount; i++)
    {
        if(LaserState[i])
        {
            ntotalsize += (int)LocalizationMsg.LaserCount[i];
            LocalizationMsg.timestamp_data[i] = 0;
        }

    }

   // PART2 DATA
      //  cout<< "part2 totalsize: "<< ntotalsize << endl;
    for(int i = 0; i < SensorCount; i++)
    {
        ncursize += LocalizationMsg.LaserCount[i];
    }
    dis = new int16_t[ncursize];
    memset(dis,0,ncursize);
    LocalizationMsg.lenth=ncursize;
    lcmintensities = new int16_t[ncursize];
    memset(lcmintensities,0,ncursize);
    ncursize=0;

    //By yu : protect for laser pointcloud is null.
    if(pScan.point_cloud.size() <= 0)
        return false;
    for(int i = 0; i < SensorCount; i++)
    {
        if(pScan.point_cloud[i] == NULL) //By yu.do protect when point cloud is null.
            continue;
        if(LaserState[i] && (LocalizationMsg.LaserCount[i] ==
                             pScan.point_cloud[i]->num_points))
        {
            for(int j = 0; j < LocalizationMsg.LaserCount[i]; j++)
            {
                dis[ncursize + j] = pScan.point_cloud[i]->distance[j];
                lcmintensities[ncursize + j] = pScan.point_cloud[i]->intensity[j];
            }
        }
        ncursize += LocalizationMsg.LaserCount[i];
    }
    LocalizationMsg.distance = dis;
    LocalizationMsg.intensity = lcmintensities;

    LocalizationMsg.position[0] = pScan.odom_data.global_pst.x;
    LocalizationMsg.position[1] = pScan.odom_data.global_pst.y;
    LocalizationMsg.position[2] = pScan.odom_data.global_pst.fThita;
    LocalizationMsg.vel[0] = pScan.odom_data.velocity.fXLinear;
    LocalizationMsg.vel[1] = pScan.odom_data.velocity.fYLinear;
    LocalizationMsg.vel[2] = pScan.odom_data.velocity.fAngular;
    LocalizationMsg.timestamp_start = 0;
    LocalizationMsg.timestamp_vel = 0;

    bLocalMsgReady = true;

//    delete[] dis;
//    delete[] lcmintensities;

    return true;
}

//for test
bool LCMTask::GetLocalizationMsg()
{
//    if(bLocalMsgReady)
//        return false;
    //PART1 laserparm
    cout<<"part1 laserparm"<<endl;
    std::ifstream FileLaserParm(WORK_PATH"LaserParm.json");
    Json::Reader Jreader;
    Json::Value LaserParmRoot;

    if(!FileLaserParm){
        return false;
    }

    int16_t nVersion = 210;
    int16_t SensorCount = 4;
    int16_t RealSensorCount = 2;
    int16_t LaserProductor[4] = {0};
    bool LaserState[4] = {false};

    if (Jreader.parse(FileLaserParm, LaserParmRoot))
    {
        LaserParmRoot.toStyledString();

        // 文件格式版本号
        if (!LaserParmRoot["version"].isNull()) {
            LocalizationMsg.version = LaserParmRoot["version"].asInt();
        }

        //激光器个数
        if(!LaserParmRoot["laser"].isNull()) {
            SensorCount = LaserParmRoot["laser"].size();
        }
        RealSensorCount = SensorCount;
        for(int j = 0; j < SensorCount; j++) {
            if(!LaserParmRoot["laser"][j]["State"].isNull()) {
                LaserState[j] = LaserParmRoot["laser"][j]["State"].asBool();
            }
            else {
                LaserState[j] = false;
            }

            if(!LaserState[j])
                RealSensorCount--;
        }
        LocalizationMsg.RealSensorCount = RealSensorCount;
    }
    else
        return false;

    int16_t* LaserCount = new int16_t[RealSensorCount];
    float* startangle =new float [RealSensorCount];
    float* endangle = new float [RealSensorCount];
    float* installposx = new float [RealSensorCount];
    float* installposy = new float [RealSensorCount];
    float* installpost = new float [RealSensorCount];
    int16_t* visualRange = new int16_t[RealSensorCount];
    int16_t* timestmp_data = new int16_t[RealSensorCount];

    //激光器参数部分
    for(int i = 0; i < SensorCount; i++)
    {
        if(!LaserState[i]) {
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
            startangle[i] = float(val_d);
        }
        else {
            startangle[i] = 0.0f;
        }

        if(!LaserParmRoot["laser"][i]["EndAngle"].isNull()) {
            val_d = LaserParmRoot["laser"][i]["EndAngle"].asDouble() / 180.0 * PI;
            endangle[i] = float(val_d);
        }
        else {
            endangle[i] = 0.0f;
        }

        if(!LaserParmRoot["laser"][i]["LaserCount"].isNull()) {
            LaserCount[i] = LaserParmRoot["laser"][i]["LaserCount"].asInt();
        }
        else {
            LaserCount[i] = 0;
        }
        //LocalizationMsg.LaserCount[i] = LaserCount[i];

        if(!LaserParmRoot["laser"][i]["x"].isNull()) {
            val_d = LaserParmRoot["laser"][i]["x"].asDouble();
            installposx[i] = float(val_d);
        }
        else {
            installposx[i] = 0.0;
        }


        if(!LaserParmRoot["laser"][i]["y"].isNull()) {
            val_d = LaserParmRoot["laser"][i]["y"].asDouble();
            installposy[i] = float(val_d);
        }
        else {
            installposy[i] = 0.0;
        }


        if(!LaserParmRoot["laser"][i]["thita"].isNull()) {
            val_d = LaserParmRoot["laser"][i]["thita"].asDouble() / 180.0 * PI;
            installpost[i] = float(val_d);
        }
        else {
            installpost[i] = 0.0;
        }

        if(!LaserParmRoot["laser"][i]["VisualRange"].isNull()) {
            visualRange[i] = LaserParmRoot["laser"][i]["VisualRange"].size();
        }
        else {
            visualRange[i] = 0;
        }

        //            float* VisualAngleStart = new float [4];
        //            float* VisualAngleEnd = new float [4];

        for(int j = 0; j < visualRange[i]; j++ )
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
        //            delete VisualAngleStart;
        //            delete VisualAngleEnd;
    }
//    LocalizationMsg.LaserCount = LaserCount;
//    LocalizationMsg.StartAngle = startangle;
//    LocalizationMsg.EndAngle = endangle;
//    LocalizationMsg.VisualRange = visualRange;
//    LocalizationMsg.InstallPos_X = installposx;
//    LocalizationMsg.InstallPos_Y = installposy;
//    LocalizationMsg.InstallPos_Thita = installpost;

    FileLaserParm.close();

    int ntotalsize = 0;
    int ncursize = 0;
    for(int i = 0; i < SensorCount; i++)
    {
        if(LaserState[i])
        {
            ntotalsize += (int)LocalizationMsg.LaserCount[i];
            timestmp_data[i] = 0;
        }

    }

   // PART2 DATA
        cout<< "part2 totalsize: "<< ntotalsize << endl;
        for(int i = 0; i < SensorCount; i++)
        {
            if(1/*LaserState[i] && (LocalizationMsg.LaserCount[i] ==
                                 pScan.point_cloud[i]->num_points)*/)
            {
                for(int j = 0; j < LocalizationMsg.LaserCount[i]; j++)
                {
                    LocalizationMsg.distance[ncursize + j] = 1555+j;//pScan.point_cloud[i]->distance[j];
                    LocalizationMsg.intensity[ncursize + j] = 221;//pScan.point_cloud[i]->intensity[j];
                }
            }
            ncursize += LocalizationMsg.LaserCount[i];
        }

    LocalizationMsg.position[0] = 0.1f;//pScan.odom_data.global_pst.x;
    LocalizationMsg.position[1] = 0.2f;//pScan.odom_data.global_pst.y;
    LocalizationMsg.position[2] = 0.3f;//pScan.odom_data.global_pst.fThita;
    LocalizationMsg.vel[0] = 0.4f;//pScan.odom_data.velocity.fXLinear;
    LocalizationMsg.vel[1] = 0.5f;//pScan.odom_data.velocity.fYLinear;
    LocalizationMsg.vel[2] = 0.6f;//pScan.odom_data.velocity.fAngular;
    //LocalizationMsg.timestamp_data = timestmp_data;
    LocalizationMsg.timestamp_start = 0;
    LocalizationMsg.timestamp_vel = 0;
    cout<<"pos: "<<LocalizationMsg.position[0]<<" "<< LocalizationMsg.position[1] <<" "<< LocalizationMsg.position[2]<<endl;
    cout<<"pos: "<<LocalizationMsg.vel[0]<<" "<< LocalizationMsg.vel[1] <<" "<< LocalizationMsg.vel[2]<<endl;
    LCMSendLocalizationMsg();
    delete LaserCount;
    delete startangle;
    delete endangle;
    delete visualRange;
    delete installpost;
    delete installposx;
    delete installposy;
    delete timestmp_data;
    bLocalMsgReady = true;
    // _LocalizationTest_Msg test;
    //    LocalizationMsgTest.RealSensorCount = 1;
    //    LocalizationMsgTest.version = 210;
    //    LocalizationMsgTest.position[0] = 0.1f;//pScan.odom_data.global_pst.x;
    //    LocalizationMsgTest.position[1] = 0.2f;//pScan.odom_data.global_pst.y;
    //    LocalizationMsgTest.position[2] = 0.3f;//pScan.odom_data.global_pst.fThita;
    //    LocalizationMsgTest.vel[0] = 0.4f;//pScan.odom_data.velocity.fXLinear;
    //    LocalizationMsgTest.vel[1] = 0.5f;//pScan.odom_data.velocity.fYLinear;
    //    LocalizationMsgTest.vel[2] = 0.6f;//pScan.odom_data.velocity.fAngular;
    //    int16_t * dis = new int16_t [5];
    //    for(int i =0; i< 5; i++)
    //    {
    //        dis[i] = i;
    //    }
    //    LocalizationMsgTest.dis = dis;
    //    bLocalMsgReady = true;
    //    cout<<"publish"<<endl;
    //    LocalizationTest_Msg_publish(lcm, "LOCALIZATIONMSG",&LocalizationMsgTest);
    //    delete dis;
    return true;
}

/*
 * lcm 向 pad 发送新的关键帧
 * param[in] id         帧ID
 * param[in] mapinfo    地图信息
 * param[in] ptsize     激光点数
 * param[in] ptinfo     激光数据
 */
void LCMTask::SendScan ( int id, float* mapinfo, int ptsize, float *ptinfo )
{
    laser_t cmd;
    long systemtime     = (long)time( (time_t *)0 );

    cmd.utime           = systemtime;
    cmd.rad0            = id;
    cmd.nranges         = ptsize;
    cmd.ranges          = ptinfo;
    cmd.nintensities    = 6;
    cmd.intensities     = mapinfo;

    std::cout << "SendScan "<<ptinfo[0]<<" "<<ptinfo[1]<<" "<<ptinfo[2]<<std::endl;


   // printf ( "UPDATE_SCAN\n" );
    laser_t_publish ( lcm, "UPDATE_POS", &cmd );
}

//
// by lishen  向 pad 发送优化后的地图
//
void LCMTask::SendTrajNodePoses ( std::map<int, Pose> &optRes )
{
    map < int, Pose >::iterator iter;
    iter = optRes.begin();

    int optnum = optRes.size();

    if ( optnum <= 0 )
        return;

    int size = optnum;

    // printf ( "OptLocResCallBackFunc  num = %d", optnum );
    laser_t cmd;
    float   *ptInfo = new float[size * 4];
    float   pttmp[1];
    pttmp[0] = 1;
    long systemtime = (long)time( (time_t *)0 );

    int j = 0;


    while ( iter != optRes.end() )
    {
        ptInfo[4 * j + 0] = iter->first;
        ptInfo[4 * j + 1] = iter->second.x;
        ptInfo[4 * j + 2] = iter->second.y;
        ptInfo[4 * j + 3] = iter->second.theta;

        iter++;
        j++;
    }


    cmd.utime        = systemtime;
    cmd.nranges      = size * 4;
    cmd.ranges       = ptInfo;
    cmd.nintensities = 1;
    cmd.intensities  = pttmp;

    laser_t_publish ( lcm, "OPT_POSE", &cmd );



    delete ptInfo;
}

//
// by dq  向 pad 发送优化后的子图位姿
//
void LCMTask::SendSubMapsPoses (int size, vector <Pose> &optRes )
{
    std::cout<<"Send OPT SubMapsPoses!!!"<<std::endl;
    std::cout<<"send submaps num: "<<size<<std::endl;
    laser_t cmd;
    float   *ptInfo = new float[size * 4];
    float   pttmp[1];
    pttmp[0] = 1;
    long systemtime = (long)time( (time_t *)0 );

    for(int j = 0; j<size; j++)
    {
        ptInfo[4 * j + 0] = j;
        ptInfo[4 * j + 1] = optRes[j].x;
        ptInfo[4 * j + 2] = optRes[j].y;
        ptInfo[4 * j + 3] = optRes[j].theta;
        //std::cout<<"$$$$$$ SubmapIndex: "<<ptInfo[4 * j + 0]<<", x: "<<ptInfo[4 * j + 1] <<", y: "<<ptInfo[4 * j + 2]<<", theta: "<<ptInfo[4 * j + 3]<<std::endl;
    }


    cmd.utime        = systemtime;
    cmd.nranges      = size * 4;
    cmd.ranges       = ptInfo;
    cmd.nintensities = 1;
    cmd.intensities  = pttmp;

    if(size > 0)
        laser_t_publish ( lcm, "OPT_POSE", &cmd );



    delete ptInfo;
}

//
// by dq  向 pad 发送local子图信息
//
void LCMTask::SendSubMaps (int submap_index, submap_data pdata, std::vector<double> &blackcell_index, bool bSend )
{
    laser_t cmd;
    float   *pttmp = new float[blackcell_index.size()];
    float   ptInfo[8];
    long systemtime = (long)time( (time_t *)0 );

    ptInfo[0] = pdata.num_x_cells;
    ptInfo[1] = pdata.num_y_cells;
    ptInfo[2] = pdata.max_x;
    ptInfo[3] = pdata.max_y;
    ptInfo[4] = pdata.slice_pose.theta;
    ptInfo[5] = pdata.slice_pose.x;
    ptInfo[6] = pdata.slice_pose.y;
    ptInfo[7] = 0;

    std::cout<<">>>>>>>>>>>submap_index: "<<submap_index<<"blackcell_index size: "<<blackcell_index.size()<<std::endl;
    std::cout<<"num_x_cells: "<<pdata.num_x_cells<<", num_y_cells: "<<pdata.num_y_cells<<std::endl;
    std::cout<<"max_x: "<<pdata.max_x<<", max_y: "<<pdata.max_y<<std::endl;
    std::cout<<"slice_pose.x: "<<pdata.slice_pose.x<<", slice_pose.y: "<<pdata.slice_pose.y<<", slice_pose.theta: "<<pdata.slice_pose.theta<<std::endl;

    for(int i = 0; i < blackcell_index.size(); i++)
    {
        pttmp[i] = blackcell_index[i];
    }

    cmd.utime        = systemtime;
    cmd.nranges      = 8;
    cmd.ranges       = ptInfo;
    cmd.nintensities = blackcell_index.size();
    cmd.intensities  = pttmp;
    cmd.rad0         = submap_index;
   // if(blackcell_index.size()>5 )
     laser_t_publish ( lcm, "UPDATE_SUBMAPS", &cmd );

    delete pttmp;
}

/*
 * lcm 向 pad 发送应答
 * param[in] commandId     应答ID
 */
void LCMTask::SendNaviCommand ( int commandId )
{
    //dq 11.15
    robot_control_t_new     NaviCommand;    // 导航向pad发送的应答

    NaviCommand.commandid = commandId;
    NaviCommand.ndparams = 0;
    NaviCommand.niparams = 0;
    NaviCommand.nsparams = 0;
    NaviCommand.nbparams = 0;

    //robot_control_t_publish( lcm, "NAVI_UI_COMMAND", &NaviCommand);
    robot_control_t_new_publish( lcm, "NAVI_UI_COMMAND", &NaviCommand);
    //printf ( "Navi to Pad Command = %d\n", commandId );
}

/*
 * lcm 向 pad 发送应答 扩展版
 * param[in] commandId     应答ID
 * param[in] commandId     应答参数
 */
void LCMTask::SendNaviCommandEx ( int commandId, int parm )
{
    //dq 11.15
    robot_control_t_new     NaviCommandEx;    // 导航向pad发送的应答
    int8_t*             i_params = (int8_t*)calloc ( 1, sizeof (int8_t) );

    NaviCommandEx.commandid = commandId;
    NaviCommandEx.ndparams = 0;
    NaviCommandEx.niparams = 1;
    NaviCommandEx.nsparams = 0;
    NaviCommandEx.nbparams = 0;

    i_params[0] = parm;
    NaviCommandEx.iparams = i_params;

    //robot_control_t_publish( lcm, "NAVI_UI_COMMAND", &NaviCommandEx);
    robot_control_t_new_publish( lcm, "NAVI_UI_COMMAND", &NaviCommandEx);
    //printf ( "Navi to Pad Command = %d\n", commandId );

    free (i_params);
}
// by dq send version
void LCMTask::SendVersion( int commandId )
{
    robot_control_t_new  Version;

    string version = VER_IN_BLACKBOX;

    Version.commandid = commandId;
    Version.ndparams = 0;
    Version.niparams = 0;
    Version.nsparams = 0;
    Version.nbparams = version.size();
    Version.bparams = (int8_t *)version.c_str();
    robot_control_t_new_publish(lcm, "NAVI_UI_COMMAND", &Version);
}
// by DQ 发送模板数据至Pad
void LCMTask::SendStaticObjects ( int commandId, int count, vector<vector<float> > points, vector<CPosture> &psts)
{
    robot_control_t_new  StaticObjects;
    int points_num = 0;
    for(int i = 0; i < points.size(); i ++)
    {
        // 形状
        points_num += points[i].size();
        // 位姿（x，y，z）+ floor（int 楼层数）
        points_num += 4;
    }
    //init
    int8_t*             iparams = (int8_t*)calloc (count+1, sizeof (int8_t));
    double*             dparams = (double*)calloc (points_num, sizeof (double));
    int scaler = 0;
    iparams[0] = count;
    for(int i = 0; i < count; i++)
    {
        iparams[i+1] = points[i].size();
        for(int j = 0; j < points[i].size(); j++)
        {
            dparams[j+scaler] = points[i][j];

        }
        dparams[points[i].size()+scaler] = psts[i].x;
        dparams[points[i].size()+scaler+1] = psts[i].y;        dparams[points[i].size()+scaler+2] = psts[i].fThita;
        dparams[points[i].size()+scaler+3] = 0; //floor
        scaler = points[i].size()+4;
    }
    StaticObjects.commandid = commandId;
    StaticObjects.ndparams = points_num;
    StaticObjects.niparams = count+1;
    StaticObjects.nsparams = 0;
    StaticObjects.nbparams = 0;

    StaticObjects.iparams = iparams;
    StaticObjects.dparams = dparams;
    robot_control_t_new_publish( lcm, "NAVI_UI_COMMAND", &StaticObjects);

    free (iparams);
    free (dparams);
}

// by DQ 发送区域数据至Pad
void LCMTask::SendPlans ( int commandId, int count, vector<vector<float> > plans, vector<int> type)
{
    robot_control_t_new  Plans;
    //init
    int8_t*             iparams = (int8_t*)calloc (count+1, sizeof (int8_t));
    double*             dparams = (double*)calloc (5*count, sizeof (double)); //(5：左上右下坐标点+层数)
    iparams[0] = count;
    for(int i = 0; i < count; i++)
    {
        iparams[i+1] = type[i];
        for(int j = 0; j < 5; j++)
        {
            dparams[5*i+j] = plans[i][j];

        }
    }
    Plans.commandid = commandId;
    Plans.ndparams = 5*count;
    Plans.niparams = count+1;
    Plans.nsparams = 0;
    Plans.nbparams = 0;
    Plans.iparams = iparams;
    Plans.dparams = dparams;
    robot_control_t_new_publish( lcm, "NAVI_UI_COMMAND", &Plans);
    free (iparams);
    free (dparams);
}

/*
 * by cgd 导航给 pad 发送心跳, 1s发送一次
 */
void LCMTask::SendHeart ()
{

#ifdef NAV_APP
    if ( LCMTask::m_bNaviToPad == true ) {
        // 导航向pad发送心跳, pad 请求一次, 导航应答一次
        LCMTask::m_bNaviToPad = false;
    }
    else {
        return;
    }

    //robot_control_t     HeartCommand;    // 导航向pad发送的心跳
    robot_control_t_new     HeartCommand;    // 导航向pad发送的心跳
    int8_t*             i_params = (int8_t*)calloc ( 10, sizeof (int8_t) );

    HeartCommand.commandid = 51;
    HeartCommand.ndparams = 0;
    HeartCommand.niparams = 10;
    HeartCommand.nsparams = 0;
    HeartCommand.nbparams = 0;

    int workMode = RoboManagerSingleton::GetInstance()->GetCurWorkMode();
    switch ( workMode )
    {
        case RLP_MODE_LOCALIZATION:     // RLP_MODE_LOCALIZATION = 3
            i_params[0] = 1;
            i_params[1] = 3;
            break;

        case RLP_MODE_AUTOMAPPING:      // RLP_MODE_AUTOMAPPING = 5
         {

             i_params[0] = 2;
             i_params[1] = 3;  //zheng

             auto pMapping = LaserAutoMappingSingleton::GetInstance();

             if(pMapping->GetMappingMode()==mapping::Mode_BuildMap)
                 i_params[2] = 0;

             if(pMapping->GetMappingMode()==mapping::Mode_ExpandMap)
                 i_params[2] = 1;

         }

            break;

        case RLP_MODE_STOPMAPPING:      // RLP_MODE_STOPMAPPING = 6
            {
               i_params[0] = 3;
               int status = RoboManagerSingleton::GetInstance()->GetAutoMappingState();
               //结束建图状态, 1：结束建图, 正在后端优化中; 2：结束建图, 优化完成, 询问pad是否保存地图;  3 saveing  默认 / 复位

               i_params[1] = status-1;

            }

               break;

        case RLP_MODE_MAPPING:          // RLP_MODE_MAPPING = 2
            i_params[0] = 4;
            i_params[1] = 3;
            break;

        case RLP_MODE_CALIBRATION:
            i_params[0] = 4;
            i_params[1] = 3;
            break;

    }


    HeartCommand.iparams = i_params;

    robot_control_t_new_publish( lcm, "NAVI_UI_COMMAND", &HeartCommand );
    printf ( "Navi to Pad Command = 51, iparam[0] =  %d, iparam[1]  = %d\n", i_params[0], i_params[1] );

    free (i_params);
#endif
}
//
// by lishen
//
bool LCMTask::SendGlobalPoindCloud(CPosture robotPosture,const ndt_oru::CStampedPointCloud cloudIn)
{
#ifndef DesktopRun
    if((GetTickCount()-cloudLastTime_ )> 5000)
#endif
    {

        int num = 0;
        int N = cloudIn.size()/600+1;

        vector<Pose> vtpoint;
        for(int i = 0; i < cloudIn.size() ; i=i+N)
        {
            const Eigen::Vector3d &pt = cloudIn.at(i);
            Pose res;

            double c = cos(robotPosture.fThita), s = sin(robotPosture.fThita);

            //res.x =  pt(0)*c - pt(1)*s + robotPosture.x;
            //res.y =  pt(0)*s + pt(1)*c  +  robotPosture.y;

            res.x =  pt(0);
            res.y =  pt(1);
            res.theta  = 0.0;

            vtpoint.push_back(res);
            num++;
        }
        //std::cout<<"num"<<num<<std::endl;

        laser_t cmd;
        float   *ptInfo = new float[num * 3+3];
        float   pttmp[3];
        ptInfo[0] = robotPosture.x;
        ptInfo[1] = robotPosture.y;
        ptInfo[2] = robotPosture.fThita;
        long systemtime = (long)time( (time_t *)0 );


        for(int j=0;j<num;j++)
        {
            ptInfo[3 *j + 0+3] = vtpoint.at(j).x;
            ptInfo[3 *j + 1+3] = vtpoint.at(j).y;
            ptInfo[3 *j + 2+3] = vtpoint.at(j).theta;
        }

        cmd.utime        = systemtime;
        cmd.nranges      = num * 3+3;
        cmd.ranges       = ptInfo;
        cmd.nintensities = 3;
        cmd.intensities  = pttmp;

        laser_t_publish ( lcm, "CURRENT_POINTCLOUD", &cmd );

          delete ptInfo;
    }

    return true;
}

bool LCMTask::SendBottomLaserPoindCloud(CPosture robotPosture,const ndt_oru::CStampedPointCloud &cloudIn)
{
#ifndef DesktopRun
    if((GetTickCount()-cloudLastTime_ )> 5000)
#endif
    {

        int num = 0;
        int N = cloudIn.size()/600+1;

        vector<Pose> vtpoint;
        for(int i = 0; i < cloudIn.size() ; i=i+N)
        {
            const Eigen::Vector3d &pt = cloudIn.at(i);
            Pose res;

            double c = cos(robotPosture.fThita), s = sin(robotPosture.fThita);

            res.x =  pt(0);
            res.y =  pt(1);
            res.theta  = 0.0;

            vtpoint.push_back(res);           num++;
        }
        //std::cout<<"num"<<num<<std::endl;

        laser_t cmd;
        float   *ptInfo = new float[num * 3+3];
        float   pttmp[3];
        ptInfo[0] = robotPosture.x;
        ptInfo[1] = robotPosture.y;
        ptInfo[2] = robotPosture.fThita;
        long systemtime = (long)time( (time_t *)0 );


        for(int j=0;j<num;j++)
        {
            ptInfo[3 *j + 0+3] = vtpoint.at(j).x;
            ptInfo[3 *j + 1+3] = vtpoint.at(j).y;
            ptInfo[3 *j + 2+3] = vtpoint.at(j).theta;
        }

        cmd.utime        = systemtime;
        cmd.nranges      = num * 3+3;
        cmd.ranges       = ptInfo;
        cmd.nintensities = 3;
        cmd.intensities  = pttmp;

        laser_t_publish ( lcm, "BOTTOM_POINTCLOUD", &cmd );

          delete ptInfo;
    }
   //  cloudLastTime_ = GetTickCount();

    return true;
}




bool LCMTask::WriteLaserParam(void)
{
    bool bRet = true;

    std::ifstream FileLaserParm(WORK_PATH"LaserParm.json");
    Json::Reader Jreader;
    Json::Value LaserParmRoot;

   /* if(iCalibRes== CLBT_SUCCESS_ONE_LASER || iCalibRes==CLBT_SUCCESS_TWO_LASER )
    {
    }
    else
        return false;*/

    if(!FileLaserParm )
    {
        return false;
    }

    if(Jreader.parse(FileLaserParm, LaserParmRoot))
    {
        int SensorCount = 0;
        if (!LaserParmRoot["laser"].isNull())
        {
            SensorCount = LaserParmRoot["laser"].size();
        }
        else
        {
             FileLaserParm.close();
             return false;
        }

        for(int i = 0; i < SensorCount; i++)
        {

           /* if (!LaserParmRoot["laser"][i]["StartAngle"].isNull()) {
                double fdata = LaserParmRoot["laser"][i]["StartAngle"].asDouble();
                char szText[20];
                sprintf(szText,"%.2f",fdata);
                std::string str(szText);
                LaserParmRoot["laser"][i]["StartAngle"] = str;

            }

            if (!LaserParmRoot["laser"][i]["EndAngle"].isNull()) {
                double fdata = LaserParmRoot["laser"][i]["EndAngle"].asDouble();
                char szText[20];
                sprintf(szText,"%.2f",fdata);
                std::string str(szText);
                LaserParmRoot["laser"][i]["EndAngle"] = str;
            }*/

            if(i==0 ||(iCalibRes == CLBT_SUCCESS_TWO_LASER&&i==1))
            {
                int kk = 0;
                if(i==0)
                    kk = 0;
                if(i==1)
                    kk = 6;

                if (!LaserParmRoot["laser"][i]["x"].isNull()) {
                    LaserParmRoot["laser"][i]["x"] = dLaserPos[kk+0];
                }


                if (!LaserParmRoot["laser"][i]["y"].isNull()) {
                    LaserParmRoot["laser"][i]["y"] = dLaserPos[kk+1];
                }

                if (!LaserParmRoot["laser"][i]["thita"].isNull()) {
                    LaserParmRoot["laser"][i]["thita"] = dLaserPos[kk+2];
                }

            }

           /* if (!LaserParmRoot["laser"][i]["MaxRange"].isNull()) {
                double fdata = LaserParmRoot["laser"][i]["MaxRange"].asDouble();
                char szText[20];
                sprintf(szText,"%.2f",fdata);
                std::string str(szText);
                LaserParmRoot["laser"][i]["MaxRange"] = str;
            }

            if (!LaserParmRoot["laser"][i]["MinRange"].isNull()) {
                double fdata = LaserParmRoot["laser"][i]["MinRange"].asDouble();
                char szText[20];
                sprintf(szText,"%.2f",fdata);
                std::string str(szText);
                LaserParmRoot["laser"][i]["MinRange"] = str;
            }

            int nRangeCount = 0;
            if (!LaserParmRoot["laser"][i]["VisualRange"].isNull()) {
                nRangeCount = LaserParmRoot["laser"][i]["VisualRange"].size();
            }

            for(int j = 0; j < nRangeCount; j++)
            {
                if (!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].isNull()) {
                    double fdata = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"].asDouble();
                    char szText[20];
                    sprintf(szText,"%.2f",fdata);
                    std::string str(szText);
                    LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleStart"] = str;
                }

                if (!LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].isNull()) {
                    double fdata = LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"].asDouble();
                    char szText[20];
                    sprintf(szText,"%.2f",fdata);
                    std::string str(szText);
                    LaserParmRoot["laser"][i]["VisualRange"][j]["VisualAngleEnd"] = str;
                }
            }*/
        }
    }

    FileLaserParm.close();


   // if(!bRet)
    {
        std::ofstream wFileLaserParm(WORK_PATH"LaserParm.json",std::ios::out );
        Json::StyledWriter WJ;
        if(!wFileLaserParm.is_open())
        {
             return false;
        }
        else
        {
             wFileLaserParm<<WJ.write(LaserParmRoot);
        }
        wFileLaserParm.close();
    }


    return bRet;
}

void LCMTask::SendWriteLaserParamResut( bool res )
{

    std::cout<<"SendWriteLaserParamResut  "<<res<<std::endl;
    robot_control_t_new  cmd;


    int8_t iparams[1];
    if(res)
        iparams[0] = 0;
    else
        iparams[0] = 1;

    cmd.commandid = 62;
    cmd.ndparams = 0;
    cmd.niparams = 1;
    cmd.iparams = iparams;
    cmd.nsparams = 0;
    cmd.nbparams = 0;

    robot_control_t_new_publish(lcm, "NAVI_UI_COMMAND", &cmd);
}




//
// by lishen
//
bool LCMTask::SendCalibrationResult(int res,double *lasertf)
{
#ifdef NAV_APP

    if(res == CLBT_SUCCESS_ONE_LASER || res == CLBT_SUCCESS_TWO_LASER )
    {
         std::cout << '\n' << "------Calibration Results-------" << '\n' << "LiDAR-odom x: " << lasertf[0] << '\n'
                    << "LiDAR-odom y: " << lasertf[1] << '\n' << "LiDAR-odom yaw(deg): " << lasertf[2]
                    << std::endl;
         std::cout << '\n' << "-------init laser pos-------" << '\n' << "LiDAR-odom x: " << lasertf[3] << '\n'
                    << "LiDAR-odom y: " << lasertf[4] << '\n' << "LiDAR-odom yaw(deg): " << lasertf[5]
                    << std::endl;

         if(res == CLBT_SUCCESS_TWO_LASER)
         {
            std::cout << '\n' << "-------Calibration Results-------" << '\n' << "LiDAR-odom x: " << lasertf[6] << '\n'
                    << "LiDAR-odom y: " << lasertf[7] << '\n' << "LiDAR-odom yaw(deg): " << lasertf[8]
                    << std::endl;
            std::cout << '\n' << "-------init laser pos-------" << '\n' << "LiDAR-odom x: " << lasertf[9] << '\n'
                    << "LiDAR-odom y: " << lasertf[10] << '\n' << "LiDAR-odom yaw(deg): " << lasertf[11]
                    << std::endl;

         }

    }

    iCalibRes = res;
    for(int i=0;i<12;i++)
         dLaserPos[i] = lasertf[i];

    m_bPadReceived = false;
    robot_control_t_new  cmd;
    int8_t iparams[1];
    iparams[0] = res;

    cmd.commandid = 61;
    cmd.ndparams = 12;
    cmd.dparams = dLaserPos;
    cmd.niparams = 1;
    cmd.iparams = iparams;
    cmd.nsparams = 0;
    cmd.nbparams = 0;

    robot_control_t_new_publish(lcm, "NAVI_UI_COMMAND", &cmd);

    usleep(200000);

    std::cout<<"CalibRes " <<iCalibRes<< std::endl;

    int count = 0;

    while((count<5) && (!m_bPadReceived))
    {
        robot_control_t_new_publish(lcm, "NAVI_UI_COMMAND", &cmd);
        usleep(200000);
        count++;
    }


#endif
}
bool LCMTask::SendLoadPbResult(int res, vector<int>& submapids)
{
#ifdef NAV_APP

    robot_control_t_new  cmd;
    //int8_t iparams[2];
    int niparams = 0;

    if(submapids.size()>0)
    {
        niparams = 1+submapids.size();
    }
    else
        niparams = 1;


    int8_t *iparams = new int8_t[niparams];
    iparams[0] = res;

    for(int i=0;i<submapids.size();i++)
    {
        iparams[i+1] = submapids.at(i);
    }


    cmd.commandid = 63;  //????
    cmd.ndparams = 0;
    cmd.niparams = niparams;
    cmd.iparams = iparams;
    cmd.nsparams = 0;
    cmd.nbparams = 0;

    robot_control_t_new_publish(lcm, "NAVI_UI_COMMAND", &cmd);

    delete iparams;

#endif
}

#pragma GCC pop_options
