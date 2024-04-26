
#include "SendMap.h"

#include <math.h>

//////////////////////////////////////////////////////////////////////////////
namespace mapping {


 double localmap_x0 = -25;
 double localmap_y0 = -25;
 double localmap_x1 = 25;
 double localmap_y1 = 25;
 double localmap_metersPerPixel = 0.05;
 int localmap_width = 1000;
 int localmap_height = 1000;

static int localmap_id = 0;
static int index_sendopt = 0;
std::vector<stSendIdTime> vtSendIdTime;
static Pose last_send_pose;
static map<int,Pose> lastoptRes;

Pose xytInvMul31(Pose a, Pose b)
{
    Pose res;
    double theta = a.theta;
    double ca = cos(theta), sa = sin(theta);
    double dx = b.x - a.x;
    double dy = b.y - a.y;

    res.x = ca*dx + sa*dy;
    res.y = -sa*dx + ca*dy;
    res.theta = b.theta - a.theta;

    return res;
}
Pose xytMultiply(Pose a, Pose b)
{
    Pose r;
    double s = sin(a.theta), c = cos(a.theta);

    r.x = c*b.x - s*b.y + a.x;
    r.y = s*b.x + c*b.y + a.y;
    r.theta = a.theta + b.theta;

    return r;
}
//
// by lishen   send map  to pad
//
void SendMap(slam_result *pSlam, std::vector<opt_result> *pOpt, Pose &RecordPose,int old_nodenum)
{

    Pose xyt = pSlam->local_pose;


 std::cout << "xyt "<<xyt.x<<" "<<xyt.y<<" "<<xyt.theta<<std::endl;
        //send scan and map info
    if(pSlam->nodeId ==0)
    {
        localmap_x0 = -25;
        localmap_y0 = -25;
        localmap_x1 = 25;
        localmap_y1 = 25;
        localmap_metersPerPixel = 0.05;
        localmap_width = 1000;
        localmap_height = 1000;
        last_send_pose = xyt;
        localmap_id = 0;
        index_sendopt = 0;
        vtSendIdTime.clear();


    }

    if(old_nodenum == pSlam->nodeId)
        vtSendIdTime.clear();




    double dis = sqrt((xyt.x-last_send_pose.x)*(xyt.x-last_send_pose.x)+(xyt.y-last_send_pose.y)*(xyt.y-last_send_pose.y));
    double dtheta = fabs(CAngle::NormalizeAngleDifference(xyt.theta, last_send_pose.theta));


    map_info mapinfo;
    int scanid = 0;


    mapinfo.resolution  = localmap_metersPerPixel;
    mapinfo.nwidth      = localmap_width;
    mapinfo.nheight     = localmap_height;
    mapinfo.x0          = localmap_x0;
    mapinfo.y0          = localmap_y0;


  //printf(" mapinfo.nwidth = %d   %d  x0:%f   t:  %f \n", mapinfo.nwidth,mapinfo.nheight,mapinfo.x0,mapinfo.y0);



    Pose send_local_xyt = xyt;


    if((*pOpt).size()>0 && vtSendIdTime.size()>0)
    {
        Pose lastoptpos;

        Pose originpos;

        for(int j=vtSendIdTime.size()-1;j>=0;j--)
        {
            timeval tt = vtSendIdTime.at(j).time;

            bool bfind = false;

            for (const auto& node_id_data : *pOpt)
            {


                lastoptpos = node_id_data.global_pose;


                 if(node_id_data.stamp.tv_sec == tt.tv_sec && node_id_data.stamp.tv_usec == tt.tv_usec)
                {
                    bfind = true;
                    originpos = vtSendIdTime.at(j).pos;

                    //printf("lastoptpos = %f %f %f\n", lastoptpos.x, lastoptpos.y,lastoptpos.theta);
                    //printf("originpos = %f %f %f\n", originpos.x, originpos.y,originpos.theta);
                    Pose delta;
                    delta = xytInvMul31(originpos,xyt);

                    //printf("delta = %f %f %f\n", delta.x, delta.y,delta.theta);
                    send_local_xyt = xytMultiply(lastoptpos, delta);


                    //printf("***********xyt = %f %f %f\n", xyt.x, xyt.y,xyt.theta);
                    //printf("send_local_xyt = %f %f %f\n", send_local_xyt.x, send_local_xyt.y,send_local_xyt.theta);
                    break;
                }
            }
            if(bfind)
                break;
        }
    }


    float map_info[6];

    map_info[0] = mapinfo.nheight;
    map_info[1] = mapinfo.nwidth;
    map_info[2] = mapinfo.x0;
    map_info[3] = mapinfo.y0;
    map_info[4] = mapinfo.resolution;
    map_info[5] = 15;


    //  by cgd 对点云进行降采样
    int dsize   = 3;
    int step    = 4;    // 降采样步长

    if((pSlam->vtlaser).size()>0)
    {
        // dsize = dsize+ (pSlam->vtlaser).size()*3;
        dsize = dsize+ ((pSlam->vtlaser).size() / step) * 3;
    }

    float *ptInfo  = new float[dsize];

    ptInfo[0] = send_local_xyt.x;
    ptInfo[1] = send_local_xyt.y;
    ptInfo[2] = send_local_xyt.theta;



    // 对点云进行降采样
    for(int i=0 ;i < (pSlam->vtlaser).size() / step;i++)
    {
        ptInfo[3*(i+1)+0] = (pSlam->vtlaser).at(i * step).x;
        ptInfo[3*(i+1)+1] = (pSlam->vtlaser).at(i * step).y;
        ptInfo[3*(i+1)+2] = (pSlam->vtlaser).at(i * step).theta;
    }

    //std::cout<<"(pSlam->vtlaser).size()"<<(pSlam->vtlaser).size()<<std::endl;

    if(pSlam->nodeId!=-1)
    {

        if(pSlam->nodeId == 0 || dis>0.5 || dtheta>(30*3.1415/180.0))
        {
           // printf("send localmap_id = %d  x: %f  y:%f   t:  %f \n", pSlam->nodeId, ptInfo[0], ptInfo[1], ptInfo[2]);
           // printf ( "map_info[0] = %.2f, map_info[1] = %.2f, map_info[2] = %.2f, map_info[3] = %.2f\n", map_info[0], map_info[1], map_info[2], map_info[3] );

            if(pSlam->nodeId!=0)
            {

                double xmin = -15;
                double ymin = -15;
                double xmax = 15;
                double ymax = 15;

                 for ( int i = 0; i < pSlam->vtlaser.size(); i++ )
                //for ( int i = 0; i < pSlam->vtlaser.size(); i = i + 10 )
                {
                    Pose   p = pSlam->vtlaser.at(i);
                    double c = cos(xyt.theta), s = sin(xyt.theta);

                    Pose res;
                    res.x     = p.x*c - p.y*s + xyt.x;
                    res.y     = p.x*s + p.y*c + xyt.y;
                    res.theta = p.theta;

                    if (res.x<xmin)	xmin = res.x;//x0
                    if (res.x>xmax)	xmax = res.x;//x1
                    if (res.y<ymin)	ymin = res.y;//y0
                    if (res.y>ymax)	ymax = res.y;//y1
                }



                // left down
                double _x0 = xmin - 5.0 ;
                double _y0 = ymin - 5.0 ;

                // right top
                double _x1 = xmax + 5.0 ;
                double _y1 = ymax + 5.0 ;

                bool expandxy1 = false;


                if ( _x1 > localmap_x1 )
                {
                    expandxy1 = true;
                    localmap_x1 = _x1 + 10;
                }
                if ( _y1 > localmap_y1 )
                {
                    expandxy1 = true;
                    localmap_y1 = _y1 + 10;
                }

                if(!expandxy1)
                {

                    if ( _x0 < localmap_x0 )
                        localmap_x0 = _x0 - 10;
                    if ( _y0 < localmap_y0 )
                        localmap_y0 = _y0 - 10;
                }



                // double _metersPerPixel = 0.05;

                localmap_width  = (localmap_x1 - localmap_x0) / localmap_metersPerPixel;
                localmap_height = (localmap_y1 - localmap_y0) / localmap_metersPerPixel;

                map_info[0] = mapinfo.nheight;
                map_info[1] = mapinfo.nwidth;
                map_info[2] = mapinfo.x0;
                map_info[3] = mapinfo.y0;
                map_info[4] = mapinfo.resolution;
                map_info[5] = 15;


               // printf(" x1 = %f  y1= %f  x0=%f   y0 %f \n", localmap_x1,localmap_y1,localmap_x0,localmap_y0);

            }

            localmap_id = pSlam->nodeId;
            LCMTask::GetLcmInstance().SendScan ( localmap_id, map_info, dsize, ptInfo );
           // LCMTask::GetLcmInstance().SendScan ( -1, map_info, dsize, ptInfo );
            stSendIdTime idtime;
            idtime.id = localmap_id;
            idtime.time = pSlam->stamp;
            idtime.pos = xyt;
            vtSendIdTime.push_back(idtime);

            localmap_id++;
            last_send_pose = xyt;
        }



    }
   /* else
    {
        if ( index_sendopt % 10 == 0 ) {
           // printf ( "localmap_id = -1\n" );
            LCMTask::GetLcmInstance().SendScan( -1, map_info, dsize, ptInfo );
        }
    }*/

    index_sendopt++;


    if(pSlam->nodeId%40==0)
    {

        map<int,Pose> optRes;

        Pose delta;
        int lastj = 0;
        bool bChanged = false;

        for (const auto& node_data : *pOpt) {

            timeval opttime = node_data.stamp;

            for(int j=0;j<vtSendIdTime.size();j++)
            {
                timeval tt = vtSendIdTime.at(j).time;
                if(opttime.tv_sec == tt.tv_sec && opttime.tv_usec == tt.tv_usec)
                {
                    optRes[vtSendIdTime.at(j).id] = node_data.global_pose;

                    delta =  xytInvMul31(node_data.global_pose, vtSendIdTime.at(j).pos);

                    lastj = j;

                    map<int,Pose>::iterator iterlast = lastoptRes.find(j);

                    if(!bChanged && iterlast!=lastoptRes.end())
                    {
                         if(fabs(node_data.global_pose.x-lastoptRes[j].x)>0.001
                            || fabs(node_data.global_pose.y-lastoptRes[j].y)>0.001
                            ||fabs(node_data.global_pose.theta-lastoptRes[j].theta)>0.001)
                         {
                              bChanged = true;
                            //  std::cout<<"bChanged&&&&&&&&&&&&&&&&&&&<<<<<<<<<<<<<<<< ?????????????????????????????????????????????????????????????????"<<std::endl;
                         }

                    }
                }
            }

        }

        for(int j=lastj+1;j<vtSendIdTime.size();j++)
        {

            Pose before = vtSendIdTime.at(j).pos;
            Pose after = xytMultiply(before, delta);
            optRes[vtSendIdTime.at(j).id] = after;
        }

        //if(bChanged)
            //LCMTask::GetLcmInstance().SendTrajNodePoses(optRes);

        lastoptRes.clear();
        lastoptRes = optRes;
    }

    // printf ( "test x = %f, y = %f, theta = %f,\n", xyt.x, xyt.y, xyt.theta );
    RecordPose = send_local_xyt;
    delete ptInfo;
}

void ClearIndex ( void )
{
    index_sendopt = 0;
}


} // namespace mapping


