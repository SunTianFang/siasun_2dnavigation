
#pragma once

#include <stdio.h>
#include <pthread.h>
#include "type.h"
#include <LCMTask.h>


namespace mapping {

struct stSendIdTime
{
    Pose pos;
    timeval time;
    int id;
};


void SendMap(slam_result *pSlam, std::vector<opt_result> *pOpt, Pose &RecordPose,int old_nodenum);

void ClearIndex ( void );

} // namespace mapping


