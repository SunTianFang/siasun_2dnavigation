#ifndef _LOCATION_H_
#define _LOCATION_H_
#include "pointCloud_T.h"
#include "Grid2d_T.h"
#include "mytypes.h"


//by smz

typedef struct Location_Summary{
	double final_cost;
    myceres::TerminationType termination_type; //TerminationType
}T_L_SUMMARY;

typedef struct Cere_Weight{

  double space_weight;
  double translation_weight;
  double rotation_weight;
}T_Cere_Weight;

void LoadData();
void Optimize(Grid2D_T &grid,PointCloud_T &pc_cloud,T_Cere_Weight &weights,double &x1,double &y1,double &t1,T_L_SUMMARY &summary);
#endif
