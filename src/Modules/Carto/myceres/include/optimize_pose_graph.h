#ifndef _POSE_GRAPH_H_
#define _POSE_GRAPH_H_
#include "mytypes.h"
#include <map>
#include <vector>

// by smz
typedef struct PoseGraph_Summary{
	double final_cost;
    myceres::TerminationType termination_type;
	
}T_PG_SUMMARY;

void Optimize_Posegraph(double *submappos,double *nodepos,double * constraintspos,double *weight,int *pair,int *ifloss,int c1,int c2,int s1,int n1,T_PG_SUMMARY &summary);
void LoadPoseGraphData();



//by lishen
void SetFrozen(int submapnum,int nodenum);
void Optimize_Posegraph_new(std::map<int, std::vector<double> > &submappos,std::map<int, std::vector<double> > &nodepos,double * constraintspos,double *weight,int * pair,int *ifloss,int c1,int c2,int s1,int n1,T_PG_SUMMARY &summary);
void Optimize_Posegraph_new2(std::map<int, std::vector<double>> &submappos,std::map<int, std::vector<double>> &nodepos,double * constraintspos,double *weight,int * pair,int *ifloss,int c1,int c2,int s1,int n1,T_PG_SUMMARY &summary);

#endif
