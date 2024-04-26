#include "stdafx.h"
#include <iostream>
#include "MapOptimizer.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

unsigned int m_nNextSerialId;

///////////////////////////////////////////////////////////////////////////////

//
//   当接到一个新的里程信息时进行的处理。
//
int CMapOptimizer::OnReceiveOdometryData(const COdometryData& data)
{
	if (m_nCurStep == 0)
		AddInitialPose();

//	m_nNextSerialId++;

	// 增加一个观测步
	AddNewPose();

	// 计算相关联的两个步的编号
	unsigned int nLastPoseId = m_PoseMapper[m_nCurStep - 1];
	unsigned int nCurPoseId = m_PoseMapper[m_nCurStep];

	// 添加“约束对”
	ConstraintPairs.push_back(make_pair(nLastPoseId, nCurPoseId));

	// 统计到当前步时的约束数量
	ConstraintCounts[nCurPoseId] = ConstraintPairs.size();

	// 记录新位姿的序列号
//	int nNewId = num_steps() + num_landmarks() - 1;
//	PoseIds.push_back(num_steps() + num_landmarks() - 1);
	int nNewId = m_nNextSerialId++;
	PoseIds.push_back(nNewId);

	// 返回新添加的序列号
	return nNewId;
}

//
//   处理待优化的点型地标数据。
//
void CMapOptimizer::OnReceivePointMarkData(int nId, const CPointMarkData& data)
{
}

//
//   处理待优化的直线型地标数据。
//
void CMapOptimizer::OnReceiveLineMarkData(int nId, const CLineMarkData& data)
{
}

//
//   处理接到的两个位姿的观测差距。
//
void CMapOptimizer::OnReceivePoseDifference(int nStepId, int nPrevStepId, const CPoseDiffData& data)
{
}

unsigned int NextSerialId()
{
	return m_nNextSerialId++;
}

///////////////////////////////////////////////////////////////////

//
//   析构函数。
//
CMapOptimizer::~CMapOptimizer()
{
}

//
//   在X轴原点处生成第一个观测姿态节点。
//
void CMapOptimizer::AddInitialPose()
{
	m_PoseMapper.add(0);
	m_nCurStep = 0;
	m_nNextSerialId = 1;
}

//
//   在当前基础上增加一个观测步。
//
bool CMapOptimizer::AddNewPose()
{
	// 判断给定的观测姿态是否是一个新姿态，如果是则需要添加它
	if (m_PoseMapper.add(m_nCurStep + 1))
	{
		m_nCurStep++;

		// 特征数增加一步数据
		LandmarkCounts.resize(m_nCurStep + 1);
		LandmarkCounts[m_nCurStep] = num_landmarks();

		// “约束”数增加一步数据
		ConstraintCounts.resize(m_nCurStep + 1);
		ConstraintCounts[m_nCurStep] = ConstraintPairs.size();

		// “观测”数增加一步数据
		MeasurementCounts.resize(m_nCurStep + 1);
		MeasurementCounts[m_nCurStep] = MeasurementPairs.size();

		// 添加成功
		return true;
	}

	// 不需要添加，返回false
	else
		return false;
}

//
//   在当前基础上增加一个地标。
//
bool CMapOptimizer::AddPointMark(unsigned int nId, int nType, int nParam1, int nParam2)
{
	bool bResult = false;

	if (m_PointMarkMapper.add(nId))
	{
		// 更新当前步的特征数量
		LandmarkCounts[m_nCurStep] = num_landmarks() + 1;
		bResult = true;
	}

	// 取得当前位姿、地标的ID号
	unsigned int nCurPoseId = m_PoseMapper[m_nCurStep];
	unsigned int nLandmarkId = m_PointMarkMapper[nId];

	// 记录“观测对”数据
	MeasurementPairs.push_back(make_pair(nCurPoseId, nLandmarkId));

	// 更新当前步的“观测”数量
	MeasurementCounts[nCurPoseId] = MeasurementPairs.size();

	return bResult;
}

bool CMapOptimizer::more_data(unsigned int* step)
{
	(*step)++;
	return ((*step) + 1) <= num_steps();
}

//
//   将第0~step步的所有的“约束”(即姿态节点索引号的组对)收集到一个向量中并将其返回。
//
vector<pair<int, int>> CMapOptimizer::constraints(unsigned int step) const
{
	return vector<pair<int, int>>(ConstraintPairs.begin(), ConstraintPairs.begin() + ConstraintCounts[step]);
}

//
//   将第0~step步的所有的“观测”(即“观测节点索引号”与“观测到的点的索引号”的组对)收集到一个向量中并将其返回。
//
vector<pair<int, int> > CMapOptimizer::measurements(unsigned int step) const
{
	return vector<pair<int, int>>(MeasurementPairs.begin(), MeasurementPairs.begin() + MeasurementCounts[step]);
}
