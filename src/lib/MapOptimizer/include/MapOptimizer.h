#pragma once

#include "LaserSurveyMsg.h"
#include <vector>
#include <list>
#include <map>
#include <Eigen/Dense>
#include "Geometry.h"

using namespace std;
using namespace Eigen;

//////////////////////////////////////////////////////////////////////////////

class CPose
{
public:
	Vector3d _p;
	int      _id;

public:
	CPose(double x, double y, double t, int id)
	{
		_p[0] = x;
		_p[1] = y;
		_p[2] = t;
		_id = id;
	}

	CPose(int id)
	{
		_p[0] = _p[1] = _p[2] = 0;
		_id = id;
	}

	CPose()
	{
		_p[0] = _p[1] = _p[2] = 0;
		_id = -1;
	}

	CPosture ToCPosture() const
	{
		return CPosture(_p[0], _p[1], _p[2]);
	}
};

typedef vector<CPose, Eigen::aligned_allocator<CPose>> CVectPose;
typedef vector<CPnt, Eigen::aligned_allocator<CPnt>> CVectPointMark;
typedef vector<CLine, Eigen::aligned_allocator<CLine>> CVectLineMark;

///////////////////////////////////////////////////////////////////////////////
//   自动产生和保存索引号的工具类。
class IndexMapper
{
private:
	int m_nNextId;                  // 下一个可用索引值
	map<int, int> m_MapIndex;       // 索引表

public:
	IndexMapper() { m_nNextId = 0; }

	//
	//   利用[]操作符提供转换后的引用值
	//
	int operator[](int i)
	{
		map<int, int>::iterator it = m_MapIndex.find(i);

		// i所对应的项必须存在，否则报错
		assert(it != m_MapIndex.end());
		return m_MapIndex[i];
	}

	// 增加一个对应于键值i的项
	//   返回值：
	//     false - 原来就存在对应于索引值i的项，添加失败
	//     true  - 添加成功
	//
	bool add(int i)
	{
		// 找到对应于i的项
		map<int, int>::iterator it = m_MapIndex.find(i);

		// 如果不存在，则在此添加该项，并标记它为一个序列号
		if (it == m_MapIndex.end())
		{
			// if entry does not exist, create one with next ID
			m_MapIndex[i] = m_nNextId++;
			return true;
		}

		// 如果已存在，则返回false
		else
			return false;
	}
};

///////////////////////////////////////////////////////////////////////////////
// 定义地图优化器类型
class CMapOptimizer
{
protected:
	int         m_nCurStep;
	IndexMapper m_PoseMapper;
	IndexMapper m_PointMarkMapper;

	// 对于每一步，所观测到的(截止到当前步的)特征的数量
	vector<int> LandmarkCounts;

	// indices into PoseNodePtrs vector
	// 对于每一步，所形成的里程之间的“约束”的数量
	vector< pair<int, int> > ConstraintPairs;

	// number of measurements up to each time step
	vector<int> ConstraintCounts;

	// indices into PoseNodePtrs and LandmarkNodePtrs vectors
	vector< pair<int, int> > MeasurementPairs;

	// number of measurements up to each time step
	// 到每一步时的“观测”的数量所构成的向量
	vector<int> MeasurementCounts;

	vector<int> PoseIds;

public:
	/**
* Returns true if step was not the last step.
*/
	bool more_data(unsigned int* step);

	/**
	* Returns the number of points encountered up to the given step.
	* @param step Time step.
	* @return Number of points up to step.
	*/
	// 统计从0~step步发现的所有点的数量
	unsigned int num_points(unsigned int step) const
	{
		return LandmarkCounts[step];
	}

	// 将第0~step步的所有的“约束”(即姿态节点索引号的组对)收集到一个向量中并将其返回
	vector<pair<int, int>> constraints(unsigned int step) const;

	// 将第0~step步的所有的“观测”(即“观测节点索引号”与“观测到的点的索引号”的组对)收集到一个向量中并将其返回
	vector<pair<int, int>> measurements(unsigned int step) const;

public:
	CMapOptimizer() 
	{
		m_nCurStep = 0;
	}

	virtual ~CMapOptimizer();

//	virtual bool Init() { return true; }

	virtual bool AddNewPose();
	virtual bool AddPointMark(unsigned int nId, int nType, int nParam1, int nParam2);

	virtual unsigned int num_steps() const { return 0; }
	virtual unsigned int num_landmarks() const { return 0; }
	virtual void AddInitialPose();
	virtual int OnReceiveOdometryData(const COdometryData& data);

	virtual void OnReceivePointMarkData(int nId, const CPointMarkData& data);
	virtual void OnReceiveLineMarkData(int nId, const CLineMarkData& data);

	// 处理接到的两个位姿的观测差距
	virtual void OnReceivePoseDifference(int nStepId, int nPrevStepId, const CPoseDiffData& data);

	virtual bool StepOptimize(int& nStep) = 0;
	virtual void Update() = 0;
};
