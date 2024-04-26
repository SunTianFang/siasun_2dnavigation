#pragma once

#include <vector>
#include <map>
#include "MapOptimizer.h"
#include "ScrnRef.h"
#include "LaserSurveyMsg.h"
#include "Geometry.h"

#include "g2o/stuff/misc.h"
#include "g2o/types/slam2d/se2.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam2d_addons/line_2d.h"
#include "g2o/types/slam2d_addons/types_slam2d_addons.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "types_slam2d_online.h"
#include "graph_optimizer_sparse_incremental.h"
#include "g2o_slam_interface.h"

//#define SLAM_INCREMENTAL

using namespace std;
using namespace g2o;
using namespace Eigen;

class ChfLine
{
public:
	Vector2d _hfl;
	int      _id;

public:
	ChfLine(double x1, double y1, double x2, double y2, int id)
	{
		double beta;
		if ((x1 - x2) * (y1 - y2) > 0)
			beta = atan2(y2 - y1, x2 - x1) + const_pi() / 2;
		else
			beta = atan2(y2 - y1, x2 - x1) - const_pi() / 2;

		double rho = x1 * cos(beta) + y1 * sin(beta);

		if (rho < 0)
		{
			beta = beta - const_pi();
			rho = -rho;
		}

		_hfl = Vector2d(beta, rho);

		_id = id;
	}

	ChfLine(int id)
	{
		_hfl << 0, 0;
		_id = id;
	}

	ChfLine()
	{
		_hfl << 0, 0;
		_id = -1;
	}
};

class CWLine : public CLineBase
{
public:
	double _theta;
	double _rho;
	int _id;
public:
	CWLine(double theta, double rho, int id)
	{
		_theta = theta;
		_rho = rho;
		_id = id;
		a = cos(_theta);
		b = sin(_theta);
		c = -_rho;
		if (a < 0)
		{
			a = -a;
			b = -b;
			c = -c;
		}

	}
};

class CSlamRange
{
public:
	double left;
	double right;
	double top;
	double bottom;

public:
	CSlamRange()
	{
	}

	void Reset()
	{
		left = 10000;
		right = -10000;
		top -= 10000;
		bottom = 10000;
	}

	void CheckPoint(double x, double y)
	{
		if (x < left)
			left = x;

		if (x > right)
			right = x;

		if (y > top)
			top = y;

		if (y < bottom)
			bottom = y;
	}

	void CheckPoint(const CPnt& pt)
	{
		CheckPoint(pt.x, pt.y);
	}
};

///////////////////////////////////////////////////////////////////////////////
//   定义“Slam优化器”类型。
class CG2oOptimizer : public SparseOptimizer, public CMapOptimizer
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
	ParameterSE2Offset* sensorOffset;
	Vector2d m_TranslateNoise;                 // 位姿检测的误差
	double m_dRotNoise;                      // 转动的误差
	Vector2d m_LandmarkNoise;               // 地标检测的误差
	SE2 m_posePrev;                       // 上一次的姿态
	SE2 m_poseCur;                        // 当前姿态
	vector<int> m_vectPoseId;
	vector<int> m_vectLandmarkId;
	vector<int> m_vectLineId;
	Vector2d ref_hfl;

	map<int, vector<CPnt, Eigen::aligned_allocator<CPnt>>> m_vectLineMarkPoints;
//	map<int, CLine> m_vectLineMarks;

	CSlamRange m_SlamRange;

private:
	bool IsPoseId(int nId);
	bool IsLandmarkId(int nId);
	bool IsLineId(int nId);

	// 初始化
	bool Init();

public:
	void OnReceiveOdometryData(unsigned int idx_x0, unsigned int idx_x1,
		double x, double y, double t, double ixx, double ixy, double ixt, double iyy, double iyt, double itt);

	void OnReceivePointMarkData(unsigned int idx_x, unsigned int idx_l, double x, double y,
		double ixx, double ixy, double iyy);

	void OnReceiveLineMarkData(unsigned int idx_x, unsigned int idx_l, double x1, double y1,
		double x2, double y2);

	void OnDataEnd();

	void GetRange(double& left, double& right, double& top, double& bottom)
	{
		left = m_SlamRange.left;
		right = m_SlamRange.right;
		top = m_SlamRange.top;
		bottom = m_SlamRange.bottom;

		if (left > right)
		{
			left = -1;
			right = 1;
		}

		if (bottom > top)
		{
			bottom = -1;
			top = 1;
		}
	}

public:
	CG2oOptimizer();
	virtual ~CG2oOptimizer();

	virtual void AddInitialPose();
	virtual bool AddNewPose();

	virtual unsigned int num_steps() const;
	virtual unsigned int num_landmarks() const;

	virtual int OnReceiveOdometryData(const COdometryData& data);
	virtual void OnReceivePointMarkData(int nId, const CPointMarkData& data);
	virtual void OnReceiveLineMarkData(int nId, const CLineMarkData& data);

	virtual bool StepOptimize(int& nStep);
	virtual void Update();
};
