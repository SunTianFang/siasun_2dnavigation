#include "stdafx.h"
#include "G2oOptimizer.h"

static bool hasToStop = false;
bool bNextShouldFix = false;

extern CVectPose      vectPoses;
extern CVectPointMark vectPointMarks;
extern CVectLineMark  vectLineMarks;

// 针对给定的直线，根据两个线外的点的位置，对直线进行必要的扩展
void ExpandLine(CLine& ln, const CPnt& pt1, const CPnt& pt2)
{
	// 计算给定的两个点到直线的投影点
	float fLambda1, fLambda2;
	CPnt ptFoot1, ptFoot2;
	ln.DistanceToPoint(false, pt1, &fLambda1, &ptFoot1);
	ln.DistanceToPoint(false, pt2, &fLambda2, &ptFoot2);

	// 将两个Lambda值按从小到大排序
	if (fLambda2 < fLambda1)
	{
		float f = fLambda1;
		fLambda1 = fLambda2;
		fLambda2 = f;

		CPnt pt = ptFoot1;
		ptFoot1 = ptFoot2;
		ptFoot2 = pt;
	}

	// 保存直线的两个端点
	CPnt ptStart = ln.m_ptStart;
	CPnt ptEnd = ln.m_ptEnd;

	if (fLambda1 < 0)
	{
		if (fLambda2 > 1)
		{
			ptStart = ptFoot1;
			ptEnd = ptFoot2;
		}
		else if (fLambda2 >= 0)
		{
			ptStart = ptFoot1;
		}
	}
	else if (fLambda1 <= 1)
	{
		if (fLambda2 > 1)
		{
			ptEnd = ptFoot2;
		}
	}

	ln.Create(ptStart, ptEnd);
}

///////////////////////////////////////////////////////////////////////////////

CG2oOptimizer::CG2oOptimizer()
{
	Init();
}

CG2oOptimizer::~CG2oOptimizer()
{
}

//
//   初始化优化器。
//
bool CG2oOptimizer::Init()
{
	sensorOffset = NULL;
	m_posePrev = SE2(0, 0, 0);
	m_poseCur = SE2(0, 0, 0);

	m_TranslateNoise[0] = 0.1;
	m_TranslateNoise[1] = 0.1;

	m_dRotNoise = DEG2RAD(2);

	m_LandmarkNoise[0] = DEG2RAD(0.2);
	m_LandmarkNoise[1] = 0.01;
	m_vectLineMarkPoints.clear();
//	m_vectLineMarks.clear();

	/*********************************************************************************
	* creating the optimization problem
	********************************************************************************/


	typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
	typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

	// allocating the optimizer
	auto linearSolver = g2o::make_unique<SlamLinearSolver>();
	linearSolver->setBlockOrdering(false);
//	OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
	OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

	setAlgorithm(solver);

//	SE2 sensorOffsetTransf(0.2, 0.1, -0.1);
	SE2 sensorOffsetTransf(0.0, 0.0, 0.0);

	// add the parameter representing the sensor offset
	sensorOffset = new ParameterSE2Offset;
	sensorOffset->setOffset(sensorOffsetTransf);
	sensorOffset->setId(0);
	addParameter(sensorOffset);

	// 加入初始姿态
	OnlineVertexSE2* robot = new OnlineVertexSE2;
	robot->setId(0);
	robot->setEstimate(m_posePrev);
	addVertex(robot);

	m_vectPoseId.push_back(0);

	return true;
}


//
//   在X轴原点处生成第一个观测姿态节点。
//
void CG2oOptimizer::AddInitialPose()
{
	CMapOptimizer::AddInitialPose();

	CPose pose(0, 0, 0, 0);
	vectPoses.push_back(pose);
	PoseIds.push_back(0);
}

//
//   在当前基础上增加一个观测步。
//
bool CG2oOptimizer::AddNewPose()
{
	if (CMapOptimizer::AddNewPose())
	{
		// 节点数增加一步数据
		vectPoses.resize(m_nCurStep + 1);

		// Factor数增加一步数据
//		Factors.resize(m_nCurStep + 1);
		return true;
	}
	else
		return false;
}

unsigned int CG2oOptimizer::num_steps() const
{
	return vectPoses.size();
}

unsigned int CG2oOptimizer::num_landmarks() const
{
	return m_vectLandmarkId.size();
}

//
//   当收到一个里程数据时，进行相关的处理。
//
void CG2oOptimizer::OnReceiveOdometryData(unsigned int idx_x0, unsigned int idx_x1,
	double x, double y, double t,
	double ixx, double ixy, double ixt,
	double iyy, double iyt, double itt)
{
	// 查看提供的姿态编号是否是一个新号，如果是新的，应加入新vertex
	if (!IsPoseId(idx_x1))
	{
		// 计算新的姿态
		m_poseCur = m_posePrev * SE2(x, y, t);

		OnlineVertexSE2* robot = new OnlineVertexSE2;
		robot->setId(idx_x1);
		robot->setEstimate(m_poseCur);
		addVertex(robot);

		// 将新姿态的ID号记录到“已登记姿态表”中
		m_vectPoseId.push_back((int)idx_x1);

		// 加入里程约束
		//	const Simulator::GridEdge& simEdge = simulator.odometry()[i];
		// 协方差矩阵
		Matrix3d covariance;
		covariance.fill(0.);
		covariance(0, 0) = m_TranslateNoise[0] * m_TranslateNoise[0];
		covariance(1, 1) = m_TranslateNoise[1] * m_TranslateNoise[1];
		covariance(2, 2) = m_dRotNoise * m_dRotNoise;

		// 信息矩阵
		Matrix3d information = covariance.inverse();

		OnlineEdgeSE2* odometry = new OnlineEdgeSE2;
		odometry->vertices()[0] = vertex(idx_x0);
		odometry->vertices()[1] = vertex(idx_x1);
		odometry->setMeasurement(SE2(x, y, t));
		odometry->setInformation(information);
		addEdge(odometry);

		//此处为更改，未更新之前时刻位姿
		m_posePrev = m_poseCur;
	}
}

int CG2oOptimizer::OnReceiveOdometryData(const COdometryData& data)
{
	int nNewId = CMapOptimizer::OnReceiveOdometryData(data);
	int nCount = PoseIds.size();
	int nLastId = PoseIds[nCount - 2];

	OnReceiveOdometryData(nLastId, nNewId, data.x, data.y, data.t, data.ixx, data.ixy, data.ixt, data.iyy, data.iyt, data.itt);
	
	return nNewId;
}

//
//   处理待优化的地标数据。
//
void CG2oOptimizer::OnReceivePointMarkData(int nId, const CPointMarkData& data)
{
	int nLastPoseId = PoseIds.back();
	OnReceivePointMarkData(nLastPoseId, nId, data.x, data.y, data.ixx, data.ixy, data.iyy/*, data.nType, data.uParam[0], data.uParam[1]*/);
}

//
//   当收到一个地标数据时，进行相关的处理。
//
void CG2oOptimizer::OnReceivePointMarkData(unsigned int idx_x, unsigned int idx_l,
	double x, double y, double ixx, double ixy, double iyy)
{
	// 如果这是一个新地标，需要在此添加地标节点
	if (!IsLandmarkId((int)idx_l))
	{
		OnlineVertexPointXY* landmark = new OnlineVertexPointXY;
		landmark->setId(idx_l);
		Vector2d observation(x, y);
		Vector2d landmarkPos = m_poseCur * observation;
		landmark->setEstimate(landmarkPos);                   // 地标的绝对位置
		addVertex(landmark);

		m_vectLandmarkId.push_back(idx_l);
	}

	Matrix2d covariance; covariance.fill(0.);
	covariance(0, 0) = m_LandmarkNoise[0] * m_LandmarkNoise[0];
	covariance(1, 1) = m_LandmarkNoise[1] * m_LandmarkNoise[1];
	Matrix2d information = covariance.inverse();

	// 在此添加一条边(Edge)
	OnlineEdgeSE2PointXY* landmarkObservation = new OnlineEdgeSE2PointXY;
	landmarkObservation->vertices()[0] = vertex(idx_x);
	landmarkObservation->vertices()[1] = vertex(idx_l);

	// 设置地标的观测值(相对于观测姿态)
	Vector2d l = Vector2d(x, y);
	landmarkObservation->setMeasurement(l);

	landmarkObservation->setInformation(information);
	landmarkObservation->setParameterId(0, sensorOffset->id());
	addEdge(landmarkObservation);
}

//
//   当收到一个线段数据时，进行相关的处理。
//
void CG2oOptimizer::OnReceiveLineMarkData(unsigned int idx_x, unsigned int idx_l,
	double x1, double y1, double x2, double y2)
{
	ChfLine hfline(x1, y1, x2, y2, idx_l);

	// 计算第一个点的世界坐标
	Vector2d tmppoint1(x1, y1);
	Vector2d PointWorld1(m_poseCur * tmppoint1);
	CPnt linemark1(PointWorld1(0), PointWorld1(1), idx_l);
	m_vectLineMarkPoints[idx_l - 1].push_back(linemark1);
	
	// 计算第二个点的世界坐标
	Vector2d tmppoint2(x2, y2);
	Vector2d PointWorld2(m_poseCur * tmppoint2);
	
	CPnt linemark2(PointWorld2(0), PointWorld2(1), idx_l);
	m_vectLineMarkPoints[idx_l - 1].push_back(linemark2);

	Line2D tmpline(hfline._hfl);

	// 如果这是一个新线段，需要在此添加线段角点
	if (!IsLineId((int)idx_l))
	{
		OnlineVertexLine2D* line = new OnlineVertexLine2D;
		line->setId(idx_l);

		Line2D LineWorld = m_poseCur * tmpline;
		line->setTheta(LineWorld[0]);
		line->setRho(LineWorld[1]);// 线段的绝对位置
		addVertex(line);
		m_vectLineId.push_back(idx_l);
	}

	Matrix2d covariance; covariance.fill(0.);
	covariance(0, 0) = m_LandmarkNoise[0] * m_LandmarkNoise[0];
	covariance(1, 1) = m_LandmarkNoise[1] * m_LandmarkNoise[1];
	Matrix2d information = covariance.inverse();

	// 在此添加一条边(Edge)
	OnlineEdgeSE2Line2D* lineObservation = new OnlineEdgeSE2Line2D;
	lineObservation->vertices()[0] = vertex(idx_x);
	lineObservation->vertices()[1] = vertex(idx_l);

	//使用核函数去除转换错误误差

	/*
	RobustKernelCauchy* rk = new RobustKernelCauchy;
	rk->setDelta(0.1);
	lineObservation->setRobustKernel(rk);
	*/


	// 设置线段的观测值(相对于观测姿态)
	VertexLine2D* vl = dynamic_cast<VertexLine2D*>(vertex(idx_l));
	Line2D ini(Eigen::Vector2d(vl->theta(), vl->rho()));

	Line2D obs_est = Line2D(m_poseCur.inverse() * ini);

	if (tmpline(1) * obs_est[1] < 0)
	{
		if (tmpline(0) > 0)
			tmpline(0) -= const_pi();
		else
			tmpline(0) += const_pi();
		tmpline(1) = -tmpline(1);
	}
	lineObservation->setMeasurement(tmpline);

	lineObservation->setInformation(information);
	//landmarkObservation->setParameterId(0, sensorOffset->id());
	addEdge(lineObservation);
}

void CG2oOptimizer::OnReceiveLineMarkData(int nId, const CLineMarkData& data)
{
	int nLastPoseId = PoseIds.back();
	OnReceiveLineMarkData(nLastPoseId, nId, data.x1, data.y1, data.x2, data.y2);
}

bool CG2oOptimizer::StepOptimize(int& nStep)
{
#if 1
	OnlineVertexSE2* firstRobotPose = dynamic_cast<OnlineVertexSE2*>(vertex(0));
	firstRobotPose->setFixed(true);
	setVerbose(false);
	initializeOptimization();
	optimize(10);

//	Update();
#endif

	nStep++;
	
	return true;
}

void CG2oOptimizer::Update()
{
	vectPoses.clear();
	vectPointMarks.clear();
	vectLineMarks.clear();

	m_SlamRange.Reset();

	for (size_t i = 0; i < indexMapping().size(); ++i)
	{
		// 测试Vertex的类型
		OnlineVertexSE2* v1 = dynamic_cast<OnlineVertexSE2*>(indexMapping()[i]);

		// 如果是位姿类型，说明该Vertex是机器人的姿态
		if (v1 != NULL)
		{
			SE2 estimate = v1->estimate();
			CPose pose(estimate[0], estimate[1], estimate[2], v1->id());
			vectPoses.push_back(pose);

			// 更新边界范围
			m_SlamRange.CheckPoint(estimate[0], estimate[1]);
		}
		else
		{
			// 测试Vertex是否是地标类型
			OnlineVertexPointXY* v2 = dynamic_cast<OnlineVertexPointXY*>(indexMapping()[i]);

			// 如果是，说明该Vertex是地标
			if (v2 != NULL)
			{
				Vector2d estimate = v2->estimate();
				CPnt landmark(estimate[0], estimate[1], v2->id());
				vectPointMarks.push_back(landmark);

				// 更新边界范围
				m_SlamRange.CheckPoint(estimate[0], estimate[1]);
			}
			else
			{
				// 测试Vertex是否是直线类型
				OnlineVertexLine2D* v3 = dynamic_cast<OnlineVertexLine2D*>(indexMapping()[i]);

				// 如果是，说明该Vertex是直线段
				if (v3 != NULL)
				{
					Vector2d estimate = v3->estimate();

					// 取得优化后该直线特征的参数
					CWLine line(estimate[0], estimate[1], i);
					CPnt pt1 = line.GetProjectPoint(m_vectLineMarkPoints[i][0]);
					CPnt pt2 = line.GetProjectPoint(m_vectLineMarkPoints[i][1]);

					CLine LineMark(pt1, pt2);
					for (int j = 2; j < m_vectLineMarkPoints[i].size(); j += 2)
					{
						pt1 = m_vectLineMarkPoints[i][j];
						pt2 = m_vectLineMarkPoints[i][j + 1];
						ExpandLine(LineMark, pt1, pt2);
					}

					LineMark.m_nId = i + 1;

					vectLineMarks.push_back(LineMark);

					m_SlamRange.CheckPoint(pt1);
					m_SlamRange.CheckPoint(pt2);
				}
			}
		}
	}
}

bool CG2oOptimizer::IsPoseId(int nId)
{
	for (int i = 0; i < m_vectPoseId.size(); i++)
		if (m_vectPoseId[i] == nId)
			return true;

	return false;
}

bool CG2oOptimizer::IsLandmarkId(int nId)
{
	for (int i = 0; i < m_vectLandmarkId.size(); i++)
		if (m_vectLandmarkId[i] == nId)
			return true;

	return false;
}

bool CG2oOptimizer::IsLineId(int nId)
{
	for (int i = 0; i < m_vectLineId.size(); i++)
		if (m_vectLineId[i] == nId)
			return true;

	return false;
}
