#pragma once

#include "Geometry.h"

//
//   对应于曲线上各离散采样点的数据。
//
class CCurveSamplePoint : public CPnt
{
public:
	float t;                           // 控制参量
	float fProgress;                   // 从起始点到本采样点的曲线长度
	float fSegLen;                     // 从上一采样点到本采样点的长度
	float fTangentAngle;               // 切线方向角
	float fCurvature;                  // 曲率值
	float fUserData[10];               // (预留给)定户自行定义的数据

public:
	CCurveSamplePoint()
	{
		t = 0;
		fProgress = 0;
		fSegLen = 0;
		fTangentAngle = 0;
		fCurvature = 0;

		for (int i = 0; i < 10; i++)
			fUserData[i] = 0;
	}
};

///////////////////////////////////////////////////////////////////////////////
//   定义Bezier曲线。
class DllExport CBezier
{
private:
	float dx1;
	float dy1;
	float dx2;
	float dy2;

	int   m_nSampleCount;                    // 采样点数量
	CCurveSamplePoint* m_pSamplePoints;      // 采样点数据缓冲区首指针

public:
	int   m_nCountKeyPoints;
	CPnt* m_ptKey;
	CPnt m_pt;              // The trajectory point
	CAngle m_angTangent;
	float  m_fCurvature;
	float  m_fTotalLen;         // 曲线长度

private:
	void(*UserDataCreateProc)(CBezier*, void*);

	// 清除空间
	void Clear();

public:
	// 构造任意阶贝塞尔曲线
	CBezier(int nCountKeyPoints, CPnt* pptKey);

	// 构造三阶贝塞尔曲线
	CBezier(const CPosture& pstStart, const CPosture& pstEnd, float fLen1, float fLen2);
	
	// 缺省构造函数(后续需要显式调用Create函数)
	CBezier();
	
	// 析构函数
	~CBezier();

	// 重载“=”操作符
	void operator = (const CBezier& Obj);

	// 根据给定的关键点生成任意阶Bezier曲线
	bool Create(int nCountKeyPoints, CPnt* pptKey);

	// 生成三阶Bezier曲线
	bool Create(const CPosture& pstStart, const CPosture& pstEnd, float fLen1, float fLen2);

	// 根据选点常数K生成三阶Bezier曲线
	bool Create(const CPosture& pstStart, const CPosture& pstEnd, float K);

	// 生成离散采样点数据
	bool CreateSamplePoints();

	// 在指定的序号处增加一个控制点
	bool AddKeyPoint(int nIdx, const CPnt& pt);

	// 将指定序号处的控制点移除
	bool RemoveKeyPoint(int nIdx);

	// 根据给定的处理函数，生成曲线的用户数据
	void CreateUserData(void(*pProc)(CBezier*, void*), void* pParam);

	// 取得指定的用户数据
	float GetUserData(int nSampleIdx, int nDataIdx) const;

	// 核对一下，看曲线是否突破了规定的约束
	bool CheckConstraints(float fMaxCurvature, float fMaxCurvatureDiff) const;

	// 设置当前进度以便确定当前点
	virtual void SetCurT(float t);

	// The trajectory generation function
	virtual CPnt TrajFun() const {return m_pt;}

	// The tangent angle generation function
	virtual CAngle TangentFun() const {return m_angTangent;}

	// The curvature generation function
	virtual float CurvatureFun() const {return m_fCurvature;}

	// 根据距起点的进度距离确定对应的t值
	float GetTFromProgress(float fLen);

	// 计算曲线外一点pt到曲线上最近距离的点
	bool GetClosestPoint(const CPnt& pt, CPnt* ptClosest = NULL, float* t = NULL);

	// 从文本文件装入曲线数据
	bool Create(FILE *StreamIn);

	// 将曲线数据保存到文本文件
	bool Save(FILE *StreamOut);

#ifdef _MFC_VER
	// 从二进制文件装入曲线数据
	bool Create(CArchive& ar);

	// 将曲线数据保存到二进制文件
	bool Save(CArchive& ar);

	// 绘制曲线
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPointSize, bool bShowKeyPoints = false, int nPenStyle = PS_SOLID);

	// 画出控制点
	void DrawCtrlPoints(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nPointSize);
#endif
};

