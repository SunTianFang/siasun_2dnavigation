#ifndef __CLeastSquareMethod
#define __CLeastSquareMethod

#include "CMatrix.h"

///////////////////////////////////////////////////////////////////////////////
//   最小二乘法。
class CLeastSquareMethod
{
public:
	int m_nMaxRows;      // 最在允许的行数量
	int m_nRows;         // 实际的行数量
	int m_nCols;         // 列的数量

	CMatrix A;           // 最小二乘法等式左侧矩阵
	CMatrix B;           // 最小二乘法等式右侧列向量

public:

	CLeastSquareMethod(){}

	CLeastSquareMethod(int nMaxRows, int nCols);

	// 重新开始设置矩阵参数
	void Start();

	// 加入一行约束条件
	bool AddRow(float* pa, float b);

	// 求解最小二乘法
	bool Solve(float* pX, int nNum);

#ifdef _MSC_VER
	void Dump();
#endif
};
#endif
