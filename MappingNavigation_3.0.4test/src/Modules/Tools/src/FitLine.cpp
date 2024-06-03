#include "stdafx.h"
#include <math.h>
#include "FitLine.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/**
* 最小二乘法直线拟合（不是常见的一元线性回归算法）
* 将离散点拟合为  a x + b y + c = 0 型直线
* 假设每个点的 X Y 坐标的误差都是符合 0 均值的正态分布的。
* 与一元线性回归算法的区别：一元线性回归算法假定 X 是无误差的，只有 Y 有误差。
*/
bool lineFit(vector<CPnt> &points, double &a, double &b, double &c)
{
	int size = points.size();
	if (size < 2)
	{
		a = 0;
		b = 0;
		c = 0;
		return false;
	}

	double x_mean = 0;
	double y_mean = 0;
	for (int i = 0; i < size; i++)
	{
		x_mean += points[i].x;
		y_mean += points[i].y;
	}
	x_mean /= size;
	y_mean /= size; //至此，计算出了 x y 的均值

	double Dxx = 0, Dxy = 0, Dyy = 0;

	for (int i = 0; i < size; i++)
	{
		Dxx += (points[i].x - x_mean) * (points[i].x - x_mean);
		Dxy += (points[i].x - x_mean) * (points[i].y - y_mean);
		Dyy += (points[i].y - y_mean) * (points[i].y - y_mean);
	}

	double lambda = ((Dxx + Dyy) - sqrt((Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy)) / 2.0;
	double den = sqrt(Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx));
	a = Dxy / den;
	b = (lambda - Dxx) / den;
	c = -a * x_mean - b * y_mean;

	return true;
}

void LineFitTest(CLine& Line, int nCount)
{
	vector<CPnt> Points;

	for (int i = 0; i < nCount; i++)
	{
		float fPercentage = (rand() % 100) / 100.0f;
		float fLen = Line.Length();
		CPnt pt = Line.TrajFun(fPercentage * fLen);

		int nAngle = rand() % 360;
		CAngle ang(nAngle, IN_DEGREE);

		fLen = (rand() % 100) / 300.0f;

		CLine Line1(pt, ang, fLen);

		Points.push_back(Line1.m_ptEnd);
	}

	double a, b, c;
	lineFit(Points, a, b, c);
}
