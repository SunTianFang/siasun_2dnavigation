#include <stdafx.h>
#include "Combination.h"

///////////////////////////////////////////////////////////////////////////////

//
//   计算阶乘(n!)。
//
int Factorial(int n)
{
	int m = 1;
	for (int i = 1; i <= n; i++)
		m *= i;

	return m;
}

//
//   计算组合数C(n, k)
//
int Combination(int n, int k)
{
	return Factorial(n) / (Factorial(k) * Factorial(n - k));
}
