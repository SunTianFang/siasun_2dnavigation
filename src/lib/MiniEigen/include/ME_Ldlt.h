#ifndef __LDLT
#define __LDLT

#include "ME_Matrix.h"
#include "ME_Vector.h"

namespace MiniEigen
{
	template<typename Type, int nDim>
	class LDLT
	{
	private:
		Matrix<Type, nDim, nDim> A, D, L;

	public:
		LDLT(const Matrix<Type, nDim, nDim>& m)
		{
			A = m;
			for (int k = 0; k < nDim; k++)
			{
				for (int i = 0; i < k; i++)
					A[k][k] -= A[i][i] * A[k][i] * A[k][i];

				for (int j = k + 1; j < nDim; j++)
				{
					for (int i = 0; i < k; i++)
						A[j][k] -= A[j][i] * A[i][i] * A[k][i];
					A[j][k] /= A[k][k];
				}
			}

			D.setZero();
			for (int i = 0; i < nDim; i++)
				D[i][i] = A[i][i];

			// L复制A的下三角矩阵(对角线置为1)
			L.setIdentity();
			for (int i = 0; i < nDim; i++)
			{
				for (int j = 0; j < i; j++)
					L[i][j] = A[i][j];
			}
		}

		// 针对给定的右侧向量(rhs)，用LDLT法进行求解
		Matrix<Type, nDim, 1> solve(const Matrix<Type, nDim, 1>& rhs)
		{
			Matrix<Type, nDim, 1> X(rhs);

			// LY = B  => Y
			for (int k = 0; k < nDim; k++)
			{
				for (int i = 0; i < k; i++)
					X(k) -= X(i) * L[k][i];
				X(k) /= L[k][k];
			}

			// DL^TX = Y => X
			L = D * L.transpose();

			for (int k = nDim - 1; k >= 0; k--)
			{
				for (int i = k + 1; i < nDim; i++)
					X(k) -= X(i) * L[k][i];
				X(k) /= L[k][k];
			}

			return X;
		}
	};
}
#endif
