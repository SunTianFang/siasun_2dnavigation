#ifndef __Jacob_Eigen
#define __Jacob_Eigen

#include <math.h>
#include <map>
#include "ME_Matrix.h"

using namespace std;

namespace MiniEigen
{
template <typename Type, int nDim>
class SelfAdjointEigenSolver
{
  public:
    typedef Matrix<Type, nDim, nDim> MatrixType;
    typedef Matrix<Type, nDim, 1> VectorType;

  private:
    MatrixType eigenVectors_;
    VectorType eigenValues_;

  private:
    bool JacbiCor(const MiniEigen::Matrix<Type, nDim, nDim> &m, MiniEigen::Matrix<Type, nDim, nDim> &eigenVects,
                  MiniEigen::Matrix<Type, nDim, 1> &eigenValues, Type dEps, int nMaxCycles)
    {
        MiniEigen::Matrix<Type, nDim, nDim> matrix(m);

        // 将特征向量矩阵置为单位矩阵
        eigenVects.setIdentity();

        // 迭代
        bool bOk = false;
        for (int nCycles = 0; nCycles < nMaxCycles; nCycles++)
        {
            //在matrix的非对角线上找到最大元素
            double dbMax = matrix[0][1];
            int nRow = 0;
            int nCol = 1;
            for (int i = 0; i < nDim; i++)    //行
            {
                for (int j = 0; j < nDim; j++)    //列
                {
                    double d = fabs(matrix[i][j]);

                    if ((i != j) && (d > dbMax))
                    {
                        dbMax = d;
                        nRow = i;
                        nCol = j;
                    }
                }
            }

            //精度符合要求
            if (dbMax < dEps)
            {
                bOk = true;
                break;
            }

            double dApp = matrix[nRow][nRow];
            double dApq = matrix[nRow][nCol];
            double dAqq = matrix[nCol][nCol];

            //计算旋转角度
            double dAngle = 0.5 * atan2(-2 * dApq, dAqq - dApp);
            double dSinTheta = sin(dAngle);
            double dCosTheta = cos(dAngle);
            double dSin2Theta = sin(2 * dAngle);
            double dCos2Theta = cos(2 * dAngle);

            matrix[nRow][nRow] =
                dApp * dCosTheta * dCosTheta + dAqq * dSinTheta * dSinTheta + 2 * dApq * dCosTheta * dSinTheta;

            matrix[nCol][nCol] =
                dApp * dSinTheta * dSinTheta + dAqq * dCosTheta * dCosTheta - 2 * dApq * dCosTheta * dSinTheta;

            matrix[nRow][nCol] = 0.5 * (dAqq - dApp) * dSin2Theta + dApq * dCos2Theta;

            matrix[nCol][nRow] = matrix[nRow][nCol];

            for (int i = 0; i < nDim; i++)
            {
                if ((i != nCol) && (i != nRow))
                {
                    dbMax = matrix[i][nRow];
                    matrix[i][nRow] = matrix[i][nCol] * dSinTheta + dbMax * dCosTheta;
                    matrix[i][nCol] = matrix[i][nCol] * dCosTheta - dbMax * dSinTheta;
                }
            }

            for (int j = 0; j < nDim; j++)
            {
                if ((j != nCol) && (j != nRow))
                {
                    dbMax = matrix[nRow][j];
                    matrix[nRow][j] = matrix[nCol][j] * dSinTheta + dbMax * dCosTheta;
                    matrix[nCol][j] = matrix[nCol][j] * dCosTheta - dbMax * dSinTheta;
                }
            }

            //计算特征向量
            for (int i = 0; i < nDim; i++)
            {
                dbMax = eigenVects[i][nRow];
                eigenVects[i][nRow] = eigenVects[i][nCol] * dSinTheta + dbMax * dCosTheta;
                eigenVects[i][nCol] = eigenVects[i][nCol] * dCosTheta - dbMax * dSinTheta;
            }
        }

        // 如果迭代次数超过限制，返回false
        if (!bOk)
            return false;

        // 对特征值进行排序以及又一次排列特征向量,特征值即matrix主对角线上的元素
        std::map<int, double> mapEigen;
        for (int i = 0; i < nDim; i++)
        {
            eigenValues(i) = matrix[i][i];
            mapEigen.insert(make_pair(i, eigenValues(i)));
        }

        MiniEigen::Matrix<Type, nDim, nDim> tmp;

        std::map<int, double>::reverse_iterator iter = mapEigen.rbegin();
        for (int j = 0; iter != mapEigen.rend(), j < nDim; ++iter, ++j)
        {
            for (int i = 0; i < nDim; i++)
            {
                int k = iter->first;
                tmp[i][j] = eigenVects[i][k];
            }

            //特征值又一次排列
            eigenValues(j) = iter->second;
        }

        // 设定正负号
        for (int i = 0; i < nDim; i++)
        {
            double dSumVec = 0;
            for (int j = 0; j < nDim; j++)
                dSumVec += tmp[j][i];

            if (dSumVec < 0)
            {
                for (int j = 0; j < nDim; j++)
                    tmp[j][i] *= -1;
            }
        }

        eigenVects = tmp;

        return true;
    }

  public:
    SelfAdjointEigenSolver(const MatrixType &m) { JacbiCor(m, eigenVectors_, eigenValues_, 1E-6, 100); }

    // 取得特征值向量
    VectorType eigenvalues() { return eigenValues_; }

    // 取得特征向量矩阵
    MatrixType eigenvectors() { return eigenVectors_; }
};
}    // namespace MiniEigen
#endif
