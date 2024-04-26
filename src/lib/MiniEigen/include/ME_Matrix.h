#ifndef __MINI_EIGEN_MATRIX
#define __MINI_EIGEN_MATRIX

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <assert.h>

#define LUDECOMPOSE
//#define GAUSS_ELIMINATION_WITH_SIMPLE_PIVOTING        // 此方法目前还有问题，未调试通过

using namespace std;

#ifndef NEAR_ZERO
#define NEAR_ZERO(x) (abs(x) < 1.0e-16)
#endif

///////////////////////////////////////////////////////////////////////////////
namespace MiniEigen
{
// error codes
enum { MATERR_MEM = 1, MATERR_NOMATRIX, MATERR_NOSQUAREMATRIX, MATERR_SIZE, MATERR_SINGULAR };

// 定义通用矩阵类型
template <typename Type, int Row, int Col>
class Matrix
{
  private:
    int assignIdx;    // 使用"<<"及","操作符进行矩阵元素赋值时的地址索引

  public:
    Type *data_;    // 数据缓冲区

  public:
    int rows_, cols_;    // 行数和列数

  private:
    // 计算方阵关于某一元素(i, j)的代数余子式
    Type SubDeterminante(int n, int *deli, int *delj) const
    {
        int deln = rows_ - n;

        if (n < 1)    // end of recursion
            return 1.0;

        //
        // Recursive part.
        // First thing is to determine a free row for the
        // determinante evaluation.

        int i;
        for (i = 0; i < rows_; i++)
            if (!finditem(i, deli, deln))
                break;

        // Now loop through the row.
        Type sum = 0.0, sign = 1.0;
        for (int j = 0; j < rows_; j++)
        {
            if (!finditem(j, delj, deln))
            {
                deli[deln] = i;
                delj[deln] = j;
                sum += sign * at(i, j) * SubDeterminante(n - 1, deli, delj);
                sign = -sign;
            }
        }

        return sum;
    }

    static bool finditem(int item, int *list, int size)
    {
        for (int i = 0; i < size; i++, list++)
            if (*list == item)
                return true;

        return false;
    }

  public:
    // 构造函数
    Matrix(int _rows, int _cols)
    {
        assignIdx = -1;

        rows_ = _rows;
        cols_ = _cols;

        data_ = new Type[rows_ * cols_];
        if (data_ == NULL)
            assert(false);

        setZero();
    }

    // 仅适用于Vector3x向量的构造函数
    Matrix(Type d1, Type d2, Type d3)
    {
        if (Row != 3 || Col != 1)
            assert(false);

        assignIdx = -1;

        rows_ = Row;
        cols_ = Col;

        data_ = new Type[rows_ * cols_];
        if (data_ == NULL)
            assert(false);

        setZero();

        data_[0] = d1;
        data_[1] = d2;
        data_[2] = d3;
    }

    // Copy构造函数
    Matrix(const Matrix &another)
    {
        assignIdx = -1;

        rows_ = another.rows_;
        cols_ = another.cols_;

        data_ = new Type[rows_ * cols_];
        if (data_ == NULL)
            assert(false);

        for (int i = 0; i < rows_ * cols_; i++)
            data_[i] = another.data_[i];
    }

    // Copy构造函数
    template <int Row2, int Col2>
    Matrix(const Matrix<Type, Row2, Col2> &another)
    {
        assignIdx = -1;

        rows_ = another.rows_;
        cols_ = another.cols_;

        data_ = new Type[rows_ * cols_];
        if (data_ == NULL)
            assert(false);

        for (int i = 0; i < rows_ * cols_; i++)
            data_[i] = another.data_[i];
    }

    // 缺省构造函数
    Matrix()
    {
        assignIdx = -1;
        rows_ = Row;
        cols_ = Col;

        // 如果矩阵维度为0，则暂不分配空间
        if (Row == 0 || Col == 0)
            data_ = NULL;

        // 矩阵维度非0
        else
        {
            data_ = new Type[rows_ * cols_];
            if (data_ == NULL)
                assert(false);

            setZero();
        }
    }

    ~Matrix()
    {
        if (data_ != NULL)
        {
            delete[] data_;
            data_ = NULL;
        }
    }

    // 调整矩阵的维度
    void resize(int _rows, int _cols)
    {
        // 如果维度不变，则直接返回(数据未变)
        if (rows_ == _rows && cols_ == _cols)
            return;

        // 其它情况，均释放原有空间，原有数据丢失
        if (data_ != NULL)
            delete[] data_;

        assignIdx = -1;

        rows_ = _rows;
        cols_ = _cols;

        data_ = new Type[rows_ * cols_];
        if (data_ == NULL)
            assert(false);
    }

    void setZero()
    {
        for (int i = 0; i < rows_ * cols_; i++)
            data_[i] = 0;
    }

    void setIdentity()
    {
        setZero();

        for (int i = 0; i < rows_; i++)
            at(i, i) = 1;
    }

    void setOnes()
    {
        for (int i = 0; i < rows_ * cols_; i++)
            data_[i] = 1;
    }

    int rows() const { return rows_; }
    int cols() const { return cols_; }

    // 取指定元素
    Type &at(int i, int j) const { return data_[i * cols_ + j]; }

    // 取指定元素
    Type coeff(int i, int j) const { return data_[i * cols_ + j]; }

    // 取指定元素的引用
    Type &coeffRef(int i, int j) const { return data_[i * cols_ + j]; }

    // 返回1矩阵
    Matrix Identity()
    {
        if (rows_ != cols_)
            assert(false);

        Matrix r(rows_, rows_);
        for (int i = 0; i < rows_; i++)
            r[i][i] = 1;

        return r;
    }

    static Matrix Identity(int r, int c)
    {
        if (r != c)
            assert(false);

        Matrix m(r, r);
        m.setIdentity();
        return m;
    }

    Matrix Zero()
    {
        Matrix r(rows_, cols_);
        for (int i = 0; i < rows_ * cols_; i++)
            r[i][i] = 0;

        return r;
    }

    //	重载"[]"操作符
    inline Type *operator[](int index) const { return &data_[cols_ * index]; }

    // 取得矩阵的第i行作为行子矩阵返回
    Matrix row(int index) const
    {
        Matrix r(1, cols_);
        for (int i = 0; i < cols_; i++)
            r[0][i] = at(index, i);

        return r;
    }

    // 取得矩阵的第i列作为列子矩阵返回
    Matrix col(int index) const
    {
        Matrix r(rows_, 1);
        for (int i = 0; i < rows_; i++)
            r[i][0] = at(i, index);

        return r;
    }

    // 将矩阵某一列向量设置为指定的矩阵
    void setCol(int index, const Matrix &another)
    {
        for (int i = 0; i < rows_; i++)
            at(i, index) = another[i][0];
    }

    // 重载"()"操作符，存取指定元素
    Type &operator()(int i, int j) const { return data_[i * cols_ + j]; }

    // 重载"="操作符
    Matrix &operator=(const Matrix &another)
    {
        // 如果与新矩阵维度不同，则需要重新分配空间
        if (rows_ != another.rows_ || cols_ != another.cols_)
        {
            if (data_ != NULL)
                delete[] data_;

            rows_ = another.rows_;
            cols_ = another.cols_;

            data_ = new Type[rows_ * cols_];
            if (data_ == NULL)
                assert(false);
        }

        // 复制矩阵数据
        for (int i = 0; i < rows_ * cols_; i++)
            data_[i] = another.data_[i];

        return *this;
    }

    // 重载"="操作符
    template <int Row2, int Col2>
    Matrix &operator=(const Matrix<Type, Row2, Col2> &another)
    {
        // 如果与新矩阵维度不同，则需要重新分配空间
        //			if (rows_ != another.rows_ || cols_ != another.cols_)

        // 如果与新矩阵容量不同，则需要重新分配空间(相同容量时，可以按空间顺序直接复制!)
        if (rows_ * cols_ != another.rows_ * another.cols_)
        {
            if (data_ != NULL)
                delete[] data_;

            rows_ = another.rows_;
            cols_ = another.cols_;

            data_ = new Type[rows_ * cols_];
            if (data_ == NULL)
                assert(false);
        }

        // 复制矩阵数据
        for (int i = 0; i < rows_ * cols_; i++)
            data_[i] = another.data_[i];

        return *this;
    }

    // 重载"+="操作符
    template <int Row2, int Col2>
    Matrix &operator+=(const Matrix<Type, Row2, Col2> &another)
    {
        // 矩阵加法操作要求两个矩阵维度相同
        if (rows_ != another.rows_ || cols_ != another.cols_)
            assert(false);

        for (int i = 0; i < rows_ * cols_; i++)
            data_[i] += another.data_[i];

        return *this;
    }

    // 重载"-="操作符
    template <int Row2, int Col2>
    Matrix &operator-=(const Matrix<Type, Row2, Col2> &another)
    {
        // 矩阵减法操作要求两个矩阵维度相同
        if (rows_ != another.rows_ || cols_ != another.cols_)
            assert(false);

        for (int i = 0; i < rows_ * cols_; i++)
            data_[i] -= another.data_[i];

        return *this;
    }

    // 重载"*="操作符
    template <int Row2, int Col2>
    Matrix &operator*=(const Matrix<Type, Row2, Col2> &another)
    {
        // 矩阵乘法操作要求第一个矩阵列数与第二个矩阵行数相同
        if (cols_ != another.rows_)
            assert(false);

        *this = *this * another;
        return *this;
    }

    // 重载"*="操作符
    template <typename Type1>
    Matrix &operator*=(Type1 n)
    {
        for (int i = 0; i < rows_; i++)
            for (int j = 0; j < cols_; j++)
                at(i, j) *= n;

        return *this;
    }

    // 重载"/="操作符
    template <typename Type1>
    Matrix &operator/=(Type1 n)
    {
        for (int i = 0; i < rows_; i++)
            for (int j = 0; j < cols_; j++)
                at(i, j) /= n;

        return *this;
    }

    // 重载"<<"操作符
    Matrix &operator<<(Type e)
    {
        assignIdx = 0;
        data_[assignIdx++] = e;

        return *this;
    }

    // 重载","操作符
    Matrix &operator,(Type e)
    {
        // ","操作符仅能在"<<"操作符之后出现，且总数量不能超过矩阵元素数量
        if (assignIdx <= 0)
            assert(false);
        else if (assignIdx < rows_ * cols_)
            data_[assignIdx++] = e;

        return *this;
    }

    // 重载"+"操作符
    template <int Row2, int Col2>
    Matrix operator+(const Matrix<Type, Row2, Col2> &another) const
    {
        Matrix r(*this);
        r += another;
        return r;
    }

    // 重载"-"操作符
    template <int Row2, int Col2>
    Matrix operator-(const Matrix<Type, Row2, Col2> &another) const
    {
        Matrix r(*this);
        r -= another;
        return r;
    }

    // 重载"-"操作符，作为取负号操作
    Matrix operator-() const
    {
        Matrix r(*this);
        for (int i = 0; i < rows_ * cols_; i++)
            r.data_[i] = -r.data_[i];

        return r;
    }

    // 重载"*"操作符
    template <int Row2, int Col2>
    Matrix<Type, Row, Col2> operator*(const Matrix<Type, Row2, Col2> &another) const
    {
        // 矩阵乘法须核对维度
        if (cols_ != another.rows_)
            assert(false);

        // 定义结果矩阵
        Matrix<Type, Row, Col2> r(rows_, another.cols_);

        for (int i = 0; i < r.rows_; i++)
            for (int j = 0; j < r.cols_; j++)
            {
                Type val = 0;
                for (int k = 0; k < cols_; k++)
                    val += at(i, k) * another[k][j];
                r[i][j] = val;
            }

        return r;
    }

    // 重载"*"操作符，定义矩阵的数乘
    template <typename Type1>
    Matrix operator*(Type1 n) const
    {
        // 定义结果矩阵
        Matrix r(*this);

        for (int i = 0; i < r.rows_; i++)
            for (int j = 0; j < r.cols_; j++)
                r[i][j] *= n;

        return r;
    }

    // 重载"/"操作符，定义矩阵的数除
    template <typename Type1>
    Matrix operator/(Type1 n) const
    {
        // 定义结果矩阵
        Matrix r(*this);

        for (int i = 0; i < r.rows_; i++)
            for (int j = 0; j < r.cols_; j++)
                r[i][j] /= n;

        return r;
    }

    // 如果矩阵为1x1矩阵，可将期其唯一元素作为数据直接返回
    operator Type()
    {
        if (rows_ != 1 || cols_ != 1)
            assert(false);

        return this->data_[0];
    }

    // 计算转置矩阵
    Matrix transpose() const
    {
        Matrix r(cols_, rows_);

        for (int i = 0; i < cols_; i++)
            for (int j = 0; j < rows_; j++)
                r[i][j] = at(j, i);

        return r;
    }

    // 返回矩阵最小元素
    Type minCoeff(int *idx = NULL) const
    {
        Type ret = data_[0];
        if (idx != NULL)
            *idx = 0;

        for (int i = 1; i < rows_ * cols_; i++)
        {
            if (data_[i] < ret)
            {
                ret = data_[i];
                if (idx != NULL)
                    *idx = i;
            }
        }

        return ret;
    }

    // 返回矩阵最大元素
    Type maxCoeff(int *idx = NULL)
    {
        Type ret = data_[0];
        if (idx != NULL)
            *idx = 0;

        for (int i = 1; i < rows_ * cols_; i++)
        {
            if (data_[i] > ret)
            {
                ret = data_[i];
                if (idx != NULL)
                    *idx = i;
            }
        }

        return ret;
    }

    // 计算矩阵所有元素的和
    Type sum() const
    {
        Type ret = 0;
        for (int i = 0; i < rows_ * cols_; i++)
            ret += data_[i];

        return ret;
    }

    // 按行计算矩阵元素和，返回一个列向量
    Matrix rowWiseSum() const
    {
        Matrix r(rows_, 1);

        for (int i = 0; i < rows_; i++)
        {
            r[i][0] = 0;
            for (int j = 0; j < cols_; j++)
                r[i][0] += at(i, j);
        }

        return r;
    }

    // 按列计算矩阵元素和，返回一个行向量
    Matrix colWiseSum() const
    {
        Matrix r(1, cols_);

        for (int i = 0; i < cols_; i++)
        {
            r[0][i] = 0;
            for (int j = 0; j < rows_; j++)
                r[0][i] += at(j, i);
        }

        return r;
    }

    // 对矩阵进行强制类型转换
    template <typename Type1>
    Matrix<Type1, Row, Col> cast()
    {
        Matrix<Type1, Row, Col> r;
        for (int i = 0; i < rows_ * cols_; i++)
            r.data_[i] = (Type1)data_[i];

        return r;
    }

    // 将矩阵的第i, j行对换
    void SwapRaw(int i, int j)
    {
        for (int k = 0; k < cols_; k++)
        {
            Type tmp = at(i, k);
            at(i, k) = at(j, k);
            at(j, k) = tmp;
        }
    }

#ifdef LUDECOMPOSE

    // 对矩阵进行LU分解
    int lud(int *indx, Type *d)
    {
        int imax;
        Type dum;

        // vv stores the implicit scaling of each row
        Type *vv = new Type[rows_];
        if (vv == NULL)
            return MATERR_MEM;

        *d = 1.0;    // no row interchanges yet

        // Loop over rows_ to get the implicit scaling information
        int i;
        for (i = 0; i < rows_; i++)
        {
            Type big = 0.0;
            for (int j = 0; j < rows_; j++)
            {
                Type temp = fabs(at(i, j));
                if (temp > big)
                    big = temp;
            }

            if (big == 0.0)
            {
                delete[] vv;
                return MATERR_SINGULAR;
            }

            vv[i] = 1.0 / big;    // save the scaling.
        }

        // loop over columns
        for (int j = 0; j < rows_; j++)
        {
            for (i = 0; i < j; i++)
            {
                Type sum = at(i, j);
                for (int k = 0; k < i; k++)
                    sum -= at(i, k) * at(k, j);
                at(i, j) = sum;
            }

            // search for largest pivot element
            Type big = 0.0;
            imax = j;
            for (i = j; i < rows_; i++)
            {
                Type sum = at(i, j);
                for (int k = 0; k < j; k++)
                    sum -= at(i, k) * at(k, j);
                at(i, j) = sum;
                if ((dum = vv[i] * fabs(sum)) >= big)
                {
                    // figure of merit for pivot is better than the best so far
                    big = dum;
                    imax = i;
                }
            }

            // do we need to interchange rows_?
            if (j != imax)
            {
                for (int k = 0; k < rows_; k++)
                {
                    dum = at(imax, k);
                    at(imax, k) = at(j, k);
                    at(j, k) = dum;
                }
                *d = -(*d);          // change the parity of d.
                vv[imax] = vv[j];    // also interchange the scale factor.
            }

            indx[j] = imax;
            if (at(j, j) == 0.0)
            {
                delete[] vv;
                return (MATERR_SINGULAR);
            }

            if (j != rows_ - 1)
            {
                dum = 1.0 / at(j, j);
                for (i = j + 1; i < rows_; i++)
                    at(i, j) *= dum;
            }
        }

        delete[] vv;
        return 0;
    }

    void lubksb(int *indx, Type *b)
    {
        int ii = -1;
        for (int i = 0; i < rows_; i++)
        {
            int ip = indx[i];

            Type sum = b[ip];
            b[ip] = b[i];

            if (ii >= 0)
                for (int j = ii; j <= i - 1; j++)
                    sum -= at(i, j) * b[j];
            else if (sum)
                ii = i;

            b[i] = sum;
        }

        for (int i = rows_ - 1; i >= 0; i--)
        {
            Type sum = b[i];
            for (int j = i + 1; j < rows_; j++)
                sum -= at(i, j) * b[j];
            b[i] = sum / at(i, i);
        }
    }

    // 计算矩阵的逆阵
    int inverse(Matrix *res)
    {
        int n, rc = 0;
        Type d;
        int i, j;

        if (res == NULL)
            return MATERR_NOMATRIX;

        if (rows_ != this->cols_ || res->rows_ != res->cols_)
            return MATERR_NOSQUAREMATRIX;

        if (rows_ != res->rows_)
            return MATERR_SIZE;

        Matrix *copy = new Matrix(rows_, rows_);
        Type *colData = new Type[rows_];
        int *indx = new int[rows_];

        if (copy == NULL || indx == NULL)
            rc = MATERR_MEM;

        if (rc == 0)
        {
            copy->setZero();
            *copy += *this;

            rc = copy->lud(indx, &d);
        }

        if (rc == 0)
        {
            for (j = 0; j < rows_; j++)
            {
                for (i = 0; i < rows_; i++)
                    colData[i] = 0.0;
                colData[j] = 1.0;
                copy->lubksb(indx, colData);

                for (i = 0; i < rows_; i++)
                    res->at(i, j) = colData[i];
            }
        }

        if (indx != NULL)
            delete[] indx;

        if (colData != NULL)
            delete[] colData;

        delete copy;

        return rc;
    }

    // 计算矩阵的逆阵 - 标准形式
    Matrix inverse()
    {
        Matrix r(rows_, cols_);
        if (inverse(&r) != 0)
            assert(false);

        return r;
    }

#elif defined GAUSS_ELIMINATION_WITH_SIMPLE_PIVOTING

    //
    //   计算矩阵的逆阵。
    //
    int inverse(Matrix *res)
    {
        int n, i, j, k, pivot;
        Type fac, pivot_val;

        if (res == NULL)
            return MATERR_NOMATRIX;

        if (rows_ != cols_ || res->rows_ != res->cols_)
            return MATERR_NOSQUAREMATRIX;

        if (rows_ != res->rows_)
            return MATERR_SIZE;

        res->setOne();
        Matrix *copy = new Matrix(*this);

        //				  /1 * *\
			// bring matrix on  |0 1 *| form
        //				  \0 0 1/

        for (j = 0; j < rows_; j++)    // loop through cols_
        {
            // find row with maximum value in col j (pivot)
            pivot_val = 0.0;
            for (i = j; i < rows_; i++)
                if (fabs(copy->at(i, j)) > pivot_val)
                {
                    pivot_val = fabs(copy->at(i, j));
                    pivot = i;
                }

            if (NEAR_ZERO(pivot_val))
            {
                delete copy;
                return MATERR_SINGULAR;    // can't calculate inverse matrix
            }

            if (pivot != j)
            {
                // exchange rows_
                copy->SwapRaw(j, pivot);
                res->SwapRaw(j, pivot);
            }

            // Divide row by pivot element
            fac = 1.0 / copy->at(j, j);
            for (k = 0; k < rows_; k++)
            {
                copy->at(j, k) *= fac;
                res->at(j, k) *= fac;
            }

            //
            // no we can zero the j-th element of all rows_ below row j
            // by basic row operations.

            for (i = j + 1; i < rows_; i++)
            {
                if (copy->at(i, j) == 0.0)
                    continue;    // already zero

                fac = copy->at(i, j);
                for (k = 0; k < rows_; k++)
                {
                    copy->at(i, k) -= copy->at(j, k) * fac;
                    res->at(i, k) -= res->at(j, k) * fac;
                }
            }
        }

        //                  /1 0 0\
			 // bring matrix on  |0 1 0| form
        //                  \0 0 1/

        for (j = rows_ - 1; j >= 0; j--)    // loop through cols_
        {
            // check col j
            if (NEAR_ZERO(copy->at(j, j)))
            {
                delete copy;
                return MATERR_SINGULAR;    // can't calculate inverse matrix
            }

            //
            // no we can zero the j-th element of all rows_ above row j
            // by basic row operations.

            for (i = j - 1; i >= 0; i--)
            {
                if (copy->at(i, j) == 0.0)
                    continue;    // already zero

                fac = copy->at(i, j);
                for (k = 0; k < rows_; k++)
                {
                    copy->at(i, k) -= copy->at(j, k) * fac;
                    res->at(i, k) -= res->at(j, k) * fac;
                }
            }
        }

        delete copy;

        return 0;
    }

#else

    //
    //   计算矩阵的逆阵。
    //
    int inverse(Matrix *res)
    {
        // This routine calculates a dim x dim inverse matrix.  It uses Gaussian
            * elimination on the system [matrix][inverse] = [identity].  The values
			* of the matrix then become the coefficients of the system of equations
			* that evolves from this equation.  The system is then solved for the
			* values of [inverse].  The algorithm solves for each column of [inverse]
			* in turn.  Partial pivoting is also done to reduce the numerical error
			* involved in the computations.  If singularity is detected, the routine
			* ends and returns an error.
			*
			* (See Numerical Analysis, L.W. Johnson and R.D.Riess, 1982).
			

			int ipivot = 0, h, i, j, k;
            Type pivot, q;
            int dim;

            if (res == NULL)
                return MATERR_NOMATRIX;

            if (rows_ != cols_ || res->rows_ != res->cols_)
                return MATERR_NOSQUAREMATRIX;

            if (rows_ != res->rows_)
                return MATERR_SIZE;

            dim = rows_;

            Matrix *aug = new Matrix(dim, dim + 1);
            if (aug == NULL)
                return MATERR_MEM;

            // solve column by column
            for (h = 0; h < dim; h++)
            {
                // Set up the augmented matrix for [matrix][inverse] = [identity]
                for (i = 0; i < dim; i++)
                {
                    memcpy((*aug)[i], (*this)[i], dim * sizeof(Type));
                    aug->at(i, dim) = (h == i) ? 1.0 : 0.0;
                }

                //
                * Search for the largest entry in column i, rows_ i through dim-1.
				* ipivot is the row index of the largest entry.
				

				for (i = 0; i < dim - 1; i++)
                {
                    pivot = 0.0;

                    for (j = i; j < dim; j++)
                    {
                        Type temp;

                        temp = ABS(aug->at(j, i));
                        if (pivot < temp)
                        {
                            pivot = temp;
                            ipivot = j;
                        }
                    }
                    if (NEAR_ZERO(pivot))    // singularity check
                    {
                        delete aug;
                        return (MATERR_SINGULAR);
                    }

                    // interchange rows_ i and ipivot

                    if (ipivot != i)
                        aug->SwapRaw(i, ipivot);

                    // put augmented matrix in echelon form

                    for (k = i + 1; k < dim; k++)
                    {
                        q = -aug->at(k, i) / aug->at(i, i);
                        aug->at(k, i) = 0.0;
                        for (j = i + 1; j < dim + 1; j++)
                        {
                            aug->at(k, j) = q * aug->at(i, j) + aug->at(k, j);
                        }
                    }
                }

                if (NEAR_ZERO(aug->at(dim - 1, dim - 1)))    // singularity check
                {
                    delete aug;
                    return MATERR_SINGULAR;
                }

                // backsolve to obtain values for inverse matrix

                res->at(dim - 1, h) = aug->at(dim - 1, dim) / aug->at(dim - 1, dim - 1);

                for (k = 1; k < dim; k++)
                {
                    q = 0.0;
                    for (j = 1; j <= k; j++)
                    {
                        q += aug->at(dim - 1 - k, dim - j) * res->at(dim - j, h);
                    }
                    res->at(dim - 1 - k, h) = (aug->at(dim - 1 - k, dim) - q) / aug->at(dim - 1 - k, dim - 1 - k);
                }
            }

            delete aug;
            return 0;
    }
#endif

    // 计算矩阵的行列式
    Type determinant() const
    {
        Type r = 0;
        int err = 0;

        if (data_ == NULL)
            return MATERR_NOMATRIX;

        if (rows_ != cols_)
            return MATERR_NOSQUAREMATRIX;

        int *deli = new int[rows_];
        int *delj = new int[rows_];

        if (deli != NULL && delj != NULL)
        {
            r = SubDeterminante(rows_, deli, delj);
        }
        else
        {
            err = MATERR_MEM;
            assert(false);
        }

        if (delj != NULL)
            delete[] delj;

        if (deli != NULL)
            delete[] deli;

        return r;
    }

    // 计算方阵的逆阵，同时计算它的行列式(如果存在的话)
    void computeInverseAndDetWithCheck(Matrix &inv, Type &det, bool &exist)
    {
        exist = false;
        int r = inverse(&inv);
        if (r == 0)
        {
            exist = true;
            det = determinant();
        }
    }

    // 读取并返回矩阵的一个子块
    Matrix block(int startRow, int startCol, int sizeRow, int sizeCol) const
    {
        if (sizeRow > rows_ || sizeCol > cols_ || startRow + sizeRow > rows_ || startCol + sizeCol > cols_)
            assert(false);

        Matrix m(sizeRow, sizeCol);

        for (int i = 0; i < sizeRow; i++)
            for (int j = 0; j < sizeCol; j++)
                m[i][j] = at(startRow + i, startCol + j);

        return m;
    }

    // 将矩阵的一个子块赋值为指定的矩阵
    void setBlock(int startRow, int startCol, int sizeRow, int sizeCol, const Matrix &another)
    {
        if (sizeRow > rows_ || sizeCol > cols_ || startRow + sizeRow > rows_ || startCol + sizeCol > cols_)
            assert(false);

        if (sizeRow != another.rows_ || sizeCol != another.cols_)
            assert(false);

        for (int i = 0; i < sizeRow; i++)
            for (int j = 0; j < sizeCol; j++)
                at(startRow + i, startCol + j) = another.at(i, j);
    }

    // 所有矩阵元素取平方根后返回
    Matrix cwiseSqrt()
    {
        Matrix r(*this);
        for (int i = 0; i < rows_ * cols_; i++)
            r.data_[i] = sqrt(data_[i]);

        return r;
    }

    // 所有矩阵元素取绝对值后返回
    Matrix cwiseAbs()
    {
        Matrix r(*this);
        for (int i = 0; i < rows_ * cols_; i++)
            r.data_[i] = abs(data_[i]);

        return r;
    }

    // 取得方阵的对角元素，作为一个向量返回
    Matrix diagonal()
    {
        if (rows_ != cols_)
            assert(false);

        Matrix r(rows_, 1);
        for (int i = 0; i < rows_; i++)
            r(i) = at(i, i);

        return r;
    }

    // 取得矩阵的下三角矩阵
    Matrix lowerTrangularView()
    {
        if (rows_ != cols_)
            assert(false);

        Matrix r(*this);
        for (int i = 0; i < rows_; i++)
            for (int j = i + 1; j < cols_; j++)
                r[i][j] = 0;

        return r;
    }

    // 显示矩阵内容
    void Dump() const
    {
        for (int i = 0; i < rows_; i++)
        {
            for (int j = 0; j < cols_; j++)
            {
                Type x = at(i, j);
                printf("%.6lf  ", x);    // << /*at(i, j)*/ << "   ";
            }
            cout << endl;
        }
        cout << endl;
    }

    // 保存矩阵内容到文件
    void Dump(FILE *fp)
    {
        for (int i = 0; i < rows_; i++)
        {
            for (int j = 0; j < cols_; j++)
                fprintf(fp, "%f  ", (float)(at(i, j)));
            fprintf(fp, "\n");
        }

        fprintf(fp, "\n");
    }

    /////////////////////////////////////////////////////////
    //   以下为Vector独有的函数

    // 取得向量的行数
    int size()
    {
        if (this->cols_ != 1)
            assert(false);

        return this->rows_;
    }

    // 取向量的指定元素
    Type coeff(int i) const
    {
        if (cols_ != 1)
            assert(false);

        return data_[i];
    }

    // 取向量的指定元素的引用
    Type &coeffRef(int i) const
    {
        if (cols_ != 1)
            assert(false);

        return data_[i];
    }

    // 重载"()"操作符，取得向量的某一元素的引用
    Type &operator()(int index) const
    {
        if (this->cols_ != 1)
            assert(false);

        return this->data_[index];
    }

    // 取得向量的前num个元素，作为一个新向量返回
    Matrix head(int num) const
    {
        if (cols_ != 1 || rows_ < num)
            assert(false);

        Matrix r(num, 1);
        for (int i = 0; i < num; i++)
            r[i][0] = at(i, 0);

        return r;
    }

    // 取得向量的后num个元素，作为一个新向量返回
    Matrix tail(int num) const
    {
        if (cols_ != 1 || rows_ < num)
            assert(false);

        Matrix r(num, 1);
        for (int i = 0; i < num; i++)
            r[i][0] = at(rows_ - num + i, 0);

        return r;
    }

    // 计算向量的模
    Type norm() const
    {
        if (this->cols_ != 1)
            assert(false);

        Type ret = 0;
        for (int i = 0; i < this->rows_; i++)
        {
            Type d = this->data_[i];
            ret += d * d;
        }
        ret = sqrt(ret);
        return ret;
    }

    // 计算两个向量的点乘
    Type dot(const Matrix &another)
    {
        if (this->rows_ != another.rows_ || this->cols_ != another.cols_ || this->cols_ != 1)
            assert(false);

        Type ret = 0;
        for (int i = 0; i < this->rows_; i++)
        {
            ret += this->data_[i] * another.at(i, 0);
        }

        return ret;
    }

    // 构造对角矩阵，以该向量元素为矩阵的对角元素
    Matrix<Type, Row, Row> asDiagonal()
    {
        if (this->cols_ != 1)
            assert(false);

        Matrix<Type, Row, Row> r(this->rows_, this->rows_);    // 初始化为0矩阵

        // 设置对角元素
        for (int i = 0; i < this->rows_; i++)
            r[i][i] = this->data_[i];

        return r;
    }
};

#if 0
	// 补充定义常数在前的矩阵数乘
	template<typename Type, typename Type1, int Row, int Col>
	Matrix<Type, Row, Col> operator * (Type1 n, const Matrix<Type, Row, Col>& m)
	{
		Matrix<Type, Row, Col> r(m);
		r *= n;

		return r;
	}
#endif

// 补充定义常数在前的矩阵数乘
template <typename Type, int Row, int Col>
Matrix<Type, Row, Col> operator*(double n, const Matrix<Type, Row, Col> &m)
{
    Matrix<Type, Row, Col> r(m);
    r *= n;

    return r;
}

// 定义常用方阵类型
#define MINI_EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size) typedef Matrix<Type, Size, Size> Matrix##Size##TypeSuffix;

#define MINI_EIGEN_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix)                                                           \
    MINI_EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 2)                                                                      \
    MINI_EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 3)                                                                      \
    MINI_EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 4)                                                                      \
    MINI_EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 5)                                                                      \
    MINI_EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 6)                                                                      \
    MINI_EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 7)                                                                      \
    MINI_EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 8)                                                                      \
    MINI_EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 9)

MINI_EIGEN_MAKE_TYPEDEFS_ALL_SIZES(int, i)
MINI_EIGEN_MAKE_TYPEDEFS_ALL_SIZES(float, f)
MINI_EIGEN_MAKE_TYPEDEFS_ALL_SIZES(double, d)

// 定义不定维矩阵
#define MINI_EIGEN_MAKE_TYPEDEFS_SIZE_0(Type, TypeSuffix) typedef Matrix<Type, 0, 0> Matrix##X##TypeSuffix;

MINI_EIGEN_MAKE_TYPEDEFS_SIZE_0(int, i)
MINI_EIGEN_MAKE_TYPEDEFS_SIZE_0(float, f)
MINI_EIGEN_MAKE_TYPEDEFS_SIZE_0(double, d)

}    // namespace MiniEigen

#endif    // __MINI_EIGEN_MATRIX
