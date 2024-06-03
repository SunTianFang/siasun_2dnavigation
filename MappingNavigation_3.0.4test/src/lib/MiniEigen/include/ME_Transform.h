#pragma once
#include "ME_Matrix.h"

using namespace std;

namespace MiniEigen
{
enum {Affine};
enum {RowMajor, ColMajor};

template <typename Type, int Row = 3, int mode = Affine, int options = ColMajor>
class Transform : public Matrix<Type, Row + 1, Row + 1>
{
  public:
    Transform() : x_(0), y_(0), th_(0) { this->setIdentity(); }

    Transform(Type x, Type y, Type th) : x_(x), y_(y), th_(th)
    {
        this->setIdentity();
        this->at(0, 0) = cos(th_);
        this->at(0, 1) = -sin(th_);
        this->at(1, 0) = sin(th_);
        this->at(1, 1) = cos(th_);
        this->at(0, 3) = x_;
        this->at(1, 3) = y_;
    }

    Transform operator*(const Transform &another)
    {
        if (this->cols_ != another.rows_)
            assert(false);

        Transform r;

        for (int i = 0; i < r.rows_; i++)
            for (int j = 0; j < r.cols_; j++)
            {
                Type val = 0;
                for (int k = 0; k < this->cols_; k++)
                    val += this->at(i, k) * another.at(k, j);

                r.at(i, j) = val;
            }

        r.x_ = r.at(0, 3);
        r.y_ = r.at(1, 3);
        r.th_ = (Type)atan2((double)r.at(1, 0), (double)r.at(0, 0));

        return r;
    }

#if 1
    Matrix<Type, Row, 1> operator*(const Matrix<Type, Row, 1> &v)
    {
        if (this->cols_ != v.rows_ + 1 || v.cols_ != 1)
            assert(false);

        // 先生成齐次的向量
        Matrix<Type, Row + 1, 1> v1;
        for (int i = 0; i < Row; i++)
            v1(i) = v(i);
        v1(Row) = 1;

        Matrix<Type, Row + 1, 1> r1 = this->matrix() * v1;

        return r1.head(Row);
    }
#endif

    Transform inverse()
    {
        Matrix<Type, Row + 1, Row + 1> tmpi = this->matrix().inverse();

        Type x = tmpi.at(0, 3);
        Type y = tmpi.at(1, 3);
        Type th = (Type)atan2(tmpi.at(1, 0), tmpi.at(0, 0));
        Transform r(x, y, th);

        return r;
    }

    Matrix<Type, Row + 1, Row + 1> rotation() { return this->block(0, 0, 3, 3); }

    Matrix<Type, Row + 1, Row + 1> translation() { return this->block(0, 3, 3, 1); }

    Matrix<Type, Row + 1, Row + 1> matrix() { return this->block(0, 0, Row + 1, Row + 1); }

    Transform &rotate(Matrix<Type, 3, 3> m)
    {
        Type th = (Type)atan2((double)m.at(1, 0), (double)m.at(0, 0));
        setAngle(th);
        return *this;
    }

    void setAngle(Type th)
    {
        th_ = th;

        this->at(0, 0) = cos(th_);
        this->at(0, 1) = -sin(th_);
        this->at(1, 0) = sin(th_);
        this->at(1, 1) = cos(th_);
    }

    void setPoint(Type x, Type y)
    {
        x_ = x;
        y_ = y;

        this->at(0, 3) = x_;
        this->at(1, 3) = y_;
    }

    inline Type getx() const { return x_; }

    inline Type gety() const { return y_; }

    inline Type getth() const { return th_; }

    template <typename Type1>
    Transform<Type1> cast()
    {
        Transform<Type1> r;
        for (int i = 0; i < this->rows_ * this->cols_; i++)
            r.data_[i] = (Type1)this->data_[i];

        r.setPoint((Type1)x_, (Type1)y_);
        r.setAngle((Type1)th_);

        return r;
    }

  private:
    Type x_, y_, th_;
};

typedef Transform<double, 3, Affine, ColMajor> Affine3d;
}    // namespace MiniEigen
