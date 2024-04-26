#ifndef __MINI_EIGEN_VECTOR
#define __MINI_EIGEN_VECTOR

#include "ME_Matrix.h"

///////////////////////////////////////////////////////////////////////////////
namespace MiniEigen
{
// 定义常用向量类型
#define MINI_EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Row)   \
typedef Matrix<Type, Row, 1> Vector##Row##TypeSuffix; 

#define MINI_EIGEN_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
MINI_EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 2) \
MINI_EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 3) \
MINI_EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 4)

MINI_EIGEN_MAKE_TYPEDEFS_ALL_SIZES(int, i)
MINI_EIGEN_MAKE_TYPEDEFS_ALL_SIZES(float, f)
MINI_EIGEN_MAKE_TYPEDEFS_ALL_SIZES(double, d)

#if 0
// 定义不定维向量
#define MINI_EIGEN_MAKE_TYPEDEFS_SIZE_1(Type, TypeSuffix) typedef Matrix<Type, 0, 1> Vector##X##TypeSuffix;

MINI_EIGEN_MAKE_TYPEDEFS_SIZE_1(int, i)
MINI_EIGEN_MAKE_TYPEDEFS_SIZE_1(float, f)
MINI_EIGEN_MAKE_TYPEDEFS_SIZE_1(double, d)
#endif
}
#endif
