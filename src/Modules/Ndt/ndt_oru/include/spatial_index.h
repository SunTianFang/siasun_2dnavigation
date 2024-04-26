/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, AASS Research Center, Orebro University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AASS Research Center nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef SPATIAL_INDEX_HH
#define SPATIAL_INDEX_HH

#include <vector>
#include <iostream>
#include "ndt_cell_oru.h"

#ifdef NDT1_USE_MINI_EIGEN
#define Eigen MiniEigen
#endif

namespace ndt_oru
{
   template <typename Type>
   class CSize3D
   {
   public:
      Type x;
      Type y;
      Type z;

   public:
      CSize3D(Type _x, Type _y, Type _z)
      {
         x = _x;
         y = _y;
         z = _z;
      }

      CSize3D(const CSize3D& another)
      {
         x = another.x;
         y = another.y;
         z = another.z;
      }

      CSize3D()
      {
         x = y = z = -1;
      }

      void Set(Type _x, Type _y, Type _z)
      {
         x = _x;
         y = _y;
         z = _z;
      }

      CSize3D& operator = (const CSize3D& another)
      {
         x = another.x;
         y = another.y;
         z = another.z;

         return *this;
      }

      CSize3D& operator += (const CSize3D& another)
      {
         x += another.x;
         y += another.y;
         z += another.z;

         return *this;
      }

      CSize3D& operator -= (const CSize3D& another)
      {
         x -= another.x;
         y -= another.y;
         z -= another.z;

         return *this;
      }

      CSize3D operator + (const CSize3D& another) const
      {
         CSize3D tmp(*this);
         tmp += another;

         return tmp;
      }

      CSize3D operator - (const CSize3D& another) const
      {
         CSize3D tmp(*this);
         tmp -= another;

         return tmp;
      }

      CSize3D& operator *= (Type k)
      {
         x *= k;
         y *= k;
         z *= k;

         return *this;
      }
   };

   /** \brief Base class for all spatial indexing structures
       \details
   A SpatialIndex is anything that holds PointInterface pointers
   and organizes them in a manner accessible from outside.
   This class defines what is necessary to be a spatial index - namely
   the ability to find the cell in which a point is placed and to store
   newly observed points. It should also be possible to check the size of
   the occupied space, as well as to get cells neighboring any given cell.
   */
   class SpatialIndex
   {
   protected:
      std::vector<NDTCell*> activeCells_;

   public:
      typedef std::vector<NDTCell*> CellPtrVector;
      typedef typename CellPtrVector::iterator CellVectorItr;
      typedef typename CellPtrVector::reverse_iterator CellVectorRitr;
      typedef typename CellPtrVector::const_iterator CellVectorConstItr;
      typedef typename CellPtrVector::const_reverse_iterator CellVectorConstRitr;

      virtual ~SpatialIndex() {}

      // 取得指定点处所对应的单元
      virtual NDTCell* getCellForPoint(const Eigen::Vector3d & point, bool allocIfNeeded = false) = 0;

      // 向SpatialIndex中加入一个空间点
      virtual NDTCell* addPoint(const Eigen::Vector3d &point) = 0;

      ///iterator through all cells in index, points at the begining
      virtual CellVectorItr begin() { return activeCells_.begin(); }

      virtual CellVectorConstItr begin() const { return activeCells_.begin(); }

      ///iterator through all cells in index, points at the begining
      virtual CellVectorRitr rbegin() { return activeCells_.rbegin(); }

      virtual CellVectorConstRitr rbegin() const { return activeCells_.rbegin(); }

      ///iterator through all cells in index, points at the end
      virtual CellVectorItr end() { return activeCells_.end(); }

      virtual CellVectorConstItr end() const { return activeCells_.end(); }

      ///iterator through all cells in index, points at the end
      virtual CellVectorRitr rend() { return activeCells_.rend(); }

      virtual CellVectorConstRitr rend() const { return activeCells_.rend(); }

      virtual int size() const { return activeCells_.size(); }

      ///clone - create an empty object with same type
      virtual SpatialIndex* clone() const = 0;

      ///copy - create the same object as a new instance
//      virtual SpatialIndex* copy() const = 0;

      // 以下方法，继承类可选择性提供(非必需)

      // 设置空间管理范围的中心
        virtual bool setCenter(const Eigen::Vector3d &pt) { return true; }

      // 设置空间管理范围
        virtual bool setSize(double sx, double sy, double sz) { return true; }

      // 设置单元(cell)类型，以便进行“工厂式”复制
      virtual void setCellType(NDTCell* type) {};

      ///reads map contents from .jff file
      virtual int loadFromJFF(FILE* jffin) const
      {
         std::cerr << "Calling from SpatialIndex.h\n";
         return -1;
      }
   };

} //end namespace

#ifdef NDT1_USE_MINI_EIGEN
#undef Eigen
#endif

#endif
