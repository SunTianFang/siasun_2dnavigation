

#ifndef TRANSFORM_RIGID_TRANSFORM_H_
#define TRANSFORM_RIGID_TRANSFORM_H_

#include <cmath>
#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "mymath.h"
#include "port.h"


namespace transform {

template <typename FloatType>
class Rigid2 {
 public:
//	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::Matrix<FloatType, 2, 1,Eigen::DontAlign> Vector  ;
  typedef Eigen::Rotation2D<FloatType> Rotation2D  ;

  Rigid2() : translation_(Vector::Zero()), rotation_(Rotation2D::Identity()) {}
  Rigid2(const Vector& translation, const Rotation2D& rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid2(const Vector& translation, const double rotation)
      : translation_(translation), rotation_(rotation) {}

  static Rigid2 Rotation(const double rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Rotation(const Rotation2D& rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Translation(const Vector& vector) {
    return Rigid2(vector, Rotation2D::Identity());
  }

  static Rigid2<FloatType> Identity() { return Rigid2<FloatType>(); }

  template <typename OtherType>
  Rigid2<OtherType> cast() const {
    return Rigid2<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }

  Rotation2D rotation() const { return rotation_; }

  double normalized_angle() const {
    return common::NormalizeAngleDifference(rotation().angle());
  }

  Rigid2 inverse() const {
    const Rotation2D rotation = rotation_.inverse();
    const Vector translation = -(rotation * translation_);
    return Rigid2(translation, rotation);
  }

  std::string DebugString() const {
    std::string out;
    out.append("{ t: [");
    out.append(std::to_string(translation().x()));
    out.append(", ");
    out.append(std::to_string(translation().y()));
    out.append("], r: [");
    out.append(std::to_string(rotation().angle()));
    out.append("] }");
    return out;
  }

  bool SaveBinary(FILE *fp)
  {

         FloatType tmp = translation().x();
         if (fwrite(&tmp, sizeof(FloatType), 1, fp) != 1)
            return false;

         tmp = translation().y();
         if (fwrite(&tmp, sizeof(FloatType), 1, fp) != 1)
             return false;

         tmp = rotation().angle();
         if (fwrite(&tmp, sizeof(FloatType), 1, fp) != 1)
             return false;

        return true;
  }

  bool LoadBinary(FILE *fp)
  {
          FloatType tx, ty;
          if (fread(&tx, sizeof(FloatType), 1, fp) != 1)
            return false;
          translation_(0) = tx;

          // std::cout<<tx<<std::endl;

          if (fread(&ty, sizeof(FloatType), 1, fp) != 1)
            return false;
          translation_(1) = ty;
          //std::cout<<ty<<std::endl;


          FloatType rw;
          if (fread(&rw, sizeof(FloatType), 1, fp) != 1)
            return false;


          Eigen::Rotation2D<FloatType> ro(rw);

          rotation_ = ro;
          //translation_ = tr;
         return true;
  }

 private:
  Vector translation_;
  Rotation2D rotation_;
};

template <typename FloatType>
Rigid2<FloatType> operator*(const Rigid2<FloatType>& lhs,
                            const Rigid2<FloatType>& rhs) {
  return Rigid2<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      lhs.rotation() * rhs.rotation());
}

template <typename FloatType>
typename Rigid2<FloatType>::Vector operator*(
    const Rigid2<FloatType>& rigid,
    const typename Rigid2<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const transform::Rigid2<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

typedef  Rigid2<double> Rigid2d;
typedef  Rigid2<float> Rigid2f;

template <typename FloatType>
class Rigid3 {
 public:




 
  //Eigen::Matrix<double,4,1,Eigen::DontAlign>

  typedef  Eigen::Matrix<FloatType, 3, 1> Vector;
  typedef  Eigen::Quaternion<FloatType ,Eigen::DontAlign> Quaternion;
  typedef  Eigen::AngleAxis<FloatType> AngleAxis;

  Rigid3() : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}
  Rigid3(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid3(const Vector& translation, const AngleAxis& rotation)
      : translation_(translation), rotation_(rotation) {}

  static Rigid3 Rotation(const AngleAxis& angle_axis) {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }

  static Rigid3 Rotation(const Quaternion& rotation) {
    return Rigid3(Vector::Zero(), rotation);
  }

  static Rigid3 Translation(const Vector& vector) {
    return Rigid3(vector, Quaternion::Identity());
  }

/*  static Rigid3 FromArrays(const std::array<FloatType, 4>& rotation,
                           const std::array<FloatType, 3>& translation) {
    return Rigid3(Eigen::Map<const Vector>(translation.data()),
                  Eigen::Quaternion<FloatType>(rotation[0], rotation[1],
                                               rotation[2], rotation[3]));
  }*/

  static Rigid3<FloatType> Identity() { return Rigid3<FloatType>(); }

  template <typename OtherType>
  Rigid3<OtherType> cast() const {
    return Rigid3<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }
  const Quaternion& rotation() const { return rotation_; }

  Rigid3 inverse() const {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);
  }

  std::string DebugString() const {
    std::string out;
    out.append("{ t: [");
    out.append(std::to_string(translation().x()));
    out.append(", ");
    out.append(std::to_string(translation().y()));
    out.append(", ");
    out.append(std::to_string(translation().z()));
    out.append("], q: [");
    out.append(std::to_string(rotation().w()));
    out.append(", ");
    out.append(std::to_string(rotation().x()));
    out.append(", ");
    out.append(std::to_string(rotation().y()));
    out.append(", ");
    out.append(std::to_string(rotation().z()));
    out.append("] }");
    return out;
  }

  bool SaveBinary(FILE *fp)
  {

         FloatType tmp = translation().x();
         if (fwrite(&tmp, sizeof(FloatType), 1, fp) != 1)
            return false;

         tmp = translation().y();
         if (fwrite(&tmp, sizeof(FloatType), 1, fp) != 1)
             return false;
         tmp = translation().z();
         if (fwrite(&tmp, sizeof(FloatType), 1, fp) != 1)
             return false;
         tmp = rotation().w();
         if (fwrite(&tmp, sizeof(FloatType), 1, fp) != 1)
             return false;
         tmp = rotation().x();
         if (fwrite(&tmp, sizeof(FloatType), 1, fp) != 1)
             return false;
         tmp = rotation().y();
         if (fwrite(&tmp, sizeof(FloatType), 1, fp) != 1)
             return false;
         tmp = rotation().z();
         if (fwrite(&tmp, sizeof(FloatType), 1, fp) != 1)
             return false;

        return true;
  }

  bool LoadBinary(FILE *fp)
  {
          FloatType tx, ty, tz;
          if (fread(&tx, sizeof(FloatType), 1, fp) != 1)
            return false;
          translation_(0) = tx;

          // std::cout<<tx<<std::endl;

          if (fread(&ty, sizeof(FloatType), 1, fp) != 1)
            return false;
          translation_(1) = ty;

          //std::cout<<ty<<std::endl;
          if (fread(&tz, sizeof(FloatType), 1, fp) != 1)
            return false;
          translation_(2) = tz;

          //std::cout<<tz<<std::endl;

          FloatType rw;
          if (fread(&rw, sizeof(FloatType), 1, fp) != 1)
            return false;
          FloatType rx;
          if (fread(&rx, sizeof(FloatType), 1, fp) != 1)
            return false;
          FloatType ry;
          if (fread(&ry, sizeof(FloatType), 1, fp) != 1)
            return false;
          FloatType rz;
          if (fread(&rz, sizeof(FloatType), 1, fp) != 1)
            return false;

          Eigen::Matrix<FloatType, 3, 1> tr(tx,ty,tz);
          Eigen::Quaternion<FloatType> ro(rw, rx, ry, rz);

          rotation_ = ro;
          translation_ = tr;
          return true;
  }

  bool IsValid() const {
    return !std::isnan(translation_.x()) && !std::isnan(translation_.y()) &&
           !std::isnan(translation_.z()) &&
           std::abs(FloatType(1) - rotation_.norm()) < FloatType(1e-3);
  }

 private:
  Vector translation_;
  Quaternion rotation_;
};

template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
                            const Rigid3<FloatType>& rhs) {
  return Rigid3<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      (lhs.rotation() * rhs.rotation()).normalized());
}

template <typename FloatType>
typename Rigid3<FloatType>::Vector operator*(
    const Rigid3<FloatType>& rigid,
    const typename Rigid3<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const transform::Rigid3<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

typedef Rigid3<double> Rigid3d ;
typedef Rigid3<float> Rigid3f ;

// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
// specification http://wiki.ros.org/urdf/XML/joint.
Eigen::Quaterniond RollPitchYaw(double roll, double pitch, double yaw);

// Returns an transform::Rigid3d given a 'dictionary' containing 'translation'
// (x, y, z) and 'rotation' which can either we an array of (roll, pitch, yaw)
// or a dictionary with (w, x, y, z) values as a quaternion.
//Rigid3d FromDictionary(common::LuaParameterDictionary* dictionary);






/*

template <typename FloatType>
class myRigid3 {
 public:

 	struct myQuaternion
	{
		FloatType w;
		FloatType x;
		FloatType y;
		FloatType z;
		
		myQuaternion(FloatType _w, FloatType _x,FloatType _y,FloatType _z)
		{
			w = _w;
			x = _x;
			y = _y;
			z = _z;
		} 
		myQuaternion(myQuaternion &_tmp)
		{
			w = _tmp.w;
			x = _tmp.x;
			y = _tmp.y;
			z = _tmp.z;
		}    
		
		myQuaternion Identity()
		{

			return myQuaternion(1,0,0,0);

		}
		myQuaternion conjugate() const
		{
  			return myQuaternion(w,-x,-y,-z);
		}
		void operator=(const myQuaternion &rhs)
		{
			w = rhs.w;
			x = rhs.x;
			y = rhs.y;
			z = rhs.z;


		}
		bool operator==(const myQuaternion &rhs)
		{
			if(w == rhs.w && x == rhs.x && y==rhs.y && z==rhs.z)
				return true;
			else
				return false;
		}



	};
	struct myMatrix
	{
		FloatType x;
		FloatType y;
		FloatType z;

		myMatrix(FloatType _x,FloatType _y,FloatType _z)
		{
			x = _x;
			y = _y;
			z = _z;
		}

		myMatrix(myMatrix &_tmp)
		{
			x = _tmp.x;
			y = _tmp.y;
			z = _tmp.z;
		}

		myMatrix Zero()
		{

			return myMatrix(0.0,0.0,0.0);
		}
		myMatrix operator-(myMatrix &rhs)
		{

		  	FloatType _x = x -  rhs.x;
		  	FloatType _y = y -  rhs.y;
		  	FloatType _z = z -  rhs.z;			
			
		 	 return myMatrix(_x,_y,_z);

		}
		void operator=(const myMatrix &rhs)
		{
	
			x = rhs.x;
			y = rhs.y;
			z = rhs.z;


		}
		bool operator==(const myMatrix &rhs)
		{
			if(x == rhs.x && y==rhs.y && z==rhs.z)
				return true;
			else
				return false;
		}




	};

 
  typedef  myMatrix Vector;
  typedef  myQuaternion Quaternion;


  myRigid3() : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}
  myRigid3(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}



  static myRigid3 Rotation(const Quaternion& rotation) {
    return myRigid3(Vector::Zero(), rotation);
  }

  static myRigid3 Translation(const Vector& vector) {
    return myRigid3(vector, Quaternion::Identity());
  }



		bool operator==(const myRigid3 &rhs)
		{
			if(translation_ == rhs.translation() && rotation_==rhs.rotation() )
				return true;
			else
				return false;
		}



  static myRigid3<FloatType> Identity() { return myRigid3<FloatType>(); }





		myMatrix product(myQuaternion &lhs, myMatrix &rhs)
		{

		 

		  const FloatType tx  = FloatType(2)*lhs.x;
		  const FloatType ty  = FloatType(2)*lhs.y;
		  const FloatType tz  = FloatType(2)*lhs.z;
		  const FloatType twx = tx*lhs.w;
		  const FloatType twy = ty*lhs.w;
		  const FloatType twz = tz*lhs.w;
		  const FloatType txx = tx*lhs.x;
		  const FloatType txy = ty*lhs.x;
		  const FloatType txz = tz*lhs.x;
		  const FloatType tyy = ty*lhs.y;
		  const FloatType tyz = tz*lhs.y;
		  const FloatType tzz = tz*lhs.z;


		  FloatType matrix[3][3];

		  matrix[0][0] = FloatType(1)-(tyy+tzz);
		  matrix[0][1] = txy-twz;
		  matrix[0][2] = txz+twy;
		  matrix[1][0] = txy+twz;
		  matrix[1][1] = FloatType(1)-(txx+tzz);
		  matrix[1][2] = tyz-twx;
		  matrix[2][0] = txz-twy;
		  matrix[2][1] = tyz+twx;
		  matrix[2][2] = FloatType(1)-(txx+tyy);

		  FloatType x = matrix[0][0]*rhs.x +  matrix[0][1]*rhs.y +  matrix[0][2] *rhs.z;
		  FloatType y = matrix[1][0]*rhs.x +  matrix[1][1]*rhs.y +  matrix[1][2] *rhs.z;
		  FloatType z = matrix[2][0]*rhs.x +  matrix[2][1]*rhs.y +  matrix[2][2] *rhs.z;			
			return myMatrix(x,y,z);
		}





  template <typename OtherType>
  myRigid3<OtherType> cast() const {
    return myRigid3<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }
  const Quaternion& rotation() const { return rotation_; }

  myRigid3 inverse() const {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -product(rotation , translation_);
    return myRigid3(translation, rotation);
  }


 private:
  Vector translation_;
  Quaternion rotation_;
};

/*

template <typename FloatType>
myRigid3<FloatType> operator*(const myRigid3<FloatType>& lhs,
                            const myRigid3<FloatType>& rhs) {
  return myRigid3<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      (lhs.rotation() * rhs.rotation()).normalized());
}

template <typename FloatType>
typename myRigid3<FloatType>::Vector operator*(
    const myRigid3<FloatType>& rigid,
    const typename myRigid3<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}


typedef myRigid3<double> myRigid3d ;
typedef myRigid3<float> myRigid3f ;

*/


struct myRigid3d
{

		double rw;
		double rx;
		double ry;
		double rz;

		double vx;
		double vy;
		double vz;

} ;


Rigid3d myToRigid3d(const  myRigid3d &my);
myRigid3d Rigid3dTomy(const Rigid3d &rigid);






}  // namespace transform


#endif  // TRANSFORM_RIGID_TRANSFORM_H_
