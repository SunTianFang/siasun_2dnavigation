
#ifndef TRANSFORM_PROTO_H_
#define TRANSFORM_PROTO_H_


namespace proto{


typedef  int8_t int8;
typedef  int16_t int16;
typedef  int32_t int32;
typedef  int64_t int64;
typedef  uint8_t uint8;
typedef uint16_t uint16  ;
typedef uint32_t uint32;
typedef uint64_t uint64;


struct Vector2d {
  double x ;
  double y ;
};

struct Vector2f {
  float x ;
  float y ;
};

struct Vector3d {
  double x ;
  double y ;
  double z ;
};

struct Vector3f {
  float x ;
  float y ;
  float z ;
};

struct Vector4f {
  float x ;
  float y ;
  float z ;
  float t ;
};

struct Quaterniond {
  double x ;
  double y ;
  double z ;
  double w ;
};

struct Quaternionf {
  float x ;
  float y ;
  float z ;
  float w ;
};

struct Rigid2d {
  Vector2d translation ;
  double rotation ;
};

struct Rigid2f {
  Vector2f translation ;
  float rotation ;
};

struct Rigid3d {
  Vector3d translation ;
  Quaterniond rotation ;
};

struct Rigid3f {
  Vector3f translation ;
  Quaternionf rotation ;
};

}

#endif
