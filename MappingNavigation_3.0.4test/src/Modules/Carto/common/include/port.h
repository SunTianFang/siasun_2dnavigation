
#ifndef COMMON_PORT_H_
#define COMMON_PORT_H_

#include <cinttypes>
#include <cmath>
//#include <math.h>
#include <string>



using namespace std;




typedef  int8_t int8;
typedef  int16_t int16;
typedef  int32_t int32;
typedef  int64_t int64;
typedef  uint8_t uint8;
typedef uint16_t uint16  ;
typedef uint32_t uint32;
typedef uint64_t uint64;


namespace common {

inline int RoundToInt(const float x) { return std::lround(x); }

inline int RoundToInt(const double x) { return std::lround(x); }

inline int64 RoundToInt64(const float x) { return lround(x); }

inline int64 RoundToInt64(const double x) { return lround(x); }



}  // namespace common


#endif  // CARTOGRAPHER_COMMON_PORT_H_
