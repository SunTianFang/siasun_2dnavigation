
#ifndef TRANSFORM_TIMESTAMPED_TRANSFORM_H_
#define TRANSFORM_TIMESTAMPED_TRANSFORM_H_

#include "mytime.h"
#include "rigid_transform.h"
#include "mylogging.h"

using namespace myLogging;

namespace transform {

struct TimestampedTransform {

  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  common::Time time;
  transform::Rigid3d transform;
};

TimestampedTransform Interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const common::Time time);

}  // namespace transform


#endif  // TRANSFORM_TIMESTAMPED_TRANSFORM_H_
