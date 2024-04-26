//
//   The interface of class "CLaserOdometry".
//

#pragma once

#include <stdio.h>
#include <mutex>

namespace mapping {

class CLaserOdometry
{
private:
    std::mutex odom_mtx;

private:
    CLaserOdometry();
    ~CLaserOdometry();

public:
    void Clear();

};

} // namespace mapping

