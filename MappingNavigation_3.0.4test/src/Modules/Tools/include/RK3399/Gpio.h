//
//   The interface of class "CGpio".
//

#pragma once

#include <mutex>

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CGpio".
namespace RK3399 {

class CGpio
{
private:
    std::mutex mtx;
    int m_pin;
    int m_dir;

public:
    CGpio();

public:
    void Set(int pin, int dir);

    int Export();

    int Unexport();

    int Direction();

    int Write(int value);

    int Read();
};

} // namespace RK3399
