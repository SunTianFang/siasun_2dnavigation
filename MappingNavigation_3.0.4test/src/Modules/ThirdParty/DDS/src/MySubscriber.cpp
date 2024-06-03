
#include "MySubscriber.h"
#include <iostream>
#include <thread>

using namespace std;

void MySubscriber::initSubscriber()
{
    bool bResult = ddsSubRcData.init("/AgvData", &MySubscriber::ddsSubscribeHandler, this);
    cout << "MySubscriber::initSubscriber bResult = " << (int)bResult << endl;
}

void MySubscriber::ddsSubscribeHandler(const NAVPointCloudDDS* const ddsRcDataMsg)
{
    cout << "MySubscriber::ddsSubscribeHandler in " << endl;
    if (callback_ != nullptr)
    {
        callback_(ddsRcDataMsg);
    }
}

void MySubscriber::RegistCallback(Callback cb)
{
    callback_ = cb;
}
