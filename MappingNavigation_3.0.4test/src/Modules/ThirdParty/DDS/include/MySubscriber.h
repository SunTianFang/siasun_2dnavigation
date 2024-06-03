#ifndef MY_SUBSCRIBER
#define MY_SUBSCRIBER

#include "DDS_Data.h"
#include "DDS_DataPubSubTypes.h"
#include "DDS_DataSubscriber.h"

// template<typename T>
class MySubscriber
{
public:
    typedef std::function<void(const NAVPointCloudDDS* const)> Callback;
    DDS_DataSubscriber<NAVPointCloudDDSPubSubType, NAVPointCloudDDS> ddsSubRcData;

private:
    Callback callback_;

public:
    MySubscriber(){}
    ~MySubscriber(){}
    void RegistCallback(Callback cb);
    void startSubscrib();
    void initSubscriber();
    void ddsSubscribeHandler(const NAVPointCloudDDS* const ddsRcDataMsg);
};

#endif
