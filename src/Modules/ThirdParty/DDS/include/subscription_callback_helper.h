
// https://docs.ros.org/en/api/roscpp/html/subscription__callback__helper_8h_source.html

#ifndef SUBSCRIPTION_CALLBACK_HELPER_H
#define SUBSCRIPTION_CALLBACK_HELPER_H

#include <boost/function.hpp>
#include <boost/make_shared.hpp>

namespace RoboSLAM3 {

class SubscriptionCallbackHelper
{
public:
    virtual ~SubscriptionCallbackHelper() {}

    virtual void call( void* params) = 0;
};
typedef boost::shared_ptr <SubscriptionCallbackHelper> SubscriptionCallbackHelperPtr;

template <typename P>
class SubscriptionCallbackHelperT : public SubscriptionCallbackHelper
{

public:
    typedef boost::function <void (P)> Callback;

    SubscriptionCallbackHelperT ( const Callback& callback )
        : callback_(callback)
    {}

    virtual void call( void* params )
    {
        // std::cout << "helper.h dds: seq = " << ((P)params)->cloud_deskewed().header().seq() << ", stamp = " << std::setprecision(19) << ((P)params)->cloud_deskewed().header().stamp() << ", frame_id = " << ((P)params)->cloud_deskewed().header().frame_id() << std::endl;

        callback_ ( (P) params );
    }

private:
    Callback callback_;
};




}   // namespace RoboSLAM3

#endif // SUBSCRIPTION_CALLBACK_HELPER_H