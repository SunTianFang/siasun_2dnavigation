#ifndef __TOPCAMPUB__
#define __TOPCAMPUB__
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

typedef struct __CONFIGPARAM{
    // auto adjustment info
    int caa_score, caa_upper, caa_lower, caa_step;
    double caa_lr;
    double initExposure, initGain;
    bool high_mode;
    // states
    bool caa_enable;
} CONFIGPARAM;


class Config {

public:
    // auto adjustment info
    int caa_score, caa_upper, caa_lower, caa_step;
    double caa_lr;
    double initExposure, initGain;

    // states
    bool caa_enable;
public:

  Config();
  void configParamInit(CONFIGPARAM n);
};

//----------------------------------------------------------------

//下面的类为相机图片获取类

//----------------------------------------------------------------
class topCamPub{
public:
    topCamPub();

/*初始化相机接口，如果失败，返回FALSE*/
    bool Init(void);

/*获取图像接口，如果失败，返回FALSE*/
    bool GetFrame(cv::Mat &image);

/*获取相机ID*/
    const std::string& getCamId();

/*关闭数据获取*/
    void Stop();

public:
    Config conf;

};

#endif

