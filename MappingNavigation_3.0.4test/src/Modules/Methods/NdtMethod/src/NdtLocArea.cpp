#include <stdafx.h>
#include "NdtLocArea.h"

///////////////////////////////////////////////////////////////////////////////
//   定义基于NDT的定位参数。

CNdtLocalizationParam::CNdtLocalizationParam()
{
    submapId = 1;    // Change By Sam
    localMode = 0;
    threshNumX = 20;
    threshNumY = 20;
    threshRatioX = 40;
    threshRatioY = 40;
    disLimit = 10;
    enableGridMatch = false;
    enableSlam = false;
    gridMatchType = 0;

    fastMatchLineSearchWindow = 0.5;
    fastMatchAngleSearchWindow = 15.0;  //degree

}

CNdtLocalizationParam::CNdtLocalizationParam(const CNdtLocalizationParam &other)
{
    *this = other;
}

//
//   生成本数据的一个副本。
//
CLocalizationParam *CNdtLocalizationParam::Duplicate()
{
    CNdtLocalizationParam *copy = new CNdtLocalizationParam;
    *copy = *this;
    return copy;
}

bool CNdtLocalizationParam::LoadBinary(FILE *fp)
{
    if (fread(&submapId, sizeof(int), 1, fp) != 1)
        return false;

    // 读取定位模式
    if (fread(&localMode, sizeof(int), 1, fp) != 1)
        return false;

#if 1
    // 读取SLAM定位使能
    int m;
    if (fread(&m, sizeof(int), 1, fp) != 1)
        return false;
    enableSlam = (bool)m;

    // 读取最大允许SLAM定位距离
    if (fread(&maxSlamDist, sizeof(float), 1, fp) != 1)
        return false;
#endif

#if 0
    if (fread(&N_, sizeof(int), 1, fp) != 1)
        return false;
#else
    int n[2] = {threshNumX, threshNumY};
    if (fread(n, sizeof(int), 2, fp) != 2)
        return false;

    threshNumX = n[0];
    threshNumY = n[1];

    float f[3];
    if (fread(f, sizeof(float), 3, fp) != 3)
        return false;

     threshRatioX = f[0];
     threshRatioY = f[1];
     disLimit = f[2];
#endif

     //lishen

    // 读取扩展定位使能
 #if 1
    if (fread(&m, sizeof(int), 1, fp) != 1)
        return false;
    enableGridMatch = (bool)m;


    if (fread(&m, sizeof(int), 1, fp) != 1)
        return false;
    gridMatchType = (int)m;



    float fast[2];
    if (fread(fast, sizeof(float), 2, fp) != 2)
        return false;
    fastMatchLineSearchWindow = fast[0];

    fastMatchAngleSearchWindow = fast[1];


#endif

    return true;
}

bool CNdtLocalizationParam::SaveBinary(FILE *fp)
{
    if (fwrite(&submapId, sizeof(int), 1, fp) != 1)
        return false;

    // By Sam: 写入定位模式
    if (fwrite(&localMode, sizeof(int), 1, fp) != 1)
        return false;

    // 写入SLAM定位使能
    int m = enableSlam;
    if (fwrite(&m, sizeof(int), 1, fp) != 1)
        return false;

    // 写入最大允许SLAM定位距离
    if (fwrite(&maxSlamDist, sizeof(float), 1, fp) != 1)
        return false;

    int n[2] = {threshNumX, threshNumY};
    if (fwrite(n, sizeof(int), 2, fp) != 2)
        return false;

    float f[3] = {threshRatioX, threshRatioY, disLimit};
    if (fwrite(f, sizeof(float), 3, fp) != 3)
        return false;

     //lishen
    //写入扩展定位使能
    m = enableGridMatch;
    if (fwrite(&m, sizeof(int), 1, fp) != 1)
        return false;

    m = gridMatchType;
    if (fwrite(&m, sizeof(int), 1, fp) != 1)
        return false;

    float fast[2] = {fastMatchLineSearchWindow, fastMatchAngleSearchWindow};

   //     std::cout<<"save fastMatchLineSearchWindow" <<fastMatchLineSearchWindow<<std::endl;
    if (fwrite(fast, sizeof(float), 2, fp) != 2)
        return false;

    return true;
}

///////////////////////////////////////////////////////////////////////////////
//   NDT定位应用矩形: 定义在该矩形区域内的NDT定位参数。

//
//   从二进制文件读取数据。
//
bool CNdtLocalizationRect::LoadBinary(FILE *fp)
{
    if (!CRectangle::LoadBinary(fp))
        return false;

    return param_.LoadBinary(fp);
}

//
//   将数据写入二进制文件。
//
bool CNdtLocalizationRect::SaveBinary(FILE *fp)
{
    if (!CRectangle::SaveBinary(fp))
        return false;

    return param_.SaveBinary(fp);
}
