#include "stdafx.h"
#include "robo/LocalizationMethod.h"

bool CLocalizationMethod::Initialize()
{
    // 装载对应于该定位方法的地图
    if (!LoadMap())
        return false;

    // 装载对应于该定位方法的各种参数(定位参数、评价参数，....)
    if (!LoadParam())
        return false;

    // 其它的初始化工作(如果有的话)
    // ...

    return true;
}

bool CLocalizationMethod::OnReceivePose(const CStampedPos &pose)
{
    return true;
}

bool CLocalizationMethod::OnReceiveVelocity(const CVelocity &vel)
{
    return true;
}

bool CLocalizationMethod::OnReceiveOdomData()
{
    return true;
}

bool CLocalizationMethod::OnReceiveLaserScan(const CStampedPtCloud &Cloud)
{
    return true;
}
