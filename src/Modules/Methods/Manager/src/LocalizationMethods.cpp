#include <stdafx.h>
#include "LocalizationMethods.h"
#include "NdtMethod.h"
#include "FeatureMethod.h"
#include "TemplateMethod.h"
#include "ScanMatchMethod.h"
#include "SlamMethod.h"

///////////////////////////////////////////////////////////////////////////////
//   实现“各种定位方法的集合”这一数据类型。

//
//   生成所有的定位方法。
//
bool CLocalizationMethods::Create()
{
    resize(NUM_LOCALIZATION_METHODS);

    CLocalizationMethod *method;

    // NDT方法
    method = new CNdtMethod;
    if (method == NULL)
        return false;
    at(0) = method;

    // 特征方法
    method = new CFeatureMethod;
    if (method == NULL)
        return false;
    at(1) = method;

    // 模板方法
    method = new CTemplateMethod;
    if (method == NULL)
        return false;
    at(2) = method;

    // by lishen
    method = new CScanMatchMethod;
    if (method == NULL)
        return false;
    at(3) = method;

    // by lishen
    method = new CSlamMethod;
    if (method == NULL)
        return false;
    at(4) = method;


    // !!! 将来如果增加新的定位方法，应在更改NUM_LOCALIZATION_METHODS宏的基础上在下面继续添加 !!!
    // ...
    // ...

    return true;
}

//
//   清除所有定位方法.
//
void CLocalizationMethods::Clear()
{
    for (int i = 0; i < (int)size(); i++)
    {      
        if (at(i) != NULL)
        {            
            delete at(i);
        }
    }

    clear();
}

//
//   初始化所有定位方法。
//
bool CLocalizationMethods::Initialize()
{
    return true;
}
bool CLocalizationMethods::UnloadMap() //lishen
{
    for (int i = 0; i < (int)size(); i++)
    {
        if (at(i) != NULL)
        {
            at(i)->UnloadMap();
        }
    }
}

//
//   根据给定的方法编号，生成对应的定位参数。
//
CLocalizationParam *CLocalizationMethods::CreateLocalizationParam(int index)
{
    if (index < 0 || index >= (int)size())
        return NULL;

    return at(index)->CreateLocParam();
}

//
//   从二进制文件装入地图。
//
bool CLocalizationMethods::LoadBinary(FILE *fp, string filename,int floor, bool bChangeFloor)
{

    for (int i = 0; i < (int)size(); i++)
    {
        if (!at(i)->LoadBinary(fp,filename,floor,bChangeFloor))
            return false;

    }

    return true;
}

//
//   将地图写入二进制文件。
//
bool CLocalizationMethods::SaveBinary(FILE *fp, string filename)
{
    for (int i = 0; i < (int)size(); i++)
        if (!at(i)->SaveBinary(fp,filename))
            return false;

    return true;
}
bool CLocalizationMethods::SaveFeatureBinary(FILE *fp, string filename)
{
    for (int i = 0; i < 3; i++)
        if (!at(i)->SaveBinary(fp,filename))
            return false;

    return true;
}
