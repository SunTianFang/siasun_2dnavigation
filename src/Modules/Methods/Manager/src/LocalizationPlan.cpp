#include "stdafx.h"
#include "LocalizationPlan.h"
#include "LocalizationMethods.h"

#define READ_AS_V4_FORMAT
#define SAVE_AS_V4_FORMAT

CLocalizationMethods *CLocalizationRect::methods_ = NULL;

///////////////////////////////////////////////////////////////////////////////
//   定义“定位应用矩形区域”。

CLocalizationRect::CLocalizationRect()
{
    for (int i = 0; i < MAX_METHODS_PER_RECT; i++)
    {
        methodId_[i] = -1;
        param_[i] = NULL;
    }
}

CLocalizationRect::CLocalizationRect(const CLocalizationRect &other) : CRectangle(other)
{
    for (int i = 0; i < MAX_METHODS_PER_RECT; i++)
    {
        methodId_[i] = other.methodId_[i];
        param_[i] = NULL;

        if (other.param_[i] != NULL)
            param_[i] = other.param_[i]->Duplicate();
    }
}

//
//   根据给定的矩形区域生成对象。
//
CLocalizationRect::CLocalizationRect(const CRectangle &r) : CRectangle(r)
{
    // 其它所有方法都不启用
    for (int i = 0; i < MAX_METHODS_PER_RECT; i++)
    {
        methodId_[i] = -1;
        param_[i] = NULL;
    }
    // 仅第一个方法设置为已注册的首个方法
    methodId_[0] = 0;
    param_[0] = methods_->CreateLocalizationParam(0);
}

CLocalizationRect::~CLocalizationRect()
{
    ClearMethods();
}

//
//   清除所有“定位方法数据”。
//
void CLocalizationRect::ClearMethods()
{
    for (int i = 0; i < MAX_METHODS_PER_RECT; i++)
    {
        methodId_[i] = -1;

        if (param_[i] != NULL)
        {
            delete param_[i];
            param_[i] = NULL;
        }
    }
}

//
//   设置指向定位方法集合的指针。
//
void CLocalizationRect::SetLocalizationMethods(CLocalizationMethods *methods)
{
    methods_ = methods;
}

//
//   判断指定的定位方法在此矩形区域中是否被启用。
//   注意: methodId > 0
//   返回值：
//     >= 0: 启用的方法的序号
//     -1:   未被启用
//
int CLocalizationRect::CheckMethodUsed(int methodId)
{
    for (int i = 0; i < MAX_METHODS_PER_RECT; i++)
        if (methodId_[i] == methodId)
            return i;

    return -1;
}

bool CLocalizationRect::LoadBinary(FILE *fp)
{

    ClearMethods();

    // 先读入矩形数据
    if (!CRectangle::LoadBinary(fp))
        return false;

    // 再读入定位方法编号
    if (fread(methodId_, sizeof(int), MAX_METHODS_PER_RECT, fp) != MAX_METHODS_PER_RECT)
        return false;

#ifdef READ_AS_V4_FORMAT
    for (int i = 0; i < MAX_METHODS_PER_RECT; i++)
    {
        if (methodId_[i] >= 0)
        {
            // 根据方法编号生成参数块
            param_[i] = methods_->CreateLocalizationParam(methodId_[i]);

            if (param_[i] == NULL)
                return false;
            // 读入对应于该方法的参数
            if (!param_[i]->LoadBinary(fp))
                return false;
        }
    }
#else
    // 读入各种方法的参数
    int j = 0;
    for (int i = 0; i < (int)methods_->size(); i++)
    {

        int n;
        if (fread(&n, sizeof(int), 1, fp) != 1)
            return false;

        if (n == 0)
            continue;

        // 根据方法编号生成参数块
        param_[j] = methods_->CreateLocalizationParam(i);


        if (param_[j] == NULL)
            return false;

        // 读入对应于该方法的参数
        if (!param_[j++]->LoadBinary(fp))
            return false;

    }
#endif


    return true;
}

bool CLocalizationRect::SaveBinary(FILE *fp)
{
    // 先写入矩形数据
    if (!CRectangle::SaveBinary(fp))
        return false;

    // 再写入定位方法编号
    if (fwrite(methodId_, sizeof(int), MAX_METHODS_PER_RECT, fp) != MAX_METHODS_PER_RECT)
        return false;

#ifdef SAVE_AS_V4_FORMAT
    for (int i = 0; i < MAX_METHODS_PER_RECT; i++)
    {
        if (methodId_[i] >= 0)
        {
            if(param_[i]!=NULL)
            {
                if (!param_[i]->SaveBinary(fp))
                    return false;
            }
        }
    }
#else
    // 写入各种方法的参数
    int j = 0;
    for (int i = 0; i < (int)methods_->size(); i++)
    {
        int index = CheckMethodUsed(i);
        int n = (index >= 0);
        if (fwrite(&n, sizeof(int), 1, fp) != 1)
            return false;

        if (n == 0)
            continue;
        if(param_[i]!=NULL)
        {
            param_[index]->SaveBinary(fp);
        }
    }
#endif

    return true;
}

///////////////////////////////////////////////////////////////////////////////

//
//   根据给定的定位矩形及参数块，生成对应于“首选”或“备选”的具体定位指令。
//
void CLocalizationInst::Create(const CLocalizationRect &r, int index)
{
    methodId_ = r.methodId_[index];
    if (methodId_ >= 0)
    {
        param_ = r.param_[index];
        localizationRect_.Create(r.GetLeftTopPoint(), r.GetRightBottomPoint());
    }
    else
        param_ = NULL;
}

///////////////////////////////////////////////////////////////////////////////

//
//   对于给定的姿态，根据定位方案形成对应的“定位指令”。
//
CLocalizationInst *CLocalizationPlan::GetInstructions(const CPosture &pst)
{
    for (int i = 0; i <(int)size(); i++)
    {
        // 查看当前的矩形是否包含给定的位姿点
        if (at(i)->Contain(pst))
        {
            // 如果包含，则根据矩形内的参数形成定位指令
            CLocalizationRect *r = dynamic_cast<CLocalizationRect*>(at(i));
            if (r == NULL)
                continue;

            // 指令分“首选”/“备选”两级
            for (int j = 0; j < MAX_METHODS_PER_RECT; j++)
                instructions_[j].Create(*r, j);

            // 找到的情况下，总是返回指令数组的首地址
            return instructions_;
        }
    }

    return NULL;    // 没有找到，返回NULL
}
