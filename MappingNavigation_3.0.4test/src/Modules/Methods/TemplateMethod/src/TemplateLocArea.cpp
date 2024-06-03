#include <stdafx.h>
#include "TemplateLocArea.h"

///////////////////////////////////////////////////////////////////////////////
//   定义基于模板的定位参数。

CTemplateLocalizationParam::CTemplateLocalizationParam()
{
    objId = 0.1;
    directionAngle = CAngle::ToRadian(30);
    useMultiTemplate = 0;
    templateRatio = 60;
    useTemplateNum = 1;
    objNames.resize(1);
    objNames[0] = "";
}

//
//   生成本数据的一个副本。
//
CLocalizationParam *CTemplateLocalizationParam::Duplicate()
{
    CTemplateLocalizationParam *copy = new CTemplateLocalizationParam;
    *copy = *this;
    return copy;
}

//
//   从二进制文件读取数据。
//
bool CTemplateLocalizationParam::LoadBinary(FILE *fp)
{
#if 1
    // 读入物体的数量
    int countObjs;
    if (fread(&countObjs, sizeof(int), 1, fp) != 1)
        return false;

    // 生成字串向量
    objNames.clear();
    objNames.resize(countObjs);

    // 依次读入各物体的名称
    for (int i = 0; i < countObjs; i++)
    {
        char buf[64];

        // 读入物体名称长度
        int len;
        if (fread(&len, sizeof(int), 1, fp) != 1)
            return false;

        // 名称最大长度为64字节
        if (len < 0 || len >= 64)
            return false;

        // 读入物体名称字串
        if (fread(buf, sizeof(char), len, fp) != len)
            return false;
        buf[len] = '\0';

        // 为名字串赋值
        objNames[i] = buf;
    }
#endif

    float f[2];
    if (fread(f, sizeof(float), 2, fp) != 2)
        return false;
    objId = f[0];
    directionAngle = f[1];

    if (fread(&useMultiTemplate, sizeof(int), 1, fp) != 1)
        return false;

    if (fread(&templateRatio, sizeof(float), 1, fp) != 1)
        return false;
    // 11.7版本，与旧版地图区别，多读写一位“使用模板个数”
    if (fread(&useTemplateNum, sizeof(int), 1, fp) != 1)
        return false;

    return true;
}

//
//   将数据写入二进制文件。
//
bool CTemplateLocalizationParam::SaveBinary(FILE *fp)
{
#if 1
    // 写入物体的数量
    int countObjs = (int)objNames.size();
    if (fwrite(&countObjs, sizeof(int), 1, fp) != 1)
        return false;

    // 依次写入各物体的名称
    for (int i = 0; i < (int)objNames.size(); i++)
    {
        // 写入物体名称长度
        int len = objNames[i].length();
        if (fwrite(&len, sizeof(int), 1, fp) != 1)
            return false;

        // 定入物体名称字串
        const char *p = objNames[i].c_str();
        if (fwrite(p, sizeof(char), len, fp) != len)
            return false;
    }
#endif

    float f[2] = {objId, directionAngle};
    if (fwrite(f, sizeof(float), 2, fp) != 2)
        return false;
    // 改为：是否使用多个模板共同定位
    if (fwrite(&useMultiTemplate, sizeof(int), 1, fp) != 1)
        return false;

    if (fwrite(&templateRatio, sizeof(float), 1, fp) != 1)
        return false;
    // 增加使用模板的数量信息（同一个矩形区域中，使用的模板个数）
    if (fwrite(&useTemplateNum, sizeof(int), 1, fp) != 1)
        return false;

    return true;
}

///////////////////////////////////////////////////////////////////////////////
//   模板定位应用矩形: 定义在该矩形区域内的模板定位参数。

//
//   从二进制文件读取数据。
//
bool CTemplateLocalizationRect::LoadBinary(FILE *fp)
{
    if (!CRectangle::LoadBinary(fp))
        return false;

    return param_.LoadBinary(fp);
}

//
//   将数据写入二进制文件。
//
bool CTemplateLocalizationRect::SaveBinary(FILE *fp)
{
    if (!CRectangle::SaveBinary(fp))
        return false;

    return param_.SaveBinary(fp);
}
