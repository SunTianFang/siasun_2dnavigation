#ifndef __CStockedObjects
#define __CStockedObjects

#include "BasObject.h"

#ifdef USE_YAML
#include "yaml-cpp/yaml.h"
#endif

///////////////////////////////////////////////////////////////////////////////
//    定义“CStockedObjects”，用来存放所有已定义的物体。
class DllExport CStockedObjects : public std::vector<CBasObject>
{
  public:
    CStockedObjects() {}

    // 重载操作符“+=”，对CObjTemplate对象进行添加
    CStockedObjects &operator+=(const CBasObject &obj);

    // 重载操作符“+=”，对CStockedObjects对象进行添加
    CStockedObjects &operator+=(const CStockedObjects &objs);

    // 根据模板名称找到模板所索引号
    int FindByName(std::string name);

    // 从文本文件装入数据
    bool LoadText(FILE *fp);

    // 将数据写入文本文件
    bool SaveText(FILE *fp);

    // 从二进制文件装入数据
    bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    bool SaveBinary(FILE *fp);
    bool LoadPadYaml(int count,vector<vector<float>> points);

#ifdef USE_YAML
    // 从Yaml文件读入数据
    bool LoadYaml(YAML::Node &config);
#endif
};
#endif
