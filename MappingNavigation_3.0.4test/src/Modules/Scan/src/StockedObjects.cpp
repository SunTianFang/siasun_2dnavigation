#include <stdafx.h>
#include "StockedObjects.h"
#include "LineElement.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//   “CStockedObjects”类的实现。

//
//   重载操作符“+=”，对CObjTemplate对象进行添加。
//
CStockedObjects &CStockedObjects::operator+=(const CBasObject &obj)
{
    push_back(obj);
    return *this;
}

//
//   重载操作符“+=”，对CStockedObjects对象进行添加。
//
CStockedObjects &CStockedObjects::operator+=(const CStockedObjects &objs)
{
    for (int i = 0; i < (int)objs.size(); i++)
        push_back(at(i));

    return *this;
}

//
//   根据模板名称找到模板所索引号。
//
int CStockedObjects::FindByName(std::string name)
{
    for (int i = 0; i < (int)size(); i++)
        if (at(i).name == name)
            return i;

    return -1;
}

//
//   从数据文件装入数据。
//
bool CStockedObjects::LoadText(FILE *fp)
{
    clear();

    // 先读入物体数量
    int count;
    if (fscanf(fp, "%d\n", &count) != 1)
        return false;

    resize(count);

    // 依次读入各个物体的参数
    for (int i = 0; i < count; i++)
    {
        if (!at(i).LoadText(fp))
            return false;
    }

    return true;
}

//
//   将数据写入文本文件。
//
bool CStockedObjects::SaveText(FILE *fp)
{
    // 先读入物体数量
    int count = (int)size();
    if (fscanf(fp, "%d\n", &count) != 1)
        return false;

    // 依次读入各个物体的参数
    for (int i = 0; i < count; i++)
    {
        if (!at(i).SaveText(fp))
            return false;
    }

    return true;
}

//
//   从二进制文件装入数据。
//
bool CStockedObjects::LoadBinary(FILE *fp)
{
    clear();

    // 先读入物体数量
    int count;
    if (fread(&count, sizeof(int), 1, fp) != 1)
        return false;

    resize(count);

    // 依次读入各个物体的参数
    for (int i = 0; i < count; i++)
    {
        if (!at(i).LoadBinary(fp))
            return false;
    }

    return true;
}

//
//   将数据写入二进制文件。
//
bool CStockedObjects::SaveBinary(FILE *fp)
{
    // 先读入物体数量
    int count = (int)size();
    if (fwrite(&count, sizeof(int), 1, fp) != 1)
        return false;

    // 依次写入各个物体的参数
    for (int i = 0; i < count; i++)
    {
        if (!at(i).SaveBinary(fp))
            return false;
    }

    return true;
}

// by DQ 从Pad读入模板数据
bool CStockedObjects::LoadPadYaml(int count, vector<vector<float> > points)
{
    clear();
    // 先读入模型的数量
    resize(count);

    for (int i = 0;i < count; i++)
    {

        // 读取模型名称
        at(i).name = i+"";
        at(i).m_nTempId = i;

        // 如果是个连续的多段线
        if(points[i].back() == 0)
        {
            points[i].pop_back();
            // 取出多边形各顶点坐标
            int num = points[i].size() / 2;
            for (int j = 0; j < num - 1; j++)
            {
                CPnt ptStart(points[i][2 * j], points[i][2 * j + 1]);
                CPnt ptEnd(points[i][2 * j + 2], points[i][2 * j + 3]);
                CLine line(ptStart, ptEnd);
                at(i) += CLineElement(line);
            }
        }

        // 如果是一个“圆形”模型
        else if (points[i].back() == 1)
        {
            points[i].pop_back();
            // 读取圆的中心点和半径
            CPnt center(points[i][0], points[i][1]);
            float radius = points[i][3];

            // 构造圆形对象
            CCircle circle(center, radius);

            at(i) += circle;
        }
    }
    return true;
}

#ifdef USE_YAML
//
//   从Yaml文件读入数据。
//
bool CStockedObjects::LoadYaml(YAML::Node &config)
{
    // 先读入模型的数量
    int count = 0;
    for (YAML::const_iterator it = config.begin(); it != config.end(); it++)
        count++;

    resize(count);

    int i = 0;
    for (YAML::const_iterator it = config.begin(); it != config.end(); it++, i++)
    {
        // 取得模型主类型名
        string strItem = it->first.as<string>();

        // 取得对应于此主类型名的YAML节点
        YAML::Node subNode = config[strItem];

        // 取得此模型的类型名
        string strType = subNode["type"].as<string>();

        // 读取模型名称
        at(i).name = subNode["name"].as<string>();
        at(i).m_nTempId = i;

        // 如果是一个“多边形”模型(封闭型，最终点即起点)
        if (strType == "polygon")
        {
            // 取出多边形各顶点坐标
            vector<float> points = subNode["points"].as<vector<float>>();

            int num = points.size() / 2;
            CPnt *p = new CPnt[num + 1];
            for (int j = 0; j < num; j++)
            {
                p[j].x = points[2 * j];
                p[j].y = points[2 * j + 1];
            }

            // 自动加入最终点
            p[num].x = p[0].x;
            p[num].y = p[0].y;

            at(i) += CPolyRegion(num + 1, p);
            delete []p;
        }

        // 如果是一个“圆形”模型
        else if (strType == "circle")
        {
            // 读取圆的中心点和半径
            vector<float> v = subNode["center"].as<vector<float>>();
            CPnt center(v[0], v[1]);
            float radius = subNode["radius"].as<float>();

            // 构造圆形对象
            CCircle circle(center, radius);

            at(i) += circle;
        }

        // 如果是个连续的多段线
        else if (strType == "polyline")
        {
            // 取出多边形各顶点坐标
            vector<float> points = subNode["points"].as<vector<float>>();
            int num = points.size() / 2;
            for (int j = 0; j < num - 1; j++)
            {
                CPnt ptStart(points[2 * j], points[2 * j + 1]);
                CPnt ptEnd(points[2 * j + 2], points[2 * j + 3]);
                CLine line(ptStart, ptEnd);
                at(i) += CLineElement(line);
            }
        }

        // 如果是一个复合类型物体
        else if (strType == "complex")
        {
            vector<string> partNames = subNode["objects"].as<vector<string>>();
            vector<float> poses = subNode["poses"].as<vector<float>>();

            for (int j = 0; j < (int)partNames.size(); j++)
            {
                int objId = FindByName(partNames[j]);
                if (objId >= 0)
                {
                    CPosture pst(poses[3 * j], poses[3 * j + 1], poses[3 * j + 2]);
                    CBasObject obj(at(objId));
                    obj.InvTransform(pst);
                    at(i) += obj;
                }
            }
        }
    }
    return true;
}
#endif
