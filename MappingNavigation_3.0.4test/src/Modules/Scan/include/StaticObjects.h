#ifndef __CStaticObjects
#define __CStaticObjects

#include <string>
#include "StockedObjects.h"

#ifdef QT_VERSION
#include <QColor>
#include <QPainter>
#endif

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// 定义单个的静态物体。
class DllExport CStaticObject : public CBasObject
{
  protected:
    CStockedObjects *stockedObjects;    // 指向所有物体模板的指针(用来生成该物体的模板)
    CPosture m_pst;                     // 物体当前姿态
    CBasObject m_template;              // 物体形状模板

  public:
    CStaticObject(const CStaticObject &other);
    CStaticObject()
    {
        stockedObjects = NULL;
    }

    // 取得对象的引用
    CStaticObject &GetStaticObject() { return *this; }

    // 生成一个副本
    virtual CStaticObject *Duplicate();

    // 设置所有物体模板
    void SetStockedObjects(CStockedObjects *objs);

    // 取得所有物体的模板
    CStockedObjects *GetStockedObjects() { return stockedObjects; }

    // 设置本物体模板
    void SetTemplate(const CBasObject &t);

    // 取得本物体模板
    CBasObject &GetTemplate() { return m_template; }

    // 设置物体姿态
    void SetPosture(const CPosture &pst);

    // 取得物体姿态
    CPosture GetPosture() const { return m_pst; }

    // 重载"="操作符
    void operator=(const CStaticObject &other);

    // 判断该物体是否与给定的直线相交
    virtual bool LineHit(const CLine &ray, CPnt &pt, float &dist) const;

    // 从二进制文件中装入数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
};

///////////////////////////////////////////////////////////////////////////////
// 定义多个静态物体。
class DllExport CStaticObjects : public std::vector<CStaticObject *>
{
  //private:
  public:
    CStockedObjects *stockedObjects;

  public:
    CStaticObjects() { stockedObjects = NULL; }
    ~CStaticObjects();

    // 清除全部物体
    void Clear();

    // 生成新的静态物体
    virtual CStaticObject *NewStaticObject();

    // 设置所有物体模板
    void SetStockedObjects(CStockedObjects *objs);

    // 根据给定的物体名称找到对应物体的序号
    int FindIndexByName(const string &name) const;

    // 重载操作符“+=”，对CStaticObject对象进行添加
    CStaticObjects &operator+=(const CStaticObject &obj);

    // 重载操作符“+=”，对CStaticObjects对象进行添加
    CStaticObjects &operator+=(const CStaticObjects &objs);

    // 删除指定的物体
    void Delete(int index);

    // 取得所有物体名称的列表
    vector<string> GetObjectsNameList();

    // 判断多个物体是否与给定的直线相交
    virtual bool LineHit(const CLine &ray, CPnt &pt, float &dist) const;

    // 从二进制文件中装入数据
    virtual bool LoadBinary(FILE *fp);

    // 将数据写入二进制文件
    virtual bool SaveBinary(FILE *fp);
};
#endif
