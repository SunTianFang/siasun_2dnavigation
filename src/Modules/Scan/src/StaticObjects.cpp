#include <stdafx.h>
#include <float.h>
#include "StaticObjects.h"

// DQ muban 10.26
#include "LineElement.h"
#include "CircleElement.h"

///////////////////////////////////////////////////////////////////////////////
//   “CStaticObject”类的实现。

CStaticObject::CStaticObject(const CStaticObject &other)
{
    stockedObjects = NULL;
    *this = other;
}

//
//   生成一个副本。
//
CStaticObject *CStaticObject::Duplicate()
{
    return new CStaticObject(*this);
}

//
//   设置所有物体模板。
//
void CStaticObject::SetStockedObjects(CStockedObjects *objs)
{
             std::cout<<"stockedObjects = objs\n";
    stockedObjects = objs;
}

//
//   设置物体模板。
//
void CStaticObject::SetTemplate(const CBasObject &t)
{
    m_template = t;
    GetBasObject() = m_template;
}

//
//   设置物体姿态。
//
void CStaticObject::SetPosture(const CPosture &pst)
{
    m_pst = pst;
    //lishen  ????
    int tmp_nTempId = m_nTempId;       // 该物体的模板编号
    int tmp_nObjId  = m_nObjId;        // 该物体的物体编号
    std::string tmp_name = name;    // 该物体的名称

    GetBasObject() = m_template;

    m_nTempId = tmp_nTempId ;       // 该物体的模板编号
    m_nObjId = tmp_nObjId;        // 该物体的物体编号
    name = tmp_name ;    // 该物体的名称

    InvTransform(pst);
}

//
//   重载"="操作符。
//
void CStaticObject::operator = (const CStaticObject &other)
{
    GetBasObject() = ((CStaticObject&)other).GetBasObject();
    stockedObjects = ((CStaticObject&)other).GetStockedObjects();
    m_pst = other.GetPosture();
    m_template = ((CStaticObject&)other).GetTemplate();

    //lishen
    name = ((CStaticObject&)other).name;
    m_nTempId = ((CStaticObject&)other).m_nTempId;
    m_nObjId = ((CStaticObject&)other).m_nObjId;


}

//
//   判断该物体是否与给定的直线相交。
//
bool CStaticObject::LineHit(const CLine &ray, CPnt &pt, float &dist) const
{
    bool hit = false;
    float minDist = FLT_MAX, d;
    CPnt ptNearest, ptHit;

    // 依次考察各物体元素，看是否与直线有交点
    for (int i = 0; i < (int)size(); i++)
    {
        // 如有交点，记录最近交点
        if (at(i)->LineHit(ray, ptHit, d))
        {
            if (d < minDist)
            {
                minDist = d;
                ptNearest = ptHit;
                hit = true;
            }
        }
    }

    // 如果有相交点，则返回最近点及距离
    if (hit)
    {
        pt = ptNearest;
        dist = minDist;
        return true;
    }
    else
        return false;
}

//
//   从二进制文件中装入数据。
//
bool CStaticObject::LoadBinary(FILE *fp)
{

    // 读取数据之前，必须先已设置过模板库指针
    if (stockedObjects == NULL)
        return false;

    char buf[64] = "";
#if 1
    // 读取物体名称
    // 先读取名称长度
    int nameLen;
    if (fread(&nameLen, sizeof(int), 1, fp) != 1)
        return false;
    std::cout << "By Sam: Load static object nameLen = " << nameLen << std::endl;

    // 再读取名称字串
    if (fread(buf, sizeof(char), nameLen, fp) != nameLen)
        return false;
    buf[nameLen] = '\0';
#endif
    std::cout << "By Sam: Load static object name = " << buf << std::endl;

    // 读取模板号和布署姿态
    int tempId;
    if (fread(&tempId, sizeof(int), 1, fp) != 1)
        return false;
    std::cout << "By Sam: Load static object ID = " << tempId << std::endl;

    float f[3];
    if (fread(f, sizeof(float), 3, fp) != 3)
        return false;
    std::cout << "By Sam: Save static object pst.x = " <<  f[0] <<
                 ", m_pst.y = " <<  f[1] <<
                 ", m_pst.thita = " <<  f[2] << std::endl;

    // 验证模板号的合法性
    if (tempId >= (int)(stockedObjects->size()))
    {
        // dq 模板序号检查失败
        std::cout<<"##############check failed############"<<std::endl;
        return false;
    }

    // 设置模板和姿态
    SetTemplate(stockedObjects->at(tempId));


    CPosture pst(f[0], f[1], f[2]);
    SetPosture(pst);


    // 在此才为名称赋值，因在SetTemplate()、SetPosture()中会复制模板名作为物体名
    name = buf;

    //by lishen
    m_nTempId = tempId;
    m_pst.x = f[0];
    m_pst.y = f[1];
    m_pst.fThita = f[2];


    return true;
}

//
//   将数据写入二进制文件。
//
bool CStaticObject::SaveBinary(FILE *fp)
{
#if 1
    // 写入物体名称
    // 先写入名称长度
    int nameLen = name.length();
    if (fwrite(&nameLen, sizeof(int), 1, fp) != 1)
        return false;

    std::cout << "By Sam: Save static object nameLen = " << nameLen << std::endl;

    // 再写入名称字串
    if (fwrite(name.c_str(), sizeof(char), nameLen, fp) != nameLen)
        return false;
#endif
    std::cout << "By Sam: Save static object name = " << name.c_str() << std::endl;

    // 写入模板号
    if (fwrite(&m_nTempId, sizeof(int), 1, fp) != 1)
        return false;
    std::cout << "By Sam: Save static object ID = " << m_nTempId << std::endl;

    // 写入布署姿态
    float f[3] = { m_pst.x, m_pst.y, m_pst.fThita };
    if (fwrite(f, sizeof(float), 3, fp) != 3)
        return false;
    std::cout << "By Sam: Save static object pst.x = " <<  m_pst.x <<
                 ", m_pst.y = " <<  m_pst.y <<
                 ", m_pst.thita = " <<  m_pst.fThita << std::endl;

    return true;
}

///////////////////////////////////////////////////////////////////////////////
//   “CStaticObjects”类的实现。

CStaticObjects::~CStaticObjects()
{
    Clear();
}

//
//   设置所有物体模板。
//
void CStaticObjects::SetStockedObjects(CStockedObjects *objs)
{
    if (stockedObjects != NULL)
        delete stockedObjects;

    stockedObjects = objs;
}

//
//   根据给定的物体名称找到对应物体的序号。
//
int CStaticObjects::FindIndexByName(const std::string &name) const
{
    for (int i = 0; i < (int)size(); i++)
        if (at(i)->name == name)
            return i;

    return -1;
}

//
//   清除全部物体。
//
void CStaticObjects::Clear()
{
    for (int i = 0; i < (int)size(); i++)
        if (at(i) != NULL)
            delete at(i);

    clear();
}

//
//   生成新的静态物体。
//
CStaticObject *CStaticObjects::NewStaticObject()
{
    return new CStaticObject;
}

CStaticObjects &CStaticObjects::operator+=(const CStaticObject &obj)
{
    push_back(((CStaticObject&)obj).Duplicate());
    back()->m_nObjId = (int)size() - 1;
    return *this;
}

//
//   重载操作符“+=”，对CStaticObjects对象进行添加。
//
CStaticObjects &CStaticObjects::operator+=(const CStaticObjects &objs)
{
    for (int i = 0; i < (int)objs.size(); i++)
        *this += *objs[i];

    return *this;
}

//
//   删除指定的物体。
//
void CStaticObjects::Delete(int index)
{
    if (index >= size())
        return;

    delete(at(index));
    erase(begin() + index);
}

//
//   取得所有物体名称的列表。
//
vector<string> CStaticObjects::GetObjectsNameList()
{
    vector<string> nameList;
    nameList.resize((int)size());

    for (int i = 0; i < (int)size(); i++)
        nameList[i] = at(i)->name;

    return nameList;
}

//
//   判断多个物体是否与给定的直线相交。
//
bool CStaticObjects::LineHit(const CLine &ray, CPnt &pt, float &dist) const
{
    bool hit = false;
    float minDist = FLT_MAX, d;
    CPnt ptNearest, ptHit;

    // 依次考察各物体元素，看是否与直线有交点
    for (int i = 0; i < (int)size(); i++)
    {
        // 如有交点，记录最近交点
        if (at(i)->LineHit(ray, ptHit, d))
        {
            if (d < minDist)
            {
                minDist = d;
                ptNearest = ptHit;
                hit = true;
            }
        }
    }

    // 如果有相交点，则返回最近点及距离
    if (hit)
    {
        pt = ptNearest;
        pt.r = minDist * 1000;
        dist = minDist;
        return true;
    }
    else
        return false;
}

//
//   从二进制文件中装入数据。
//
bool CStaticObjects::LoadBinary(FILE *fp)
{

    // 先从文件中读入模板库部分
    CStockedObjects *temp = new CStockedObjects;
    if (!temp->LoadBinary(fp))
        return false;

/*
 * by dq 不管是否设置过模板库指针，读取时均以FeatureMap中的模板库为准，Mapping软件中读取的yaml文件仅在添加模板时使用
 */
    // 如果尚未设置过模板库指针(在AU10中的运行时情况就是如此)，则启用此模板库
 /*   if (stockedObjects == NULL)
        stockedObjects = temp;
    // 否则，应已设置过模板库(如在Mapping软件中运行时)，此时并不启用刚读入的模板库
    else
    {
        delete temp;
    }
*/
/*
    if (stockedObjects != NULL)
    {      
        delete stockedObjects;    
        stockedObjects = NULL;
    }
*/
    stockedObjects = temp;	

    Clear();

    // 先读入物体数量
    int count;
    if (fread(&count, sizeof(int), 1, fp) != 1)
        return false;

    resize(count);

    // 依次读入并初始化各个静态物体
    for (int i = 0; i < count; i++)
    {
        // 为新物体分配空间
        CStaticObject *obj = NewStaticObject();
        if (obj == NULL)
            return false;

        // 初始化静态物体的模板库指针
        obj->SetStockedObjects(stockedObjects);

        // 再读入该物体的参数，并对其进行初始化
        if (!obj->LoadBinary(fp))
            return false;

        at(i) = obj;
        std::cout << "By Sam Test For " << i << std::endl;
    }

    return true;
}

//
//   将数据写入二进制文件。
//
bool CStaticObjects::SaveBinary(FILE *fp)
{
    // 写入模板库数据
    if (stockedObjects == NULL || !stockedObjects->SaveBinary(fp))
        return false;

    // 写入物体数量
    int count = size();
    if (fwrite(&count, sizeof(int), 1, fp) != 1)
        return false;

    // 依次写入各个静态物体的参数
    for (int i = 0; i < count; i++)
        if (!at(i)->SaveBinary(fp))
            return false;

    return true;
}
