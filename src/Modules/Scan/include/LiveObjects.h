#ifndef __CLiveObjects
#define __CLiveObjects

#ifdef USE_YAML
#include "yaml-cpp/yaml.h"
#endif

#include "StockedObjects.h"

#ifdef QT_VERSION
#include <QColor>
#include <QPainter>
#endif

// 指令分类
#define OBJ_ACTION_INIT 1
#define OBJ_ACTION_MOVE 2
#define OBJ_ACTION_STOP 3
#define OBJ_ACTION_APPEAR 4
#define OBJ_ACTION_DISAPPEAR 0

// 状态分类
enum { OBJ_NOT_STARTED, OBJ_STATIC, OBJ_DYNAMIC };

struct CObjActionData
{
    int stepId;
    int actionType;    // 1-物体出现; 2-物体运动; 3-物体停止; 0-物体消失
    float param[3];    // 三个浮点数参数
};

class CObjActionPlan : public std::vector<CObjActionData>
{
  public:
    CObjActionPlan() {}
    bool Load(FILE *fp);

#ifdef USE_YAML
    bool LoadYaml(YAML::Node &yn);
#endif
};

// 定义物体运行记录类型
class CObjectRec : public std::vector<CBasObject>
{
  private:
    bool visible;

  public:
    CObjectRec() { visible = true; }
    CObjectRec &operator+=(const CBasObject &obj);

    void SetVisible(bool v) { visible = v; }
    bool IsVisible() { return visible; }
};

///////////////////////////////////////////////////////////////////////////////
// 定义单个物体的运行脚本。
class CLiveObject : public CBasObject
{
  private:
    CPosture m_pst;           // 物体当前姿态
    CBasObject m_template;    // 物体形状模板
    CObjActionPlan m_plan;    // 物体运动计划
    int m_curPlan;            // 当前的运行计划的执行位置
    int m_curStep;            // 当前执行的步编号
    int m_status;             // 物体状态：0-未启动, 1-静止; 2-运动
    float dx;                 // 物体的运行速度x
    float dy;                 // 物体的运行速度y
    float dtheta;             // 物体的运行速度theta
    CObjectRec m_records;

  private:
    void Clear();

    // 使物体移动
    void Move(float x, float y, float theta);

  public:
    CLiveObject(const CBasObject &ot) : m_pst(0, 0, 0)
    {
        m_template = ot;
        m_status = OBJ_NOT_STARTED;
        m_curPlan = 0;
        m_curStep = 0;
        dx = dy = dtheta = 0;
    }

    CLiveObject()
    {
        m_status = OBJ_NOT_STARTED;
        m_curPlan = 0;
        m_curStep = 0;
        dx = dy = dtheta = 0;
    }

    ~CLiveObject();

    // 设置物体模板
    void SetTemplate(const CBasObject &ot) { m_template = ot; }

    // 设置物体姿态
    void SetPosture(const CPosture &pst);

    // 取得物体姿态
    CPosture GetPosture() const { return m_pst; }

    // 判断该物体是否与给定的直线相交
    virtual bool LineHit(const CLine &ray, CPnt &pt, float &dist) const;

    // 执行运行计划
    bool Run(int stepId);

    // 从文件中装入物体模板
    bool LoadTemplate(FILE *fp);

    // 从文件中装入物体运行计划
    bool LoadActionPlan(FILE *fp);

#ifdef USE_YAML
    // 从YAML文件中装入物体运行计划
    bool LoadActionPlanYaml(YAML::Node &yn);
#endif

    // 根据物体模板和物体姿态，重新计算物体的状态
    void Update();

#ifdef _MFC_VER
    void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nLineWidth, int stepId);
#elif defined QT_VERSION
    void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor, int nLineWidth, int stepId);
#endif
};

///////////////////////////////////////////////////////////////////////////////
// 定义多个仿真物体。
class CLiveObjects : public std::vector<CLiveObject>
{
  private:
    CStockedObjects *stockedObjects;

  public:
    CLiveObjects() {}

    // 设置所有物体模板
    void SetStockedObjects(CStockedObjects *objs);

    // 重载操作符“+=”，对CLiveObject对象进行添加
    CLiveObjects &operator+=(const CLiveObject &obj);

    // 重载操作符“+=”，对CLiveObjects对象进行添加
    CLiveObjects &operator+=(const CLiveObjects &objs);

    // 判断多个物体是否与给定的直线相交
    virtual bool LineHit(const CLine &ray, CPnt &pt, float &dist) const;

    // 执行运行计划
    bool Run(int stepId);

    // 从文件中装入多个仿真物体
    bool Load(FILE *fp);

#ifdef USE_YAML
    // 从YAML文件中装入多个仿真物体
    bool LoadYaml(YAML::Node &node);
#endif

#ifdef _MFC_VER
    void Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nLineWidth, int stepId);
#elif defined QT_VERSION
    void Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor, int nLineWidth, int stepId);
#endif
};
#endif
