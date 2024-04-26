#include <stdafx.h>
#include <float.h>
#include <vector>
#include "LiveObjects.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//   “CObjActionPlan”类的实现。

bool CObjActionPlan::Load(FILE *fp)
{
    int count;
    float x, y, theta;

    if (fscanf(fp, "%d", &count) != 1)
        return false;

    for (int i = 0; i < count; i++)
    {
        int type, stepId;
        if (fscanf(fp, "%d %d", &stepId, &type) != 2)
            return false;

        CObjActionData data;

        // 根据类型处理后续部分
        switch (type)
        {
        case OBJ_ACTION_INIT:
            data.actionType = OBJ_STATIC;

            // 读入状态值
            if (fscanf(fp, "%f %f %f\n", &x, &y, &theta) != 3)
                return false;

            data.stepId = stepId;
            data.param[0] = x;
            data.param[1] = y;
            data.param[2] = theta;

            break;

        case OBJ_ACTION_MOVE:
            data.actionType = OBJ_DYNAMIC;

            // 读入状态值
            if (fscanf(fp, "%f %f %f\n", &x, &y, &theta) != 3)
                return false;

            data.stepId = stepId;
            data.param[0] = x;
            data.param[1] = y;
            data.param[2] = theta;
            break;

        case OBJ_ACTION_STOP:
            data.actionType = OBJ_DYNAMIC;
            data.stepId = stepId;
            data.param[0] = data.param[1] = data.param[2] = 0;
            break;

        case OBJ_ACTION_DISAPPEAR:
            data.actionType = OBJ_NOT_STARTED;
            break;

        default:
            break;
        }

        push_back(data);
    }

    return true;
}

#ifdef USE_YAML
//
//   从YAML节点读取运行计划
//
bool CObjActionPlan::LoadYaml(YAML::Node &yn)
{
    // 取得计划中所含的步数
    int count = 0;
    for (YAML::const_iterator it = yn.begin(); it != yn.end(); it++)
        count++;

    count--;

    for (int j = 0; j < count; j++)
    {
        // 取得对应的action编号串
        std::stringstream str_stream;
        str_stream << j + 1;
        string strAction = "action_" + str_stream.str();

        CObjActionData data;

        // 取得对应于该action的yaml节点
        YAML::Node actionNode = yn[strAction];

        // 取得步编号
        int stepId = actionNode["step"].as<int>();

        // 取得动作串
        string action = actionNode["action"].as<string>();

        // 如果动作为“初始化”,下面获取出现时的姿态
        if (action == "init")
        {
            vector<float> v = actionNode["posture"].as<vector<float>>();

            data.actionType = OBJ_ACTION_INIT;
            data.stepId = stepId;
            data.param[0] = v[0];
            data.param[1] = v[1];
            data.param[2] = v[2];
        }

        // 如果动作为“出现”,下面设置出现状态
        else if (action == "appear")
        {
            data.actionType = OBJ_ACTION_APPEAR;
            data.stepId = stepId;
        }

        // 如果动作为“移动”,下面获取移动的速度
        else if (action == "move")
        {
            vector<float> v = actionNode["velocity"].as<vector<float>>();
            data.actionType = OBJ_ACTION_MOVE;
            data.stepId = stepId;
            data.param[0] = v[0];
            data.param[1] = v[1];
            data.param[2] = v[2];
        }

        // 如果动作为“停止”,下面设置停止状态
        else if (action == "stop")
        {
            data.actionType = OBJ_ACTION_MOVE;
            data.stepId = stepId;
            data.param[0] = 0;
            data.param[1] = 0;
            data.param[2] = 0;
        }

        // 如果动作为“消失”,下面设置消失状态
        else if (action == "disappear")
        {
            data.actionType = OBJ_ACTION_DISAPPEAR;
            data.stepId = stepId;
        }

        push_back(data);
    }

    return true;
}
#endif

///////////////////////////////////////////////////////////////////////////////
//   “CLiveObject”类的实现。

CLiveObject::~CLiveObject()
{
    Clear();
}

void CLiveObject::Clear()
{
    for (int i = 0; i < (int)size(); i++)
        delete at(i);

    clear();
}

//
//   设置物体姿态。
//
void CLiveObject::SetPosture(const CPosture &pst)
{
    m_pst = pst;
    Update();
}

//
//   根据物体模板和物体姿态，重新计算物体的状态。
//
void CLiveObject::Update()
{
    Clear();

    // 先复制模板数据
    for (int i = 0; i < (int)m_template.size(); i++)
        push_back(m_template[i]->Duplicate());

    for (int i = 0; i < (int)size(); i++)
        at(i)->InvTransform(m_pst);
}

//
//   对物体进行移动。
//
void CLiveObject::Move(float x, float y, float theta)
{
    m_pst.x += x;
    m_pst.y += y;
    m_pst.fThita += theta;

    m_pst.fThita = CAngle::NormAngle(m_pst.fThita);

    // 更新物体各元素的位置
    Update();
}

//
//   判断该物体是否与给定的直线相交。
//
bool CLiveObject::LineHit(const CLine &ray, CPnt &pt, float &dist) const
{
    if (m_status == OBJ_NOT_STARTED)
        return false;

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
//   执行运行计划。
//
bool CLiveObject::Run(int stepId)
{
    // 如果还有计划步可执行，则试图执行新的计划
    if (m_curPlan < m_plan.size())
    {
        CObjActionData &action = m_plan[m_curPlan];

        // 数据步编号相符时，才试图执行运行计划
        if (action.stepId == stepId)
        {
            switch (action.actionType)
            {
            // 设定静态姿态
            case OBJ_ACTION_INIT:
                // 取得设定的姿态
                SetPosture(CPosture(action.param[0], action.param[1], action.param[2]));
                m_status = OBJ_STATIC;
                m_curPlan++;
                break;

            // 设定动态速度
            case OBJ_ACTION_MOVE:
                if (m_status == OBJ_NOT_STARTED)
                    return false;    // 必须已启动，否则报错

                dx = action.param[0];
                dy = action.param[1];
                dtheta = action.param[2];

                m_status = OBJ_DYNAMIC;
                m_curPlan++;
                break;

            case OBJ_ACTION_STOP:
                m_status = OBJ_DYNAMIC;
                dx = dy = dtheta = 0;
                m_curPlan++;
                break;

            // 重新出现，不改变原来的姿态
            case OBJ_ACTION_APPEAR:
                m_status = OBJ_STATIC;
                m_curPlan++;
                break;

            case OBJ_ACTION_DISAPPEAR:
                m_status = OBJ_NOT_STARTED;
                m_curPlan++;
                break;

            default:
                break;
            }
        }
    }

    // 如果物体处于运动状态，现在调整它的姿态
    if (m_status == OBJ_DYNAMIC)
    {
        Move(dx, dy, dtheta);
    }

    // 为运行记录分配新空间

    m_records += GetBasObject();

    if (m_status == OBJ_NOT_STARTED)
        m_records.back().SetVisible(false);
    else
        m_records.back().SetVisible(true);

    return true;
}

//
//   从文件中装入单个物体模板。
//
bool CLiveObject::LoadTemplate(FILE *fp)
{
    if (!m_template.LoadText(fp))
        return false;

    return true;
}

//
//   从文件中装入单个物体运行计划。
//
bool CLiveObject::LoadActionPlan(FILE *fp)
{
    // 在此读入物体移动计划
    if (!m_plan.Load(fp))
        return false;

    return true;
}

#ifdef USE_YAML
//
//   从YAML文件中装入物体运行计划。
//
bool CLiveObject::LoadActionPlanYaml(YAML::Node &yn)
{
    if (!m_plan.LoadYaml(yn))
        return false;

    return true;
}
#endif

#ifdef _MFC_VER

void CLiveObject::Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nLineWidth,
                       int stepId)
{
    // 依次画出物体的各个元素
    if (!m_records[stepId].IsVisible())
        return;

    for (int i = 0; i < (int)size(); i++)
    {
        m_records[stepId].at(i)->Plot(ScrnRef, pDC, crColor, nLineWidth);
    }
}

#elif defined QT_VERSION
void CLiveObject::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor,
                       int nLineWidth, int stepId)
{
    // 依次画出物体的各个元素
    if (!m_records[stepId].IsVisible())
        return;

    for (int i = 0; i < (int)size(); i++)
    {
        m_records[stepId].at(i)->Plot(ScrnRef, pPainter, crColor, nLineWidth);
    }
}
#endif

///////////////////////////////////////////////////////////////////////////////

CObjectRec &CObjectRec::operator+=(const CBasObject &obj)
{
    push_back(obj);
    return *this;
}

///////////////////////////////////////////////////////////////////////////////
//   “CLiveObjects”类的实现。

//
//   设置所有物体模板。
//
void CLiveObjects::SetStockedObjects(CStockedObjects *objs)
{
    stockedObjects = objs;

    resize(objs->size());

    for (int i = 0; i < (int)size(); i++)
    {
        at(i).SetTemplate(objs->at(i));
        //		at(i).Update();
    }
}

CLiveObjects &CLiveObjects::operator+=(const CLiveObject &obj)
{
    push_back(obj);
    return *this;
}

//
//   重载操作符“+=”，对CLiveObjects对象进行添加。
//
CLiveObjects &CLiveObjects::operator+=(const CLiveObjects &objs)
{
    for (int i = 0; i < (int)objs.size(); i++)
        push_back(at(i));

    return *this;
}

//
//   判断多个物体是否与给定的直线相交。
//
bool CLiveObjects::LineHit(const CLine &ray, CPnt &pt, float &dist) const
{
    bool hit = false;
    float minDist = FLT_MAX, d;
    CPnt ptNearest, ptHit;

    // 依次考察各物体元素，看是否与直线有交点
    for (int i = 0; i < (int)size(); i++)
    {
        // 如有交点，记录最近交点
        if (at(i).LineHit(ray, ptHit, d))
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
//   执行运行计划。
//
bool CLiveObjects::Run(int stepId)
{
    for (int i = 0; i < (int)size(); i++)
        if (!at(i).Run(stepId))
            return false;

    return true;
}

//
//   从文件中装入多个仿真物体。
//
bool CLiveObjects::Load(FILE *fp)
{
    clear();

    // 先读入物体数量
    int count;
    if (fscanf(fp, "%d", &count) != 1)
        return false;

    resize(count);

    // 依次读入各个物体的运动计划
    for (int i = 0; i < (int)size(); i++)
    {
        if (!at(i).LoadActionPlan(fp))
            return false;
    }

    return true;
}

#ifdef USE_YAML
//
//   从YAML文件中装入多个仿真物体。
//
bool CLiveObjects::LoadYaml(YAML::Node &node)
{
    // 先读入模型的数量
    int count = 0;
    for (YAML::const_iterator it = node.begin(); it != node.end(); it++)
        count++;

    resize(count);

    // 依次读入各个运行计划数据
    for (YAML::const_iterator it = node.begin(); it != node.end(); it++)
    {
        // 先取得计划名，根据计划名取得对应于该计划的yaml节点
        string strPlan = it->first.as<string>();
        YAML::Node plan = node[strPlan];

        string strModelName = plan["model"].as<string>();
        int modelId;

        // 如果是多边形
        modelId = stockedObjects->FindByName(strModelName);
        if (modelId < 0)
            return false;

#if 0
		if (strModelName == "little_square")
			modelId = 0;
		else
			modelId = 1;
#endif

        // 从action节点装入运行计划
        if (!at(modelId).LoadActionPlanYaml(plan))
            return false;
    }

    for (int i = 0; i < count; i++)
        at(i).Update();

    return true;
}
#endif

#ifdef _MFC_VER
void CLiveObjects::Plot(CScreenReference &ScrnRef, CDC *pDC, COLORREF crColor, int nLineWidth,
                        int stepId)
{
    for (int i = 0; i < (int)size(); i++)
        at(i).Plot(ScrnRef, pDC, crColor, nLineWidth, stepId);
}

#elif defined QT_VERSION
void CLiveObjects::Plot(CScreenReference &ScrnRef, QPainter *pPainter, QColor crColor,
                        int nLineWidth, int stepId)
{
    for (int i = 0; i < (int)size(); i++)
        at(i).Plot(ScrnRef, pPainter, crColor, nLineWidth, stepId);
}
#endif
