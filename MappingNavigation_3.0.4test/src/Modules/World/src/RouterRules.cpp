#include "stdafx.h"
#include "RouterRules.h"

//////////////////////////////////////////////////////////////////////////////
//   "CRouterRule"类的实现。
//
//   从文件中读入一条路由规则。
//
bool CRouterRule::Create(FILE* fp)
{
	int nFromNode, nToNode;
	if (fscanf(fp, "%d\t%d\n", &nFromNode, &nToNode) == 2)
	{
		if (!m_Route.Create(fp))
			return false;

		m_uFromNode = nFromNode;
		m_uToNode = nToNode;
		return true;
	}
	else
		return false;
}

//////////////////////////////////////////////////////////////////////////////
//   "CRouterRules"类的实现。

CRouterRules::CRouterRules()
{
	m_nCount = 0;
	m_pRules = NULL;
}

CRouterRules::~CRouterRules()
{
	Clear();
}

//
//   清除所有规则。
//
void CRouterRules::Clear()
{
	if (m_pRules != NULL)
		delete []m_pRules;

	m_pRules = NULL;
	m_nCount = 0;
}

//
//   从文件中读入所有路由规则。
//
bool CRouterRules::Create(FILE* fp)
{
	int nCount;
	if (fscanf(fp, "%d", &nCount) == 1)
	{
		m_pRules = new CRouterRule[nCount];
		if (m_pRules == NULL)
			return false;

		for (int i = 0; i < nCount; i++)
			if (!m_pRules[i].Create(fp))
				return false;
		
		m_nCount = nCount;
		return true;
	}
	else
		return false;
}

//
//   搜索从指定起点到指定终点的路由。
//
//   返回值：
//     - NULL: 没找到
//     - 其它：找到的路由对象的指针
//
CRoute* CRouterRules::Find(USHORT uFromNode, USHORT uToNode)
{
	for (int i = 0; i < m_nCount; i++)
	{
		CRouterRule& Rule = m_pRules[i];
		if (Rule.m_uFromNode == uFromNode && Rule.m_uToNode == uToNode)
			return &(Rule.GetRoute());
	}

	return NULL;
}
