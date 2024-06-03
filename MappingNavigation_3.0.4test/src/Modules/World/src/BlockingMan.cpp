#include "stdafx.h"
#include "BlockingMan.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////
//   阻塞规则类。

//
//   从文件中读入一条规则。
//
bool CBlockingRule::Create(FILE* fp)
{
	int nKeyNode;
	fscanf(fp, "%d\n", &nKeyNode);
	m_uKeyNode = (USHORT)nKeyNode;

	return CNodeSet::Create(fp);
}

///////////////////////////////////////////////////////////////////////////////
//   阻塞管理器。

CBlockingManager::CBlockingManager()
{
	m_nCount = 0;
	m_pRules = NULL;
}

CBlockingManager::~CBlockingManager()
{
	if (m_pRules != NULL)
		delete []m_pRules;
}

//
//   从一个文件中读入所有阻塞规则。
//
bool CBlockingManager::Create(FILE* fp)
{
	// 读入规则总数
	fscanf(fp, "%d", &m_nCount);

	// 为所有规则分配空间
	m_pRules = new CBlockingRule[m_nCount];
	if (m_pRules == NULL)
		return false;

	// 依次读入所有规则
	for (int i = 0; i < m_nCount; i++)
	{
		if (!m_pRules[i].Create(fp))
			return false;
	}

	return true;
}

//
//   根据所有阻塞规则，找出被节点uByNode所阻塞的所有节点的集合。
//
bool CBlockingManager::FindBlockedNodes(USHORT uByNode, CNodeSet& BlockedNodes)
{
	bool bResult = false;

	// 先清空结果集合
	BlockedNodes.Clear();

	// 依次分析各规则
	for (int i = 0; i < m_nCount; i++)
	{
		// 如找到关键节点与uByNode一致的规则
		if (m_pRules[i].GetKey() == uByNode)
		{
			// 将被阻塞的所有节点的集合加入到结果集合中
			BlockedNodes.Add(m_pRules[i].GetBlockedNodes());
			bResult = true;
		}
	}

	return bResult;
}

//
//   根据所有阻塞规则，找出被节点集合ByNodes所阻塞的所有节点的集合。
//
bool CBlockingManager::FindBlockedNodes(CNodeSet& ByNodes, CNodeSet& BlockedNodes)
{
	bool bResult = false;

	// 先清空结果集合
	BlockedNodes.Clear();

	POSITION pos = ByNodes.GetHeadPosition();
	for (int i = 0; i < ByNodes.GetCount(); i++)
	{
		CNodeSet s;
		if (FindBlockedNodes(ByNodes.GetNext(pos), s))
		{
			bResult = true;
			BlockedNodes.Add(s);
		}
	}

	return bResult;
}
