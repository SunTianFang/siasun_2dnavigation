#include <stdafx.h>
#include <assert.h>
#include "PathBase.h"
#include "GenericPath.h"

#ifdef QT_VERSION
#include <QPoint>
#include <QPainter>
#endif

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CPathBase".

CPathBase::CPathBase()
{
	// Init the class fields to initial values
	m_uCount = 0;
	m_pPathIdx = NULL;
}

CPathBase::~CPathBase()
{
	CleanUp();
}

//
//   Clean up the memory occupied by the path bank.
//
void CPathBase::CleanUp()
{
	for (USHORT i = 0; i < m_uCount; i++)
	{
		if (m_pPathIdx[i].m_ptr != NULL)
			delete m_pPathIdx[i].m_ptr;
	}

	if (m_pPathIdx != NULL)
		free(m_pPathIdx);

	m_uCount = 0;
	m_pPathIdx = NULL;
}

//
//   Get the pointer to the specified path object (form #1).
//
CPath* CPathBase::GetPathPointer(USHORT uPath)
{
	for (USHORT i = 0; i < m_uCount; i++)
	{
		CPath* pPath = m_pPathIdx[i].m_ptr;

		if (pPath->m_uId == uPath)
			return pPath;
	}

	return NULL;
}

//
//   Get the pointer to the specified path object (form #2).
//
CPath* CPathBase::GetPathPointer(USHORT uNode1, USHORT uNode2)
{
	for (USHORT i = 0; i < m_uCount; i++)
	{
		CPath* pPath = m_pPathIdx[i].m_ptr;

		USHORT uStartNode = pPath->m_uStartNode;
		USHORT uEndNode = pPath->m_uEndNode;

		if ((uNode1 == uStartNode && uNode2 == uEndNode) ||
			 (uNode1 == uEndNode && uNode2 == uStartNode))
			return pPath;
	}

	return NULL;
}

//
//   Get the specified path object (Form #2.A).
//
CPath* CPathBase::GetPathPointer(USHORT uNode1, USHORT uNode2, CMoveDir MoveDir)
{
	for (USHORT i = 0; i < m_uCount; i++)
	{
		CPath* pPath = m_pPathIdx[i].m_ptr;

		USHORT uStartNode = pPath->m_uStartNode;
		USHORT uEndNode = pPath->m_uEndNode;

		if ((MoveDir == FORWARD && uNode1 == uStartNode && uNode2 == uEndNode) ||
			 (MoveDir == BACKWARD && uNode1 == uEndNode && uNode2 == uStartNode))
			return pPath;
	}

	return NULL;
}

//
//   Get the pointer to the specified path object (form #3).
//
CPath* CPathBase::GetPathPointer(CNode& nd1, CNode& nd2)
{
	return GetPathPointer(nd1.m_uId, nd2.m_uId);
}

//
//   Get the specified path object (Form #4).
//
CPath* CPathBase::GetPathPointer(USHORT uNode1, USHORT uNode2, float fAngHeadingAtNode1)
{
	for (USHORT i = 0; i < m_uCount; i++)
	{
		// The pointer to the current path object
		CPath* pPath = m_pPathIdx[i].m_ptr;

		// Get the 2 nodes of the path
		USHORT uStartNode = pPath->m_uStartNode;
		USHORT uEndNode = pPath->m_uEndNode;

		// If the node pairs are the same, find it !
		if (uNode1 == uStartNode && uNode2 == uEndNode)
		{
			CNode& nd = pPath->GetStartNode();
			if (pPath->GetHeading(nd) == CAngle(fAngHeadingAtNode1))
				return pPath;
		}
		else if (uNode1 == uEndNode && uNode2 == uStartNode)
		{
			CNode& nd = pPath->GetEndNode();
			if (pPath->GetHeading(nd) == CAngle(fAngHeadingAtNode1))
				return pPath;
		}
	}

	return GetPathPointer(uNode1, uNode2);
}

// Test whether the specified node pair can form a path
bool CPathBase::IsPath(USHORT uNode1, USHORT uNode2)
{
	return (GetPathPointer(uNode1, uNode2) != NULL);
}

//
//   GetNeighborNode: Get (the ID of) a neighbor node of the specified node.
//
USHORT CPathBase::GetNeighborNode(USHORT uNode)
{
	for (USHORT i = 0; i < m_uCount; i++)
	{
		CPath* pPath = m_pPathIdx[i].m_ptr;

		USHORT uStartNode = pPath->m_uStartNode;
		USHORT uEndNode = pPath->m_uEndNode;

		if (uNode == uStartNode)
			return uEndNode;

		if (uNode == uEndNode)
			return uStartNode;
	}

	return NULLID;
}

//
//   GetAllNeighborNodes: Get (the ID of) a neighbor node of the specified node.
//   Return: The number of neighboring nodes found.
//   
USHORT CPathBase::GetAllNeighborNodes(USHORT uNode, USHORT* pBuf, USHORT uBufLen)
{
	USHORT uFound = 0;
	for (USHORT i = 0; i < m_uCount; i++)
	{
		CPath* pPath = m_pPathIdx[i].m_ptr;

		USHORT uStartNode = pPath->m_uStartNode;
		USHORT uEndNode = pPath->m_uEndNode;

		if (uNode == uStartNode)
			pBuf[uFound++] = uEndNode;

		else if (uNode == uEndNode)
			pBuf[uFound++] = uStartNode;

		if (uFound == uBufLen)
			break;
	}

	return uFound;
}

int CPathBase::NextID()
{
	int nNextID = 0;
	for (int i = 0; i < m_uCount; i++)
	{
		CPath* pPath = m_pPathIdx[i].m_ptr;
		if (pPath->m_uId > nNextID)
			nNextID = pPath->m_uId;
	}
	return (nNextID + 1);
}

//
//   Add a node to the nodes data base.
//
bool CPathBase::AddPath(CPath* pPath)
{
	// Allocate memory for the path indexes
	CPathIndex* pTemp = (CPathIndex*)calloc(m_uCount + 1, sizeof(CPathIndex));
	if (pTemp == NULL)
		return false;

	for (USHORT i = 0; i < m_uCount; i++)
		pTemp[i] = m_pPathIdx[i];

	pTemp[m_uCount++].m_ptr = pPath;

	free(m_pPathIdx);
	m_pPathIdx = pTemp;

	return true;
}

//
//   Delete a node from the nodes data base.
//
bool CPathBase::RemovePath(USHORT uId)
{
	USHORT i;

	if (uId >= m_uCount)
		return true;

	// Allocate memory for the path indexes
	CPathIndex* pTemp = (CPathIndex*)calloc(m_uCount - 1, sizeof(CPathIndex));
	if (pTemp == NULL)
		return false;

	for (i = 0; i < m_uCount; i++)
		if (i != uId)
			pTemp[i] = m_pPathIdx[i];
		else
		{
			delete m_pPathIdx[uId].m_ptr;
#if 0
			USHORT uNode1 = m_pPathIdx[uId].m_ptr->m_uStartNode;
			USHORT uNode2 = m_pPathIdx[uId].m_ptr->m_uEndNode;

			// 如果删除路径后它的节点变为孤立节点，则需要将节点也删除
			if (GetNeighborNode(uNode1) == NULLID)
				CPath::m_pNodeBase->RemoveNode(uNode1);

			if (GetNeighborNode(uNode2) == NULLID)
				CPath::m_pNodeBase->RemoveNode(uNode2);
#endif

			break;
		}

	m_uCount--;
	for (i; i < m_uCount; i++)
		pTemp[i] = m_pPathIdx[i + 1];

	free(m_pPathIdx);
	m_pPathIdx = pTemp;

	return true;
}

#if 0
bool CPathBase::GetPosture(USHORT uFromNode, USHORT uToNode, float fProgress, CPosture& pst)
{
	if (fProgress < 0)
		return false;

	// If the two nodes are the same
	if ((fabs(fProgress) < 1.0E-5) && (uFromNode == uToNode))
	{
		uToNode = GetNeighborNode(uFromNode);
		if (uToNode == NULLID)
			return false;
	}

	CPath* pPath = GetPathPointer(uFromNode, uToNode);
	if (pPath == NULL)
		return false;

	bool bPositiveDir = true;
	if (pPath->m_uStartNode != uFromNode)
		bPositiveDir = false;

	if (fProgress > pPath->Size())
		fProgress = pPath->Size();

	return pPath->GetPosture(fProgress, pst, bPositiveDir);
}
#endif

bool CPathBase::Create(FILE *StreamIn)
{
	int nType;

	// Clear the old path base
	CleanUp();

	// Init the count of paths
	if (fscanf(StreamIn, "%u\n", &m_uCount) == EOF)
		return false;

	// Allocate memory for the path indexes
	m_pPathIdx = (CPathIndex*)calloc(m_uCount, sizeof(CPathIndex));

	assert(m_pPathIdx != NULL);

	for (USHORT i = 0; i < m_uCount; i++)
	{
		CPath* pPath;

      if (fscanf(StreamIn, "%d,\t", &nType) == EOF)
         return false;

		switch(nType)
		{
			case LINE_TYPE:
				pPath = new CLinePath;
				break;

			case ARC_TYPE:
				pPath = new CArcPath;
				break;
								
			case SPP_TYPE:
				pPath = new CSppPath;
				break;
				
			case SPLINE_TYPE:
				pPath = new CSplinePath;
				break;

			case SIDE_TYPE:
				pPath = new CSidePath;
				break;

			case SCP_TYPE:
				pPath = new CScpPath;
				break;
				
			case LAZY_S_TYPE:
				pPath = new CLazySPath;
				break;

			case GENERIC_TYPE:
//				pPath = new CGenericPath;
				break;

			case UNKNOWN_PATH_TYPE:
				pPath = new CUnknownPath;
				break;
				
			default:
				ASSERT(false);
				break;

		}

		assert(pPath != NULL);

		if (!pPath->Create(StreamIn))
			assert(false);

		pPath->m_uType = nType;
		m_pPathIdx[i].m_ptr = pPath;
	}

	return true;
}

bool CPathBase::Save(FILE *StreamOut)
{
	// Save the count of paths
	if (fprintf(StreamOut, "%u\n", m_uCount) == EOF)
		return false;

	for (USHORT i = 0; i < m_uCount; i++)
	{
		CPath* pPath = m_pPathIdx[i].m_ptr;

		if (fprintf(StreamOut, "%u,\t", pPath->m_uType) == EOF)
			return false;

		if (!pPath->Save(StreamOut))
			assert(false);
	}
	return true;
}

CArchive& operator >> (CArchive& ar, CPathBase& Obj)
{
	// Clear the old path base
	Obj.CleanUp();

	// Init the count of paths
	ar >> Obj.m_uCount;

	// Allocate memory for the path indexes
	Obj.m_pPathIdx = (CPathIndex*)calloc(Obj.m_uCount, sizeof(CPathIndex));

	assert(Obj.m_pPathIdx != NULL);

	for (USHORT i = 0; i < Obj.m_uCount; i++)
	{
		CPath* pPath;
		USHORT uType;

		ar >> uType;

		switch (uType)
		{
			case LINE_TYPE:
				pPath = new CLinePath;
				break;

			case ARC_TYPE:
				pPath = new CArcPath;
				break;
								
			case SPP_TYPE:
				pPath = new CSppPath;
				break;
				
			case SPLINE_TYPE:
				pPath = new CSplinePath;
				break;

			case SIDE_TYPE:
				pPath = new CSidePath;
				break;

			case SCP_TYPE:
				pPath = new CScpPath;
				break;
				
			case LAZY_S_TYPE:
				pPath = new CLazySPath;
				break;

			case GENERIC_TYPE:
				pPath = new CGenericPath;
				break;

			case UNKNOWN_PATH_TYPE:
				pPath = new CUnknownPath;
				break;
				
			default:
				ASSERT(false);
				break;
		}

		assert(pPath != NULL);

		if (!pPath->Create(ar))
			assert(false);

		pPath->m_uType = uType;

#if 1
		if (uType == SPP_TYPE)
		{
			CArcPath* p = (CArcPath*)pPath;

			pPath = new CGenericPath;
			((CGenericPath*)pPath)->CreateFromArcPath(*p);
			pPath->m_uType = GENERIC_TYPE;

			delete p;
		}
#endif

		Obj.m_pPathIdx[i].m_ptr = pPath;
	}

	return ar;
}

CArchive& operator << (CArchive& ar, CPathBase& Obj)
{
	// Save the count of paths
	ar << Obj.m_uCount;

	for (USHORT i = 0; i < Obj.m_uCount; i++)
	{
		CPath* pPath = Obj.m_pPathIdx[i].m_ptr;

		ar << pPath->m_uType;

		if (!pPath->Save(ar))
			assert(false);
	}
	return ar;
}

#ifdef _MFC_VER
//
//   测试一个窗口点是否落在哪一个路径上。
//   参数：
//       nPathType < 0 : 检测所有类型路径
//       nPathType >= 0: 仅检测这种规定类型的路径
//
//   返回值：
//      >= 0: 给定的点落在路径上的编号
//       < 0: 未落到任何一个路径上
//
//   nHitType:
//     在落到曲线上的情况下，nHitType == 0, 表示落到曲线上
//                           nHitType > 0, 个示落到(序号为nHitType-1的)控制点处
//
int CPathBase::PointHitPath(CPoint& pnt, CScreenReference& ScrnRef, int& nHitType, int nPathType)
{
	// 依次对每个路径进行判断
	for (int i = 0; i < m_uCount; i++)
	{
		CPath* pPath = m_pPathIdx[i].m_ptr;

		if (nPathType >= 0 && pPath->m_uType != nPathType)
			continue;

		int _nHitType = pPath->PointHitTest(pnt, ScrnRef);

		if (_nHitType >= 0)
		{
			nHitType = _nHitType;           // 0:曲线上， >0:控制点处
			return i;                       // 曲线的序号
		}
	}

	return -1;
}

void CPathBase::Draw(CScreenReference& ScrnRef, CDC* pDc, COLORREF crPath)
{
	for (USHORT i = 0; i < m_uCount; i++)
	{
		CPath* pPath = m_pPathIdx[i].m_ptr;
		pPath->Draw(ScrnRef, pDc, crPath);
	}
}

void CPathBase::SetColor(COLORREF clr)
{
	for (USHORT i = 0; i < m_uCount; i++)
	{
		CPath* pPath = m_pPathIdx[i].m_ptr;
		pPath->SetColor(clr);
	}
}

#elif defined QT_VERSION

// 测试一个窗口点是否落在哪一个路径上
int CPathBase::PointHitPath(QPoint& pnt, CScreenReference& ScrnRef, int& nHitType, int nPathType)
{
	// 依次对每个路径进行判断
	for (int i = 0; i < m_uCount; i++)
	{
		CPath* pPath = m_pPathIdx[i].m_ptr;

		if (nPathType >= 0 && pPath->m_uType != nPathType)
			continue;

		int _nHitType = pPath->PointHitTest(pnt, ScrnRef);

		if (_nHitType >= 0)
		{
			nHitType = _nHitType;           // 0:曲线上， >0:控制点处
			return i;                       // 曲线的序号
		}
	}

	return -1;
}

void CPathBase::Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crPath)
{
    for (USHORT i = 0; i < m_uCount; i++)
    {
        CPath* pPath = m_pPathIdx[i].m_ptr;
        pPath->Draw(ScrnRef, pPainter, crPath);
    }
}
#endif
