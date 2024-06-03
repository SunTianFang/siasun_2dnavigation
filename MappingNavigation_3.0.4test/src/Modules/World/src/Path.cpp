//                          - PATH.CPP -
//
//   Implementation of class "CPath" - a class defining a generic path in
//   AGVS map. It is the base class of other path classes.
//
//   Author: Zhang Lei
//   Date:   2000. 10. 28
//
 
#include "stdafx.h"

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "PathBase.h"
#include "Tools.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// Static data member
CNodeBase* CPath::m_pNodeBase = NULL;

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CPath"

CPath::CPath(USHORT uId, USHORT uStartNode, USHORT uEndNode, float fVeloLimit, 
				 USHORT uType, USHORT uGuideType, float fNavParam, USHORT uExtType)
{
	Create(uId, uStartNode, uEndNode, fVeloLimit, uType, uGuideType, fNavParam, uExtType);
}

void CPath::Create(USHORT uId, USHORT uStartNode, USHORT uEndNode, float fVeloLimit,
	USHORT uType, USHORT uGuideType, float fNavParam, USHORT uExtType)
{
	// Make sure the nodes data base is already setup
	assert(m_pNodeBase != NULL);

	m_uId = uId;
	m_uType = uType;
	m_uExtType = uExtType;
	m_uStartNode = uStartNode;
	m_uEndNode = uEndNode;
	m_fVeloLimit[0] = m_fVeloLimit[1] = fVeloLimit;
	m_uGuideType = uGuideType;
	m_fNavParam = fNavParam;
	m_uObstacle = 0;
}

//
//   Get the pointer to the start node.
//
CNode& CPath::GetStartNode()
{
	CNode* pNode = m_pNodeBase->GetNode(m_uStartNode);
	assert(pNode != NULL);
	return *pNode;
}

//
//   Get the pointer to the end node.
//
CNode& CPath::GetEndNode()
{
	CNode* pNode = m_pNodeBase->GetNode(m_uEndNode);
	assert(pNode != NULL);
	return *pNode;
}

//
//   Get the world point of the start node.
//
CPnt& CPath::GetStartPnt()
{
	CNode& nd = GetStartNode();
	return nd.GetPntObject();
}

//
//   Get the world point of the end node.
//
CPnt& CPath::GetEndPnt()
{
	CNode& nd = GetEndNode();
	return nd.GetPntObject();
}
#if 0
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
#endif

//
//    Create the common data of a path (except "m_uType") from a text file.
//
bool CPath::Create(FILE *StreamIn)
{
	unsigned int uId, uStartNode, uEndNode, uGuideType, uExtType;

	if (fscanf(StreamIn, "%u,\t%u,\t%u,\t%f,\t%u",
				  &uId,
				  &uStartNode,
				  &uEndNode,
				  &m_fVeloLimit,
				  &uGuideType) == EOF)
      return false;
	m_uId = uId;
	m_uStartNode = uStartNode;
	m_uEndNode = uEndNode;
   m_uGuideType = uGuideType;

#ifdef WORLD_FORMAT_VER2_0
	if (fscanf(StreamIn, ",\t%u,\t%f", &uExtType, &m_fNavParam) == EOF)
      return false;
	m_uExtType = uExtType;
#endif

	// Success, return true
	return true;
}

//
//    Save: Save the common part of the path data to a text file.
//
bool CPath::Save(FILE *StreamOut)
{
	if (fprintf(StreamOut, "%u,\t%u,\t%u,\t%f,\t%u",
				  m_uId,
				  m_uStartNode,
				  m_uEndNode,
				  m_fVeloLimit,
				  m_uGuideType) == EOF)
      return false;

#ifdef WORLD_FORMAT_VER2_0
	if (fprintf(StreamOut, ",\t%u,\t%f", m_uExtType, m_fNavParam) == EOF)
      return false;
#endif

	// Success, return true
	return true;
}

bool CPath::Create(CArchive& ar)
{
	USHORT uGuideType;
	float fDummy;
	
	// Read the path data
	ar >> m_uId
		>> m_uStartNode
		>> m_uEndNode
		>> m_fVeloLimit[0]
#ifdef WORLD_FORMAT_VER3_0
		>> m_fVeloLimit[1]
#endif
		>> uGuideType;        
	m_uGuideType = uGuideType;

#ifdef WORLD_FORMAT_VER2_0
	ar >> m_uExtType;
	ar >> m_fNavParam;
#endif

	// Success, return true
	return true;
}

bool CPath::Save(CArchive& ar)
{
	// Save the path data
	ar << m_uId
	   << m_uStartNode
		<< m_uEndNode
		<< m_fVeloLimit[0]
#ifdef WORLD_FORMAT_VER3_0
		<< m_fVeloLimit[1]
#endif
		<< m_uGuideType;

#ifdef WORLD_FORMAT_VER2_0
	ar << m_uExtType;
	ar << m_fNavParam;
#endif

	// Success, return true
	return true;
}
