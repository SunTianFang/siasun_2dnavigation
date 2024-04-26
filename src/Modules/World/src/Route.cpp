#include "stdafx.h"
#include "Route.h"
#include "World.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

extern CWorld World;

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CRoute".

CRoute::~CRoute()
{
   Clear();
}

//
//   路由的拷贝。
//
void CRoute::operator = (CRoute& Obj)
{
   Clear();
   Create(Obj.m_pNodeBuf, Obj.m_nCount);
}

//
//    清空路由。
//
void CRoute::Clear()
{
   if (m_pNodeBuf != NULL)
   {
      delete []m_pNodeBuf;
      m_pNodeBuf = NULL;
      m_nCount = 0;
   }
}

bool CRoute::Create(USHORT* pBuf, int nCount)
{
	Clear();
   m_pNodeBuf = new USHORT[nCount];
   if (m_pNodeBuf == NULL)
      return false;

   for (int i = 0; i < nCount; i++)
      m_pNodeBuf[i] = pBuf[i];

   m_nCount = nCount;
   return true;
}

bool CRoute::Create(CVertex* pVertex, USHORT uEndNode)
{
	Clear();

   // First, caculate the number of vertexes
   int nVertexCount = 0;
   CVertex* p = pVertex;
   while (p != NULL)
   {
      nVertexCount++;
      p = p->pParent;
   }

   m_pNodeBuf = new USHORT[nVertexCount];
   if (m_pNodeBuf == NULL)
      return false;

   for (int i = nVertexCount-1; i >= 0; i--)
   {
      m_pNodeBuf[i] = pVertex->uID;
      pVertex = pVertex->pParent;
   }
   m_nCount = nVertexCount;

   return true;
}

//
//   从文本文件读取一个路由。
//
bool CRoute::Create(FILE* fp)
{
	int nCount;

	Clear();

	if (fscanf(fp, "%d\n", &nCount) != 1)
		return false;

   m_pNodeBuf = new USHORT[nCount];
   if (m_pNodeBuf == NULL)
      return false;

   for (int i = 0; i < nCount; i++)
	{
		if (fscanf(fp, "%d\t", &(m_pNodeBuf[i])) != 1)
			return false;
	}
	fscanf(fp, "\n");

   m_nCount = nCount;
   return true;
}

//
//   Get the start node ID.
//
USHORT CRoute::GetStartNode()
{
   if (m_pNodeBuf == NULL)
      return (USHORT)-1;
   else
      return m_pNodeBuf[0];
}

//
//   Get the end node ID.
//
USHORT CRoute::GetEndNode()
{
   if (m_pNodeBuf == NULL)
      return (USHORT)-1;
   else
      return m_pNodeBuf[m_nCount-1];
}

//
//   Get the cost of the route.
//
float CRoute::Cost()
{
   float fSum = 0;
   for (int i = 1; i < m_nCount; i++)
   {
      USHORT uNode1 = m_pNodeBuf[i-1];
      USHORT uNode2 = m_pNodeBuf[i];
      CPath* pPath = World.GetPathPointer(uNode1, uNode2);
      ASSERT(pPath != NULL);

      fSum += pPath->Cost();
   }
   return fSum;
}
