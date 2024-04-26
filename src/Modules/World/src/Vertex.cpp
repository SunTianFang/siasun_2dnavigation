#include "stdafx.h"
#include "Vertex.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CVertexList".

/*
//
//   Get the length of the list
//
USHORT CVertexList::GetCount()
{
   USHORT uCount = 0;
	for (CVertex* p = m_pHead; p != NULL; p = p->pNext)
      uCount++;

	return uCount;
}
*/

//
//   Clear the whole list.
//
void CVertexList::Clear()
{
   CVertex* p;
   while (m_pHead != NULL)
   {
      p = m_pHead;
      m_pHead = m_pHead->pNext;
      delete p;
   }

   m_pHead = NULL;
}

//
//   Search for the specified vertex.
//
CVertex* CVertexList::Search(USHORT uID)
{
	for (CVertex* p = m_pHead; p != NULL; p = p->pNext)
	   if (p->uID == uID)
	      return p;

	return NULL;
}

//
//   Insert a vertex into the list and sort the list at the same time.
//
void CVertexList::SortInsert(CVertex *pSuccessor)
{
	// If the list is empty, just insert it in the head
   if (m_pHead == NULL)
   {
   	m_pHead = pSuccessor;
   	return;
   }

   // insert into successor wrt f
   float f = pSuccessor->f;
   CVertex* p1 = m_pHead;
   CVertex* p2 = m_pHead->pNext;

   while ((p2 != NULL) && (p2->f < f))
   {
      p1 = p2;
      p2 = p2->pNext;
   }

	pSuccessor->pNext = p2;
	p1->pNext = pSuccessor;
}

//
//   Take the first vertex out of the list and return its pointer.
//
CVertex* CVertexList::GetFirst()
{
   CVertex *pVertex = m_pHead;
   m_pHead = m_pHead->pNext;

   return pVertex;
}

//
//   Put a vertex into the list.
//
void CVertexList::Put(CVertex* pVertex)
{
   pVertex->pNext = m_pHead;
   m_pHead = pVertex;
}

