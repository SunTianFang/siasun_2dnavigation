#ifndef __CVertex
#define __CVertex

#include "ZTypes.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CVertex".
class CVertex
{
public:
   USHORT uID;               // ID of the node
   float  f;                 // Cost estimation
   float  g;                 // Actual cost
   float  h;                 // Heuristic cost estimation
   CVertex *pParent;         // The parent to this vertex
   CVertex *paChild[8];      // a node may have upto 8+(NULL) children.
   CVertex *pNext;           // for filing purposes

public:
	CVertex() 
	{
		uID = 0xFFFF;
		f = g = h = 0;
		pParent = NULL;
		pNext = NULL;
		for (int i = 0; i < 8; i++)
			paChild[i] = NULL;
	}
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CVertexList".
class CVertexList
{
public:
   CVertex* m_pHead;

public:
   CVertexList()
   {
      m_pHead = NULL;
   }

   ~CVertexList() {Clear();}

   // Check whether the list is empty
   bool Empty() {return (m_pHead == NULL);}

   // Clear the whole list
   void Clear();

   // Search for the specified vertex
   CVertex* Search(USHORT uID);

   // Insert a vertex into the list and sort the list at the same time
   void SortInsert(CVertex *pSuccessor);

   // Take the first vertex out of the list and return its pointer
   CVertex* GetFirst();

   // Put a vertex into the list
   void Put(CVertex* pVertex);
};
#endif
