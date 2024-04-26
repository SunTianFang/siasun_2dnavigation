//                             - SPATH.CPP -
//
//   Author: Zhanglei
//   Date:   2005. 3. 2
//

#include "stdafx.h"
#include "SPath.h"
#include "World.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

CStack Stack;
extern CWorld World;

////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CStack".

CStack::~CStack()
{
   while (m_nCount > 0)
      Pop();
}

void CStack::Push(CVertex *pVertex)
{
   CStackData* p = new CStackData(pVertex, m_Head.m_pNext);
   m_Head.m_pNext = p;
   m_nCount++;
}

CVertex* CStack::Pop()
{
   CStackData* pStackData = m_Head.m_pNext;
	CVertex* pVertex = pStackData->m_pNode;

   m_Head.m_pNext = pStackData->m_pNext;
   delete pStackData;
	m_nCount--;

	return pVertex;
}

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CPathFinder".

CVertex* CPathFinder::ReturnBestVertex()
{
   if (m_OpenList.Empty())
      return NULL;

   // Pick Node with lowest f, in this case it's the first node in list
	//  because we sort the list wrt lowest f. Call it BESTNODE.

	// point to first node on list
   CVertex* p = m_OpenList.GetFirst();

   // Next take BESTNODE (or temp in this case) and put it on CLOSED
   m_ClosedList.Put(p);

   return p;
}

void CPathFinder::GenerateSuccessors(CVertex* pBestVertex, CNode* pNodeTo)
{
	USHORT uBuf[8];
	USHORT uResult = World.GetAllNeighborNodes(pBestVertex->uID, uBuf, 8);

	for (USHORT i = 0; i < uResult; i++)
	{
		CNode* pNodeSucc = World.GetNode(uBuf[i]);
		ASSERT(pNodeSucc != NULL);

		GenerateSucc(pBestVertex, pNodeSucc, pNodeTo);
	}
}

void CPathFinder::GenerateSucc(CVertex* pBestVertex, CNode* pNodeSucc, CNode* pNodeTo)
{
	int i;

	// g(pSuccessor)=g(pBestVertex)+cost of getting from BestNode to Successor
   CNode* pBestNode = World.GetNode(pBestVertex->uID);
//   float f = World.Distance(pBestNode, pNodeSucc);
   float f = World.Cost(pBestNode, pNodeSucc);
   float g = pBestVertex->g + f;

	// identification purposes
   USHORT uSuccID = pNodeSucc->m_uId;

   // Check whether the current node is already in the open list
	CVertex *pOld = m_OpenList.Search(uSuccID);
   if (pOld != NULL) 
   {
      // Add Old to the list of BestNode's Children (or Successors).
      for (i = 0; i < 8; i++)
         if (pBestVertex->paChild[i] == NULL)
	         break;
	   pBestVertex->paChild[i] = pOld;

		// if our new g value is < Old's then reset Old's parent to point to BestNode
	   if (g < pOld->g)  
	   {
	      pOld->pParent = pBestVertex;
	      pOld->g = g;
	      pOld->f = g + pOld->h;

		   CVertex* p = m_OpenList.m_pHead;
         CVertex* temp = p;
         CVertex *q;

		   while (p != NULL && (p->uID != pOld->uID))
		   {
   			q = p;
			   p = p->pNext;
		   }

		   if (p->uID == pOld->uID)
		   {
   			if (p == m_OpenList.m_pHead)
			   {
   				temp = temp->pNext;
				   m_OpenList.m_pHead = temp;
			   }
			   else
   				q->pNext = p->pNext;
		   }

			// Insert Successor on open list wrt f
		   m_OpenList.SortInsert(pOld); 
	   }
   }

	// if equal to NULL then not in open list, else it returns the Node in Old
   else if ((pOld = m_ClosedList.Search(uSuccID)) != NULL) 
   {
		for (i = 0; i < 8; i++)
	      if (pBestVertex->paChild[i] == NULL)           // Add Old to the list of BestNode's Children (or Successors).
	         break;

	   pBestVertex->paChild[i] = pOld;

		// if our new g value is < Old's then reset Old's parent to point to BestNode
	   if (g < pOld->g)  
	   {
	      pOld->pParent = pBestVertex;
	      pOld->g = g;
	      pOld->f = g + pOld->h;
	      PropagateDown(pOld);
         /* Since we changed the g value of Old, we need to propagate this new value downwards, i.e.
			 do a Depth-First traversal of the tree! */
	   }
   }
   else
   {
   	CVertex* pSuccessor = new CVertex;

   	pSuccessor->pParent = pBestVertex;
   	pSuccessor->g = g;
   	pSuccessor->h = World.Distance(pNodeSucc, pNodeTo);
   	pSuccessor->f = g+pSuccessor->h;
   	pSuccessor->uID = uSuccID;
   	m_OpenList.SortInsert(pSuccessor);     /* Insert pSuccessor on open list wrt f */

		for (i = 0; i < 8; i++)
	      if (pBestVertex->paChild[i] == NULL) /* Add Old to the list of BestNode's Children (or Successors). */
   	      break;

	   pBestVertex->paChild[i] = pSuccessor;
   }
}

void CPathFinder::PropagateDown(CVertex* pOld)
{
	int i;
   float g = pOld->g;            // alias.

   for (i = 0; i < 8; i++)
   {
      CVertex *pChild = pOld->paChild[i];                 // create alias for faster access.
      if (pChild == NULL)   
         break;

//		float fDistance = World.Distance(pOld->uID, pChild->uID);
		float fDistance = World.Cost(pOld->uID, pChild->uID);

		if (g + fDistance < pChild->g)
	   {
	      pChild->g = g + fDistance;
	      pChild->f = pChild->g + pChild->h;
	      pChild->pParent = pOld;                 // reset parent to new path.
         Stack.Push(pChild);
	   }     // checked out. Remember the new cost must be propagated down.
   }

   while (!Stack.Empty())
   {
  	   CVertex* pFather = Stack.Pop();
	   for (i = 0; i < 8; i++)
	   {
         CVertex* pChild = pFather->paChild[i];

         // we may stop the propagation 2 ways: either
			if (pChild == NULL)       
	         break;

//			float fDistance = World.Distance(pFather->uID, pChild->uID);
			float fDistance = World.Cost(pFather->uID, pChild->uID);

         // there are no children, or that the g value of the child is equal or better than 
         //   the cost we're propagating
   	   if (pFather->g + fDistance < pChild->g) 
	      {                           
	         pChild->g = pFather->g + fDistance;
	         pChild->f = pChild->g+pChild->h;
	         pChild->pParent = pFather;
            Stack.Push(pChild);
	      }
	   }
   }
}

bool CPathFinder::Solve(USHORT uFromNode, USHORT uToNode, CRoute& Route)
{
   // Get the CNode object pointers
   CNode* pNodeFrom = World.GetNode(uFromNode);
   ASSERT(pNodeFrom != NULL);

   CNode* pNodeTo = World.GetNode(uToNode);
   ASSERT(pNodeTo != NULL);

   // Create the 2 lists (OPEN list & CLOSED list)
   m_OpenList.Clear();
   m_ClosedList.Clear();

   // Create a vertex from the start node
   CVertex* pRoot = new CVertex;
   pRoot->h = World.Distance(pNodeFrom, pNodeTo);
   pRoot->g = 0;
   pRoot->f = pRoot->g + pRoot->h;
   pRoot->uID = uFromNode;
	pRoot->pNext = NULL;

   // Make open List point to the first node
   m_OpenList.Put(pRoot);

   CVertex *pBestVertex;
   for (;;)
   {
   	pBestVertex = (CVertex*)ReturnBestVertex();

      // If the OPEN list is empty, no path is found!
      if (pBestVertex == NULL)
         return false;

      // If we reached the destination node, find it!
	   if (pBestVertex->uID == uToNode)
	      break;

      // Generate successors
      GenerateSuccessors(pBestVertex, pNodeTo);
   }

   // Calculate the size of the shortest path
   Route.Create(pBestVertex, uToNode);

   return true;
}
