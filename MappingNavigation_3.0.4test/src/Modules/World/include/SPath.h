//                             - ASTAR.H -
//
//   Author: Zhanglei
//   Date:   2005. 3. 2
//

#ifndef __ASTAR
#define __ASTAR

#include "Vertex.h"
#include "Route.h"
#include "World.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CStack".
class CStackData 
{
public:
   CVertex    *m_pNode;
   CStackData *m_pNext;

public:
   CStackData(CVertex* pNode = NULL, CStackData* pNext = NULL)
   {
      m_pNode = pNode;
      m_pNext = pNext;
   }
};

class CStack
{
public:
   CStackData m_Head;
   int        m_nCount;

public:
   CStack() 
   {
      m_nCount = 0;
   }

   ~CStack();

   void Push(CVertex *pNode);
   CVertex *Pop();

   bool Empty() {return m_nCount == 0;}
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CPathFinder".
class CPathFinder
{
private:
   CVertexList m_OpenList;
   CVertexList m_ClosedList;

private:
   void PropagateDown(CVertex* pOld);
   void GenerateSuccessors(CVertex* BestNode, CNode* pNodeTo);
   void GenerateSucc(CVertex* BestNode, CNode* pNodeSucc, CNode* pNodeTo);
   CVertex* ReturnBestVertex();

public:
   CPathFinder() {}
   ~CPathFinder() {}

   bool Solve(USHORT uFromNode, USHORT uToNode, CRoute& Route);
};
#endif
