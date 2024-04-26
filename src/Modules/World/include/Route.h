#ifndef __CRoute
#define __CRoute

#include "Vertex.h"

//////////////////////////////////////////////////////////////////////////////
//   "CRoute"类的接口。该类定义了“路由”的概念。
class CRoute
{
public:
   USHORT* m_pNodeBuf;                 // 节点缓冲区指针
   int     m_nCount;                   // 路由中的节点数量

public:
   CRoute() 
   {
      m_pNodeBuf = NULL;
      m_nCount = 0;
   }

   ~CRoute();

   void operator = (CRoute& Obj);

	// 取得路由节点的数量
	int GetCount() {return m_nCount;}

	// clear the route
   void Clear();

   bool Create(USHORT* pBuf, int nCount);

   bool Create(CVertex* pVertex, USHORT uStartNode);

	// 从文本文件读取一个路由
	bool Create(FILE* fp);

	// Get the start node ID
   USHORT GetStartNode();

   // Get the end node ID
   USHORT GetEndNode();

   // Get the cost of the route
   float Cost();
};
#endif
