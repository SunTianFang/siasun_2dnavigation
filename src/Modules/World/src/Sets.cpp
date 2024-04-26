//                               - SETS.CPP -
//
//   Defines the Blocking sets.
//

#include "stdafx.h"
#include "Sets.h"
#include "World.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

extern CWorld World;

//////////////////////////////////////////////////////////////////////////////
//   Implmentation of class "CNodeSet".

bool CNodeSet::Occupied()
{
   POSITION pos = GetHeadPosition();
   for (int i = 0; i < GetCount(); i++)
   {
      USHORT uNode = GetNext(pos);
      if (World.GetNode(uNode)->Occupied())
         return true;
   }
   return false;
}

//
//   从文件中初始化节点集合。
//
bool CNodeSet::Create(FILE* fp)
{
	int nCount, nNode;

	// 读入集合中元素数量
	fscanf(fp, "%d\n", &nCount);

	// 依次读入所有元素
	for (int i = 0; i < nCount; i++)
	{
		fscanf(fp, "%d\t", &nNode);
		Add((USHORT)nNode);
	}

	fscanf(fp, "\n");
	return true;
}

//
//   将集合中所有元素写入一文件中。
//
bool CNodeSet::Save(FILE* fp)
{
	// 读入集合中元素数量
	fprintf(fp, "%d\n", GetCount());

	// 依次写入所有元素
	POSITION pos = GetHeadPosition();
	for (int i = 0; i < GetCount(); i++)
	{
		fprintf(fp, "%d\t", GetNext(pos));
	}

	fprintf(fp, "\n");
	return true;
}

void CNodeSet::Dump()
{
	POSITION pos = GetHeadPosition();
	for (int i = 0; i < GetCount(); i++)
		TRACE("%d\t", GetNext(pos));
	TRACE("\n");
}

//////////////////////////////////////////////////////////////////////////////
//   Implmentation of class "CPathSet".

bool CPathSet::Occupied()
{
   POSITION pos = GetHeadPosition();
   for (int i = 0; i < GetCount(); i++)
   {
      USHORT uPath = GetNext(pos);
      if (World.GetPathPointer(uPath)->Occupied())
         return true;
   }
   return false;
}

bool CPathSet::Create(FILE* fp)
{
	return true;
}

bool CPathSet::Save(FILE* fp)
{
	return true;
}

