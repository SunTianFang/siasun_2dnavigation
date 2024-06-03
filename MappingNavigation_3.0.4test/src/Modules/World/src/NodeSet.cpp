//                               - SETS.CPP -
//
//   Defines the Blocking sets.
//

#include "stdafx.h"
#include "Sets.h"
#include "World.h"

extern CWorld World;

//////////////////////////////////////////////////////////////////////////////
//   Implmentation of class "CNodeSet".

bool CNodeSet::Occupied()
{
   POSITION pos = m_List.GetHeadPosition();
   for (int i = 0; i < m_List.GetCount(); i++)
   {
      USHORT uNode = m_List.GetNext(pos);
      if (World.GetNode(uNode)->Occupied())
         return true;
   }
   return false;
}

/*
void CNodeSet::operator = (CNodeSet& Obj)
{
   m_List.RemoveAll();
   POSITION pos = Obj.m_List.GetHeadPosition();
   for (int i = 0; i < Obj.m_List.GetCount(); i++)
   {
      m_List.AddTail(Obj.m_List.GetNext(pos));
   }
}
*/

//////////////////////////////////////////////////////////////////////////////
//   Implmentation of class "CPathSet".
bool CPathSet::Occupied()
{
   POSITION pos = m_List.GetHeadPosition();
   for (int i = 0; i < m_List.GetCount(); i++)
   {
      USHORT uPath = m_List.GetNext(pos);
      if (World.GetPathPointer(uPath)->Occupied())
         return true;
   }
   return false;
}

/*
void CPathSet::operator = (CPathSet& Obj)
{
   RemoveAll();
   POSITION pos = Obj.GetHeadPosition();
   for (int i = 0; i < Obj.GetCount(); i++)
   {
      AddTail(Obj.GetNext(pos));
   }
}*/