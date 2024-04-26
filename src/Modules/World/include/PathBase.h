#pragma once

#include "Path.h"
#include "LinePath.h"
#include "ArcPath.h"
#include "SppPath.h"
#include "SplinePath.h"
#include "LazySPath.h"
#include "UnknownPath.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CPathIndex", which defines the content of a
//   path index entry. It is a helper class for "CPathBase".
class DllExport CPathIndex
{
public:
	CPath* m_ptr;                 // Pointer to the path data

public:
	// The default constructor
	CPathIndex() { m_ptr = NULL; }
};

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CPathBase", which organises all the paths
//   into a single class.
class DllExport CPathBase
{
private:
	// Clean up the memory occupied by the path data base
	void CleanUp();

protected:
	int NextID();

public:
	CPathIndex *m_pPathIdx;        // Head pointer to the indexes
	USHORT m_uCount;               // Total number of paths

public:
	// The constructor
	CPathBase();

	// The destructor
	~CPathBase();

	// Get the specified path object (Form #1)
	CPath* GetPathPointer(USHORT uPath);

	// Get the specified path object (Form #2)
	CPath* GetPathPointer(USHORT uNode1, USHORT uNode2);

	// Get the specified path object (Form #2.A)
	CPath* GetPathPointer(USHORT uNode1, USHORT uNode2, CMoveDir MoveDir);

	// Get the specified path object (Form #3)
	CPath* GetPathPointer(CNode& nd1, CNode& nd2);

	// Get the specified path object (Form #4)
	CPath* GetPathPointer(USHORT uNode1, USHORT uNode2, float fAngHeadingAtNode1);

	// Test whether the specified node pair can form a path
	bool IsPath(USHORT uNode1, USHORT uNode2);

	// Find the vehicle's posture at uNode
	USHORT GetNeighborNode(USHORT uNode);

	// Find all the neighboring nodes
	USHORT GetAllNeighborNodes(USHORT uNode, USHORT* pBuf, USHORT uBufLen);

	// Get the pointer to the object
	CPathBase* GetPathBaseObject() { return this; }

	bool GetPosture(USHORT uFromNode, USHORT uToNode, float fProgress, CPosture& pst);
	bool Create(FILE *StreamIn);
	bool Save(FILE *StreamOut);

	// Add a node to the nodes data base
	bool AddPath(CPath* nd);

	// Delete a node from the nodes data base
	bool RemovePath(USHORT uId);

	friend DllExport CArchive& operator >> (CArchive& ar, CPathBase& Obj);
	friend DllExport CArchive& operator << (CArchive& ar, CPathBase& Obj);

#ifdef _MFC_VER
	// 测试一个窗口点是否落在哪一个路径上
	int PointHitPath(CPoint& pnt, CScreenReference& ScrnRef, int& nHitType, int nPathType = -1);

	virtual void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crPath);

	void SetColor(COLORREF clr);
#elif defined QT_VERSION
    // 测试一个窗口点是否落在哪一个路径上
    int PointHitPath(QPoint& pnt, CScreenReference& ScrnRef, int& nHitType, int nPathType = -1);

    virtual void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crPath);
#endif
};
