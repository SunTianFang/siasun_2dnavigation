#pragma once

#include "Node.h"

//////////////////////////////////////////////////////////////////////////////
//   The interface of class "CNodeBase".
class DllExport CNodeBase
{
private:
	 int NextID();

public:
	USHORT m_uCount;       // The number of nodes in the node base
	CNode* m_paNode;       // The pointer to the nodes array

public:
	// The default constructor
	CNodeBase();

	// The destructor
	~CNodeBase();

	void Clear();

	USHORT GetNodeID(USHORT uIndex) {return m_paNode[uIndex].m_uId;}

	// Get the number of nodes in the node base
   USHORT GetNodeCount() {return m_uCount;}

   // Get the specified node object
	CNode* GetNode(USHORT uId);

	// Check if the code is a valid node ID
	bool IsNode(USHORT uCode);

	// Get the pointer to the node data base object
	CNodeBase& GetNodeBaseObject() {return *this;}

	// Get the X coordinate of the left-most point
	float LeftMost();

	// Get the Y coordinate of the top-most point
	float TopMost();

	// Get the X coordinate of the right-most point
	float RightMost();

	// Get the Y coordinate of the bottom-most point
	float BottomMost();

	// Get the width of the map area
	float Width();

	// Get the height of the map area
	float Height();

	// Get the coordinates of the center point
	CPnt Center();

	// Verify the node tag
	bool VerifyNodeTag(CNode& nd, CRfId& Tag);

	// Find the node with the specified tag ID
	CNode* FindNodeByTag(CRfId& Tag);

	// Create the node base from a text file
	bool Create(FILE *StreamIn);

	// Save the node base to a text file
	bool Save(FILE *StreamOut);

	// Add a node to the nodes data base
	SHORT AddNode(CNode& nd);

	// 在指定位置加入一个新节点
	CNode* AddNode(CPnt& pt);

	// Delete a node from the nodes data base
	SHORT RemoveNode(USHORT uId);

	// Modify the ID of a node
	SHORT ModifyNodeID(USHORT uOldId, USHORT uNewId);

	// Modify the type of a node
	bool ModifyNodeType(USHORT uId, USHORT uNewType);

	// Modify the extended type of a node
	bool ModifyNodeExtType(USHORT uId, SHORT uNewExtType) ;

	// Modify the location of a node
	bool ModifyNodePoint(USHORT uId, float x, float y);

	// Archive I/O routine
	friend CArchive& operator >> (CArchive& ar, CNodeBase& Obj);
	friend CArchive& operator << (CArchive& ar, CNodeBase& Obj);

#ifdef _MFC_VER
	// 测试一个窗口点是否落在哪一个节点上
	int PointHitNodeTest(CPoint& pnt, CScreenReference& ScrnRef);

	virtual void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr);
	virtual void DrawNodeID(CScreenReference& ScrnRef, CDC* pDC, LOGFONT* pLogFont);
#elif defined QT_VERSION

    // 测试一个窗口点是否落在哪一个节点上
    int PointHitNodeTest(QPoint& pnt, CScreenReference& ScrnRef);

    virtual void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr);
    virtual void DrawNodeID(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr);
#endif
};
