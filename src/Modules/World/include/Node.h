#pragma once

//#define WORLD_FORMAT_VER1_0             // Ver1.0 : Node without heading
#define WORLD_FORMAT_VER2_0               // Ver2.0 : Node with heading
#define WORLD_FORMAT_VER2_1               // Ver2.1 : Node with RF-ID code
#define WORLD_FORMAT_VER3_0               // Ver3.0 : 找地标距离、速度、地标有效宽度
#define WORLD_USE_OFFSET
#define WORLD_USE_MARKOFFSET

#include <stdio.h>

#include "rfid.h"
#include "Geometry.h"
#include "ScrnRef.h"
#include "Tools.h"

#if _MSC_VER >= 1000
	#include "ScrnRef.h"
#endif

// 节点类型宏定义
#define MARK_NODE          BIT(0)      // 该节点是否有地标
#define CONVEYOR_NODE      BIT(1)      // 该节点是否是移载站
#define SPINTURN_NODE      BIT(2)      // 该节点是否是个自旋点
#define CHARGER_NODE       BIT(3)      // 该节点是否是个充电站
#define TEMP_MARK1_NODE    BIT(4)      // 该节点是否是第1类临时地标点
#define TEMP_MARK2_NODE    BIT(5)      // 该节点是否是第2类临时地标点
#define RFID_MARK_NODE     BIT(6)      // 该节点是否有RFID标签
#define PNT_UNKNOWN_NODE   BIT(7)      // 是否是一个无位置信息的节点
#define COM_NODE		   BIT(9)

// The max. number of node is 0xFF00 (65280)
#define MAX_NODE_ID        ((USHORT)0xFF00)

//////////////////////////////////////////////////////////////////////////////
//   "CNode"类的定义.
class DllExport CNode : public CPnt
{
public:
	USHORT m_uId;             // 节点ID号
	USHORT m_uType;           // 节点类型码
	USHORT m_uExtType;        // 节点的扩展类型码
	float  m_fHeading;        // 节点处车头方向
	CRfId  m_Tag;             // 电子标签信息
	float  m_fChkMarkDist;    // 查找地标距离
	float  m_fChkMarkVel;     // 找地标速度
	float  m_fMarkWidth;      // 地标有效宽度

	float  m_fOffset1;
	float  m_fOffset2;

	float  m_fFwdMarkOffset;
	float  m_fBwdMarkOffset;

public:
	// The constructor
	CNode(USHORT uId, CPnt& pt, USHORT uType = MARK_NODE,	USHORT uExtType = 0,	
		float fHeading = 0.0f, UCHAR* pTag = NULL);

	// The default constructor
	CNode()
	{
		m_uId = 0;
		m_uType = MARK_NODE;
		m_uExtType = 0;
		m_fHeading = 0.0f;
		m_Tag = NULL;

		m_fChkMarkDist = 0.0f;
		m_fChkMarkVel = 0.0f;
		m_fMarkWidth = 0.0f;

		m_fFwdMarkOffset = 0.0f;
		m_fBwdMarkOffset = 0.0f;

		m_fOffset1 = 0.0f;
		m_fOffset2 = 0.0f;
	}

	// Check if a mark is available at this node
	bool IsMarkNode();

	// Check if a temporary mark is available at this node
	USHORT IsTempMarkNode();

	// Check if this node is a conveyor station
	bool IsConveyorNode();

	// Check if this node is a spin turn node
	bool IsSpinTurnNode();

	// Check if this node is a charger station
	bool IsChargerNode();

	//  Check if a RFID mark is available at this node
	bool IsRFIDMarkNode();

	bool IsComNode();

	// Get the tag ID of the node
	CRfId GetTag() {return m_Tag;}

	// Set the tag ID for the node
	void SetTag(CRfId& Tag) {m_Tag = Tag;}

	// Verify tag ID
	bool VerifyTag(CRfId& Tag) {return m_Tag == Tag;}

   // overloaded operator "="
   void operator = (const CNode& nd);

   // overloaded operator "=="
	bool operator ==(const CNode& nd) const;

	// overloaded operator "!="
	bool operator !=(const CNode& nd) const;

	// overloaded operator "=="
	bool operator == (USHORT uNodeId);

	// overloaded operator "!="
	bool operator != (USHORT uNodeId);

	// Create the node data from a text file
	bool Create(FILE *StreamIn);

	// Save the node data to a text file
	bool Save(FILE *StreamOut);

	// Archive I/O routine
	friend DllExport CArchive& operator >> (CArchive& ar, CNode& Obj);
	friend DllExport CArchive& operator << (CArchive& ar, CNode& Obj);

#ifdef _MFC_VER
	// Test whether the point is within the node's selection area
	virtual int PointHitTest(CPoint& pnt, CScreenReference& ScrnRef);

	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr);
	void DrawID(CScreenReference& ScrnRef, CDC* pDC, LOGFONT* pLogFont = NULL);
#elif defined QT_VERSION
    // Test whether the point is within the node's selection area
    virtual int PointHitTest(QPoint& pnt, CScreenReference& ScrnRef);

    void Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr);
    void DrawID(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr);
#endif
};
