//                    - TIMEOUTSOCK.H -
//
//   The interface of classes "CTimeOutSocket".
//
//   Author: Lv Xiangren
//   Date:   2013. 06. 22
//

#pragma once

#include <afxsock.h> 
#include "ZTypes.h"
#include "Tools.h"
#include "CSocket.h"
class  CTimeOutSocket : public CSocket
{
public:
	CTimeOutSocket();
	virtual ~CTimeOutSocket() {}
public:
	BOOL SetTimeOut(DWORD uTimeOut);
	BOOL KillTimeOut();
	BOOL GetError() {return m_bError;}

private:
	DWORD  m_dwTimeStart;
	DWORD  m_uTimeOut; 

public:
	BOOL   m_bError;
};

