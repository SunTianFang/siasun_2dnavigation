#ifndef __CTimeStamp
#define __CTimeStamp

#include "time.h"
#include"Tools.h"

//
//   定义“时间戳”。
//
class CTimeStamp
{
  public:
    unsigned long long m_dwTimeStamp;

public:
    CTimeStamp()
    {
        m_dwTimeStamp = GetTickCount();
    }
    CTimeStamp(unsigned long long uTime)
    {
        m_dwTimeStamp = uTime;
    }
    CTimeStamp(const CTimeStamp &tmsp)
    {
        m_dwTimeStamp = tmsp.m_dwTimeStamp;
    }
    unsigned long long GetTimeStamp()
    {
        return m_dwTimeStamp;
    }
    // Update the time-stamp
    void Update()
    {
        m_dwTimeStamp = GetTickCount();
    }
    // Check whether the time-stamp is up-to-date
    BOOL IsNew(unsigned long long dwTimeDiff = 500)
    {
        return (GetTickCount() - m_dwTimeStamp < dwTimeDiff);
    }

    void Stamp(unsigned long long uTime)
    {
        m_dwTimeStamp = uTime;
    }
    unsigned long long TimeDiffSince(const CTimeStamp &other)
    {
        return m_dwTimeStamp - other.m_dwTimeStamp;
    }
};

//
//   Information for BOOL data.
//
class CBoolDataInfo : public CTimeStamp
{
public:
	BOOL  m_bData;

public:
	CBoolDataInfo()
	{
		m_bData = FALSE;
	}

	void Update(BOOL bData)
	{
		m_bData = bData;
		CTimeStamp::Update();
	}
};

//
//   Information for Byte data.
//
class CByteDataInfo : public CTimeStamp
{
public:
        unsigned char  m_bData;

public:
	CByteDataInfo()
	{
		m_bData = 0;
	}

        void Update(unsigned char bData)
	{
		m_bData = bData;
		CTimeStamp::Update();
	}
};

//
//   Information for integer data.
//
class CIntDataInfo : public CTimeStamp
{
public:
	int  m_nData;

public:
	CIntDataInfo()
	{
		m_nData = 0;
	}

	void Update(int nData)
	{
		m_nData = nData;
		CTimeStamp::Update();
	}
};

//
//   Information for long data.
//
class CLongDataInfo : public CTimeStamp
{
public:
	LONG  m_lData;

public:
	CLongDataInfo()
	{
		m_lData = 0;
	}

	void Update(LONG lData)
	{
		m_lData = lData;
		CTimeStamp::Update();
	}
};

//
//   Information for long data.
//
class CULongDataInfo : public CTimeStamp
{
public:
	ULONG  m_ulData;

public:
	CULongDataInfo()
	{
		m_ulData = 0;
	}

	void Update(ULONG ulData)
	{
		m_ulData = ulData;
		CTimeStamp::Update();
	}
};

//
//   Information for position/velocity data.
//
class CFloatDataInfo : public CTimeStamp
{
public:
	float  m_fData;

public:
	CFloatDataInfo()
	{
		m_fData = 0;
	}

    CFloatDataInfo(const CFloatDataInfo& other)
    {
        this->m_fData = other.m_fData;
        this->m_dwTimeStamp = other.m_dwTimeStamp;
    }

    CFloatDataInfo& operator = (const CFloatDataInfo& Obj)
    {
        this->m_fData = Obj.m_fData;
        this->m_dwTimeStamp = Obj.m_dwTimeStamp;
        return *this;
    }

	void Update(float fData)
	{
		m_fData = fData;
		CTimeStamp::Update();
	}

    void clear()
    {
        m_fData = 0;
        m_dwTimeStamp = 0;
    }
};

//
//   Information for ADC data.
//
class CWordDataInfo : public CTimeStamp
{
public:
	WORD m_uData;

public:
	CWordDataInfo()
	{
		m_uData = 0;
	}

	void Update(WORD uData)
	{
		m_uData = uData;
		CTimeStamp::Update();
	}
};
class CShortDataInfo : public CTimeStamp
{
public:
        SHORT m_uData;

public:
        CShortDataInfo()
        {
                m_uData = 0;
        }

        void Update(SHORT uData)
        {
                m_uData = uData;
                CTimeStamp::Update();
        }
};
//
//   Information for parameter data.
//
class CGenericDataInfo : public CTimeStamp
{
public:
	UCHAR m_uchLen;
	WORD  m_uIndex;
	UCHAR m_uchBuf[4];

public:
	CGenericDataInfo()
	{
		m_uchLen = 0;
		m_uIndex = 0;
	}

	void Update(UCHAR uchLen, WORD uIndex, UCHAR* pBuf)
	{
		ASSERT(uchLen <= 4);
		if (uchLen > 4)
			return;

		m_uchLen = uchLen;
		m_uIndex = uIndex;
		for (int i = 0; i < uchLen; i++)
			m_uchBuf[i] = *pBuf++;

		CTimeStamp::Update();
	}
};

class CSpecialWordDataInfo
{
private:
	WORD m_uData;
	BOOL m_bAvailable;

public:
	CSpecialWordDataInfo()
	{
		m_uData = 0;
		m_bAvailable = FALSE;
	}

	void Update(WORD uData)
	{
		m_uData = uData;
		m_bAvailable = TRUE;
	}

	BOOL IsAvailable()
	{
		return m_bAvailable;
	}

	BOOL GetData(WORD& uData)
	{
		if(m_bAvailable)
		{
			m_bAvailable = FALSE;
			uData =  m_uData;
			return TRUE;
		}
		else
			return FALSE;
	}
};

class CStringDataInfo : public CTimeStamp
{
public:
	TCHAR m_chBuf[64];

	CStringDataInfo()
	{
		_tcsncpy(m_chBuf, _T(""), 64);
	}

	void Update(TCHAR* str)
	{
		_tcsncpy(m_chBuf, str, 64);
		CTimeStamp::Update();
	}
};
#endif
