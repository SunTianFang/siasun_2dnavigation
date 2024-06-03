#ifndef __CScanMatcher
#define __CScanMatcher

// 当前姿态的三种状态
#define POS_STATUS_UNKNOWN              0           // 姿态完全未知
#define POS_STATUS_KNOWN_ROUGHLY        1           // 姿态粗略知道
#define POS_STATUS_KNOWN                2           // 姿态已知

#include "Scan.h"

class CScan;


///////////////////////////////////////////////////////////////////////////////
//   定义通用的“二维扫描匹配”框架。
class CScanMatcher
{
protected:
	int      m_nScanCount;          // 扫描点的数量
	float    m_fAngReso;            // 扫描器的角分辨率
	float    m_fStartAng;           // 扫描起始角
	float    m_fEndAng;             // 扫描终止角
	float    m_fMaxRange;           // 最大扫描距离

	int      m_nPosStatus;          // 位置状态: 0 - 完全未知; 1-粗略知道; 2-已知
        CPosture m_pstOdRe;
        CPosture m_pstOld;              // 上一周期时的姿态
        CPosture m_pstCur;              // 当前姿态
        CPosture m_pstEstimate;         // 估算出的姿态
        int d_FirstMatchTick;

public:
	CScanMatcher()
	{
		m_nScanCount = 0;
		m_fAngReso = 0;
		m_fStartAng = m_fEndAng = 0;
		m_fMaxRange = 0;
		m_nPosStatus = POS_STATUS_UNKNOWN;
	}

	// 设置激光扫描器参数
	virtual bool SetScannerParam(float fStartAng, float fEndAng, float fAngReso, float fMaxRange)
	{
		m_fStartAng = fStartAng;
		m_fEndAng = fEndAng;
		m_fAngReso = fAngReso;
		m_fMaxRange = fMaxRange;
		m_nScanCount = (int)((m_fEndAng - m_fStartAng) / m_fAngReso + 0.5f);
		
		return true;
	}

        virtual void SetInitPosture(const CPosture& pst, bool bRough = false)
	{
		m_pstOld = m_pstCur = m_pstEstimate = pst;
		
		if (bRough)
		{
			m_nPosStatus = POS_STATUS_KNOWN_ROUGHLY;
		}
		else
			m_nPosStatus = POS_STATUS_KNOWN;
	}

	// 根据里程计设置姿态
        virtual void SetOdometricPosture(const CPosture& pst)
	{
	}

	// 设置经匹配运算校正后的姿态
        virtual void SetCorrectedPosture(const CPosture& pst)
	{
		m_nPosStatus = POS_STATUS_KNOWN;
	}

	// 提供匹配功能(成功时返回1，失败时返回负值)
        virtual int LocalizationPro(CScan* pCurScan, CPosture& pstNew, CScan* pRefScan){return 0;}

        virtual void ResetFirstMatch(const CPosture& pos)
        {
            d_FirstMatchTick = 0;
            m_pstCur = m_pstOld = pos;
        }
        virtual int GetMatchCount()
        {
            return 0;
        }

        virtual float GetMatchRatio()
        {
            return 0;
        }
};

#endif
