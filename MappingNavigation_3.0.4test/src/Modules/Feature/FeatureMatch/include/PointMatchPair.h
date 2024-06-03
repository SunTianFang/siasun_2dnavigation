#ifndef __CPointMatchPair
#define __CPointMatchPair

#include "Geometry.h"

///////////////////////////////////////////////////////////////////////////////
//   "CPointMatchPair"类的定义。
class CPointMatchPair
{
public:
        CPnt m_ptLocal;               // 在局部坐标系中的点
        CPnt m_ptWorld;               // 在全局坐标系中的点
        CPnt m_ptLocalToWorld;      // 局部坐标点影射到全局坐标系后的位置
        float        m_fMatchDiff;

        vector<CPnt> m_ptLocalSet;

public:
        CPointMatchPair(const CPnt& pnt1, const CPnt& pnt2)
	{
		Create(pnt1, pnt2);
	}

	CPointMatchPair() {}

        void Add(const CPnt& pnt1, const CPnt& pnt2)
        {
            m_ptLocalSet.push_back(pnt1);
            m_ptWorld = pnt2;
            m_fMatchDiff = 0.0f;
        }

        void Create(const CPnt& pnt1, const CPnt& pnt2)
	{
		m_ptLocal = pnt1;
		m_ptWorld = pnt2;
                m_fMatchDiff = 0.0f;
	}

        void Create(CPointMatchPair& pair)
        {
            m_ptLocal = pair.m_ptLocal;
            m_ptWorld = pair.m_ptWorld;
            m_ptLocalToWorld = pair.m_ptLocalToWorld;
            m_fMatchDiff = m_fMatchDiff;
        }


        void operator =(const CPointMatchPair &pair)
        {
            m_ptLocal = pair.m_ptLocal;
            m_ptWorld = pair.m_ptWorld;
            m_ptLocalToWorld = pair.m_ptLocalToWorld;
            m_fMatchDiff = pair.m_fMatchDiff;
        }
	// 根据此点对匹配对生成用于“最小二乘法”的参数
	bool MakeLeastSquareData(float fCoefA[2][4], float fCoefB[2]);
};
#endif
