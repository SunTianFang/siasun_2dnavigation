#ifndef __CUnorderedPair
#define __CUnorderedPair

///////////////////////////////////////////////////////////////////////////////
//   定义“无序数据对”类。
class CUnorderedPair
{
public:
	int m_nId1;              // 第一个数据
	int m_nId2;              // 第二个数据

public:
	CUnorderedPair(int nId1, int nId2)
	{
		m_nId1 = nId1;
		m_nId2 = nId2;
	}

	CUnorderedPair(const CUnorderedPair& pair)
	{
		m_nId1 = pair.m_nId1;
		m_nId2 = pair.m_nId2;
	}

	CUnorderedPair()
	{
		m_nId1 = m_nId2 = 0;
	}

	// 判断两个“数据对”是否相等
	bool operator == (CUnorderedPair& pair)
	{
		if ((m_nId1 == pair.m_nId1 && m_nId2 == pair.m_nId2) ||
			(m_nId1 == pair.m_nId2 && m_nId2 == pair.m_nId1))
			return true;
		else
			return false;
	}

	// 判断两个“数据对”是否不相等
	bool operator != (CUnorderedPair& pair)
	{
		if (*this == pair)
			return false;
		else
			return true;
	}
};
#endif
