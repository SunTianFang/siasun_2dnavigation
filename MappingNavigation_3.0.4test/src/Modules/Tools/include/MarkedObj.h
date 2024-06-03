#pragma once

class CMarkedObj
{
private:
	int m_nMark;
	
public:
	CMarkedObj() {m_nMark = 0;}
	int GetMark() {return m_nMark;} 
	void SetMark(int nMark) {m_nMark = nMark;}
};
