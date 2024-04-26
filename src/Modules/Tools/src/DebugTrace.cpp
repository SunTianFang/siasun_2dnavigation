#include "stdafx.h"

void DebugTrace(LPCTSTR pszFormat, ...)
{
#ifdef DEBUG
	va_list pArgs;
	TCHAR szMessageBuffer[16380] = { 0 };
	va_start(pArgs, pszFormat);
	_vsntprintf_s(szMessageBuffer, 16380, pszFormat, pArgs);
	va_end(pArgs);
	OutputDebugString(szMessageBuffer);
#endif
}
