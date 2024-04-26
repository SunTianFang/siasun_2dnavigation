/*
 * Archive.h
 *
 *  Created on: 2014-12-29
 *      Author: zhang-lei
 */

#ifndef ARCHIVE_H_
#define ARCHIVE_H_

#include "ZTypes.h"

class DllExport CArchive
{
public:
	enum {modeLoad, modeStore};

	int   m_fd;
	int   m_nMode;
	int   m_nStatus;

public:
	CArchive(int fd, int nMode);

	unsigned int ReadStringLength(int& nCharSize);

	CArchive& operator >> (char& ch);
	CArchive& operator >> (unsigned char &ch);
	CArchive& operator >> (short& n);
	CArchive& operator >> (unsigned short& n);
	CArchive& operator >> (unsigned int& n);
	CArchive& operator >> (int& n);
	CArchive& operator >> (long& l);
	CArchive& operator >> (unsigned long& ul);
	CArchive& operator >> (float& f);
	CArchive& operator >> (char* str);

	CArchive& operator << (char ch);
	CArchive& operator << (unsigned char ch);
	CArchive& operator << (short n);
	CArchive& operator << (unsigned short n);
	CArchive& operator << (unsigned int n);
	CArchive& operator << (int n);
	CArchive& operator << (bool b);
	CArchive& operator << (long l);
	CArchive& operator << (unsigned long ul);
	CArchive& operator << (float f);
	CArchive& operator << (char* str);
};

DllExport CArchive& operator >> (CArchive& ar, bool& b);

#endif /* ARCHIVE_H_ */
