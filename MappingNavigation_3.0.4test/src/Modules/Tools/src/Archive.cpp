#ifdef _MSC_VER
#include <io.h>
#else
#include <unistd.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Archive.h"

//////////////////////////////////////////////////////////////////////////////

CArchive::CArchive(int fd, int nMode)
{
	m_fd = fd;
	m_nMode = nMode;
	m_nStatus = 0;
}

CArchive& CArchive::operator >> (char& ch)
{
	if (m_nMode == modeLoad)
	{
		if (read(m_fd, &ch, 1) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}
	return *this;
}

CArchive& CArchive::operator >> (unsigned char &ch)
{
	if (m_nMode == modeLoad)
	{
		if (read(m_fd, &ch, 1) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}
	return *this;
}

CArchive& CArchive::operator >> (short& n)
{
	if (m_nMode == modeLoad)
	{
		if (read(m_fd, &n, sizeof(short)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}

	return *this;
}

CArchive& CArchive::operator >> (unsigned short& n)
{
	if (m_nMode == modeLoad)
	{
		if (read(m_fd, &n, sizeof(unsigned short)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}

	return *this;
}

CArchive& CArchive::operator >> (unsigned int& n)
{
	if (m_nMode == modeLoad)
	{
		if (read(m_fd, &n, sizeof(unsigned int)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}

	return *this;
}

CArchive& CArchive::operator >> (int& n)
{
	if (m_nMode == modeLoad)
	{
		if (read(m_fd, &n, sizeof(int)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}

	return *this;
}

CArchive& operator >> (CArchive& ar, bool& b)
{
	int n;
	ar >> n;
	b = (bool)n;

	return ar;
}

CArchive& CArchive::operator >> (long& l)
{
	if (m_nMode == modeLoad)
	{
		if (read(m_fd, &l, sizeof(long)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}

	return *this;
}

CArchive& CArchive::operator >> (unsigned long& ul)
{
	if (m_nMode == modeLoad)
	{
		if (read(m_fd, &ul, sizeof(unsigned long)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}

	return *this;
}

CArchive& CArchive::operator >> (float& f)
{
	if (m_nMode == modeLoad)
	{
		if (read(m_fd, &f, sizeof(float)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}

    return *this;
}

unsigned int CArchive::ReadStringLength(int& nCharSize)
{
	WORD wLength;
	BYTE bLength;

	nCharSize = 1;

	// First, try to read a one-byte length
	*this >> bLength;
	if (bLength < 0xff)
		return bLength;

	// Try a two-byte length
	*this >> wLength;
	if (wLength == 0xfffe)
	{
		// Unicode string.  Start over at 1-byte length
		nCharSize = 2;

		*this >> bLength;
		if (bLength < 0xff)
			return bLength;

		// Two-byte length
		*this >> wLength;
		// Fall through to continue on same branch as ANSI string
	}
	if (wLength < 0xffff)
		return wLength;

	return 0xffff;
}

CArchive& CArchive::operator >> (char* str)
{
	int nLen, nCharSize, i;
	char ch;

	nLen = ReadStringLength(nCharSize);
	for (i = 0; i < nLen; i++)
	{
		if (nCharSize == 1)
			*this >> str[i];
		else
		{
			*this >> str[i] >> ch;
		}

	}
	str[i] = '\0';

	return *this;
}

CArchive& CArchive::operator << (char ch)
{
	if (m_nMode == modeStore)
	{
		if (write(m_fd, &ch, 1) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}
	return *this;
}

CArchive& CArchive::operator << (unsigned char ch)
{
	if (m_nMode == modeStore)
	{
		if (write(m_fd, &ch, 1) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}
	return *this;
}

CArchive& CArchive:: operator << (short n)
{
	if (m_nMode == modeStore)
	{
		if (write(m_fd, &n, sizeof(short)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}
	return *this;
}

CArchive& CArchive::operator << (unsigned short n)
{
	if (m_nMode == modeStore)
	{
		if (write(m_fd, &n, sizeof(unsigned short)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}
	return *this;
}

CArchive& CArchive::operator << (unsigned int n)
{
	if (m_nMode == modeStore)
	{
		if (write(m_fd, &n, sizeof(unsigned int)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}
	return *this;
}

CArchive& CArchive:: operator << (int n)
{
	if (m_nMode == modeStore)
	{
		if (write(m_fd, &n, sizeof(int)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}
	return *this;
}

CArchive& CArchive:: operator << (bool b)
{
	if (m_nMode == modeStore)
	{
		int n = (int)b;
		if (write(m_fd, &n, sizeof(int)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}
	return *this;
}

CArchive& CArchive:: operator << (long l)
{
	if (m_nMode == modeStore)
	{
		if (write(m_fd, &l, sizeof(long)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}
	return *this;
}

CArchive& CArchive:: operator << (unsigned long ul)
{
	if (m_nMode == modeStore)
	{
		if (write(m_fd, &ul, sizeof(unsigned long)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}
	return *this;
}

CArchive& CArchive:: operator << (float f)
{
	if (m_nMode == modeStore)
	{
		if (write(m_fd, &f, sizeof(float)) < 0)
		{
			m_nStatus = -1;
		}
		else
		{
			m_nStatus = 0;
		}
	}
	return *this;
}

CArchive& CArchive::operator << (char* str)
{
	int nLen, i;
	nLen = strlen(str);

	*this << nLen;

	for (i = 0; i < nLen; i++)
		*this << str[i];

	return *this;
}

