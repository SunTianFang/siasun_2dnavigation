//                           - TOOLS.CPP -
//
//   Implementation of some commonly used mathematical tools.
//
//   Author: Zhang Lei
//   Date:   2000. 10. 26
//

#include "stdafx.h"
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <assert.h>
#include "Tools.h"

#if defined _DEBUG && defined _MFC_VER
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define INVALID_HEX_CHAR 16    //

static unsigned char m_pchOctTab[] = "01234567";
static unsigned char m_pchHexTab[] = "0123456789ABCDEF";

//////////////////////////////////////////////////////////////////////////////

//
//   EstimateArea: Estimate the covering area of a line in cartesian space,
//   in which the start value, end value and slope are known.
//
//   [Note]:
//     The values mentioned here can only be non-negative values, i.e.,
//          fStartValue >= 0
//          fEndValue >= 0
//          fSlope > 0
//
float EstimateArea(float fStartValue, float fEndValue, float fSlope)
{
   // If the start value is equal to the end value, return 0
   if (fStartValue == fEndValue)
      return 0.0f;

   // Estimate the covering area
   return (float)(fabs((Square(fEndValue) - Square(fStartValue)) / (2 * fSlope)));
}

//
//   EstimateEndValue: Estimate the end value of a line in cartesian
//   space, in which the start value, slope and covering area are known.
//
//   [Note]:
//     1. The values here can only be non-negative values, i.e.,
//          fStartValue >= 0
//          fEndValue >= 0
//
//     2. Depending on the condition of the 2 values, the slope can be
//        positive, negative or 0:
//          fSlope > 0 (if fStartValue < fEndValue)
//          fSlope < 0 (if fStartValue > fEndValue)
//          fSlope any value (if fStartValue == fEndValue);
//
float EstimateEndValue(float fStartValue, float fArea, float fSlope)
{
   assert(fStartValue >= 0);
   assert(fArea >= 0);

   // If the slope is 0, the end value is equal to the start value
   if (fSlope == 0)
      return fStartValue;

   // Try to caculate the square of "End Value"
   float fSquEndValue = Square(fStartValue) + 2 * fSlope * fArea;

   // This value must be non-negative
   assert(fSquEndValue >= 0);

   // Get the "End Value"
   return (float)sqrt(fSquEndValue);
}

//
//   ApprEqual: Check if the 2 floats are approximately equal.
//
bool ApprEqual(float x, float y, float fGate)
{
   return (fabs(x - y) < fGate);
}

//
//   SwabWord: Swab the high/low bytes of the given word.
//
void SwabWord(unsigned short int &uWord)
{
   union
   {
      unsigned short uWord;
      unsigned char c[2];
   } DataBuf;

   DataBuf.uWord = uWord;
//   _swab((char *)DataBuf.c, (char *)DataBuf.c, 2);
	unsigned char ch = DataBuf.c[0];
	DataBuf.c[0] = DataBuf.c[1];
	DataBuf.c[1] = ch;

   uWord = DataBuf.uWord;
}

//
//   Reverse the order of bits in a word data.
//
void ReverseBitsOrder(unsigned short &uWord)
{
   unsigned short uTemp = 0;
   for (int i = 0; i < 16; i++)
   {
      if (uWord & BIT(i))
         uTemp |= 1;
      if (i != 15)
         uTemp <<= 1;
   }
   uWord = uTemp;
}

static unsigned short BitMaskTab[16] = {0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
                                        0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000};

//
//   Search for "1" in the mask byte and return the index (low to high).
//
short FindBit(unsigned short Mask, short nFromBit)
{
   for (short i = nFromBit; i < 16; i++)
      if (Mask & BitMaskTab[i])
         return i;

   return -1;
}

//
//   Generate mask byte for the specified bit.
//
unsigned short FindMaskWord(short nBit)
{
   assert(nBit < 16);

   return BitMaskTab[nBit];
}

BOOL HexStrToUSHORT(UCHAR *pchHex, USHORT& UData)
{
    // Find the index of the 1st hex char
    USHORT uHigh = LookUpHex(toupper(*pchHex++));

    // If it's not a valid hex char, return FALSE
    if (uHigh == INVALID_HEX_CHAR)
        return FALSE;

    // Find the index of the 1st hex char
    USHORT uHigh2 = LookUpHex(toupper(*pchHex++));

    // If it's not a valid hex char, return FALSE
    if (uHigh2 == INVALID_HEX_CHAR)
        return FALSE;

    // Find the index of the 2nd hex char
    USHORT uLow = LookUpHex(toupper(*pchHex++));

    // If it's not a valid hex char, return FALSE
    if (uLow == INVALID_HEX_CHAR)
        return FALSE;

    // Find the index of the 2nd hex char
    USHORT uLow2 = LookUpHex(toupper(*pchHex));

    // If it's not a valid hex char, return FALSE
    if (uLow2 == INVALID_HEX_CHAR)
        return FALSE;

    // Assemble the character
    UData = (USHORT)((uHigh<<12) | (uHigh2<<8) | (uLow<<4) | uLow2);

    // Success, return TRUE
    return TRUE;
}

#if 0
// Get the nearest integer
int round(double d)
{
	double dFloor = floor(d);
	double dCeil = ceil(d);
	if (fabs(dFloor - d) < 0.5)
		return (int)dFloor;
	else
		return (int)dCeil;
}
#endif

//
//   Look up the specified character in the hex character table.
//
//   Note:
//     On success, return the index of the character;
//     On failure, return INVALID_HEX_CHAR (16)
//
unsigned short LookUpHex(unsigned char ch)
{
   for (unsigned short i = 0; i < 16; i++)
   {
      // If the character is found, return the index
      if (m_pchHexTab[i] == ch)
         return i;
   }

   // The char does not represent a hex char, return 16
   return INVALID_HEX_CHAR;
}

////
////   Convert an ASCII character to its hex string.
////   Example:  0x2E ==> "2E"
////
void CharToHexStr(unsigned char chAscii, unsigned char *pchHex)
{
   // The higher 4 bits
   *pchHex++ = m_pchHexTab[chAscii >> 4];

   // The lower 4 bits
   *pchHex = m_pchHexTab[chAscii & 0x0F];
}

//
//   Convert an hex string into its ASCII character.
//   Example: "2E" => 0x2E
//
//   Function return:
//     false - Failure
//     true  - Success
//
bool HexStrToChar(unsigned char *pchHex, unsigned char &chAscii)
{
   // Find the index of the 1st hex char
   unsigned short uHigh = LookUpHex(toupper(*pchHex++));

   // If it's not a valid hex char, return false
   if (uHigh == INVALID_HEX_CHAR)
      return false;

   // Find the index of the 2nd hex char
   unsigned short uLow = LookUpHex(toupper(*pchHex));

   // If it's not a valid hex char, return false
   if (uLow == INVALID_HEX_CHAR)
      return false;

   // Assemble the character
   chAscii = (unsigned char)((uHigh << 4) | uLow);

   // Success, return true
   return true;
}

#if 0
//
//   Convert an ASCII character to its oct string.
//
void CharToOctStr(unsigned char chAscii, unsigned char *pchOct)
{
   _itoa(chAscii, (char *)pchOct, 8);
}
#endif

bool IsFloat(char *str, float *pFloat)
{
   char *p = str;
   while (*p == ' ')
      p++;    // Skip blanks
   if (*p == '-' || *p == '+')
      p++;    // skip sign symbol
   int nDotCount = 0;

   while (*p)
   {
      if (*p == '.')
      {
         if (++nDotCount > 1)
            return false;
      }
      else if (!isdigit(*p))
         return false;

      p++;
   }

   if (pFloat != NULL)
      *pFloat = (float)atof(str);
   return true;
}

#ifdef _MFC_VER
int HexStrToInt(CString &str)
{
   int n;

   if (_stscanf((_TCHAR *)str.GetBuffer(20), _T("%X"), &n) == 1)
      return n;
   else
      return -1;
}

bool DecStrToInt(CString &str, int &i)
{
   if (_stscanf((_TCHAR *)str.GetBuffer(20), _T("%d"), &i) == 1)
      return true;
   else
      return false;
}

bool DecStrToFloat(CString &str, float &f)
{
   if (_stscanf((_TCHAR *)str.GetBuffer(20), _T("%f"), &f) == 1)
      return true;
   else
      return false;
}

CString FloatToStr(float f)
{
   CString str;
   str.Format(_T("%0-f"), f);
   return str;
}
#else

#endif

float FabsAngleDiff(float angle1, float angle2)
{
    if (angle1 > 10000.0 || angle1 < -10000.0)
    {
        angle1 = PI/3;
    }

    if (angle2 > 10000.0 || angle2 < -10000.0)
    {
        angle2 = PI/3;
    }

    while(angle1 < 0) angle1 += 2*PI;
    while(angle1 > 2*PI) angle1 -= 2*PI;
    while(angle2 < 0) angle2 += 2*PI;
    while(angle2 > 2*PI) angle2 -= 2*PI;
    float diff = (float)fabs(angle1 - angle2);
    if(diff <= PI)
        return diff;
    else
        return 2*PI - diff;
}
