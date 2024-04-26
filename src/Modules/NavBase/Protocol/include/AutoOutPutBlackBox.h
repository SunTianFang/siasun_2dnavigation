//                             - CAutoOutPutBlackBox.h-
//
//   The  class "CAutoOutPutBlackBox".
//
//   Author:
//   Date:   2020. 3 . 2
//

#ifndef CAUTOOUTPUTBLACKBOX_H
#define CAUTOOUTPUTBLACKBOX_H

#include "ZTypes.h"
#include "Project.h"
#include "Tools.h"
#include "MagicSingleton.h"


#include <regex>
#include <string>
#include <vector>
#include <dirent.h>
#include <iostream>
#include <string>

using namespace std;

class find_suffix
{
public:
    find_suffix(std::string path, std::string suffix);
    void find_min();
    void output_resule();
    vector<string> files;
    int min_distance;
    string m_suffix;
    vector<smatch> strs;
};

class DllExport CAutoOutPutBlackBox
{
private:
    CAutoOutPutBlackBox(const int iDequeSum = 10);
    friend MagicSingleton<CAutoOutPutBlackBox>;

    public:
    void   BlackBoxExport();
    void   DelBlackBox();
    void   Compress(string &strNeedAddNames);
    void   UpLoad();         // by lcj: Auto upload  blackbox
    string RetCurrentTime();
    void BlackBoxOperation(string &strNeedAddNames);
    void RecordBlackBox(string strAddName = "SpecialEvent");
    BOOL ReadBlackBoxFileName();
    BOOL DeleteBlackBox(const string strName);
    void Start();
    bool Stop();
    void SetNeedOutPutFlag(BOOL bSet = TRUE)
    {
        m_crit.Lock();
        m_bIsNeedOutPut = bSet;
        m_crit.Unlock();
    }
    BOOL GetNeedOutPutFlag()
    {
        BOOL bRet = FALSE;
        m_crit.Lock();
        bRet = m_bIsNeedOutPut;
        m_crit.Unlock();
        return bRet;
    }

private:
    bool    m_bStarted;
	
    public:
        CCriticalSection m_crit;
        int    m_LostTimes;
        BOOL   m_bNeedDelet;
        BOOL   m_bIsNeedOutPut;
        HANDLE m_hKillThread;       // Handle of "Kill thread" event
        HANDLE m_hThreadDead;       // Handle of "Thread dead" event
        pthread_t m_AutoOutPutThread;
        std::deque<string> m_qBlackBoxQue;
        int m_iRecodeSum;
};
/**************sfe1012 add 2020 01 20*****************/

using AutoBlackBoxSingleton = MagicSingleton<CAutoOutPutBlackBox>;

#endif // CAUTOOUTPUTBLACKBOX_H
