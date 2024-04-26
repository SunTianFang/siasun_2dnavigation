//                             - CAutoOutPutBlackBox.cpp-
//
//   The  class "CAutoOutPutBlackBox".
//
//   Author:
//   Date:   2020. 3 . 2
//
#pragma GCC push_options
#pragma GCC optimize ("O0")

#include "AutoOutPutBlackBox.h"
#include <BlackBox.h>
#include "LaserMapping.h"

#ifdef USE_BLACK_BOX
extern CBlackBox NavBox;       // Navigation black box
extern CBlackBox EventBox;
extern CBlackBox CanBox;
extern CBlackBox CustomBox;
extern CBlackBox LaserBox;   // The laser scanner blackbox
extern CBlackBox NetBox;
extern CBlackBox LocBox;
extern CBlackBox MsgBox;
#endif



find_suffix::find_suffix(std::string path, std::string suffix)
{
    // 读取指定路径下需要查看后缀的文件,保存到files中
    this->files.clear();
    regex reg_obj(suffix, regex::icase);

    std::vector<std::string> paths;
    paths.push_back(path);
    for(int i = 0; i < paths.size(); i++)
    {
        string curr_path = paths[i];
        DIR *dp;
        struct dirent *dirp;
        if((dp = opendir(curr_path.c_str())) == NULL)
        {
            cerr << "can not open this file." << endl;
            continue;
        }
        while((dirp = readdir(dp)) != NULL)
        {
            if(dirp->d_type == 4)
            {
                if((dirp->d_name)[0] == '.')
                    continue;
                string tmp_path = curr_path + "/" + dirp->d_name;
                paths.push_back(tmp_path);
            }
            else if(dirp->d_type == 8)
            {
                string regex_str("(\\d{4})-(\\d{2})-(\\d{2})-(\\d{2}):(\\d{2}):(\\d{2})");
                regex pattern(regex_str, regex::icase);

                if((regex_search(dirp->d_name, reg_obj)) && regex_search(dirp->d_name, pattern))
                {
                    string full_path = dirp->d_name;
                    files.push_back(full_path);
                }
            }
        }
        if (files.size() == 0)
        {
            cout<< path<<":No Find Any *BlackBox.tar.gz"<<endl;
        }
        else
        {
            sort(files.begin(), files.end());
        }
        closedir(dp);
    }
}

//提取时间戳保存到strs中,并排序
void find_suffix::find_min(){
     std::vector<long long> tmp_st;

    for(int i = 0; i < strs.size(); i++){
        string str;
        str.clear();
        for(int j=1; j<strs[i].size(); j++)
        {
            str += strs[i][j];
        }
        //str = str + to_string(i);
        //cout<<str<<endl;
        tmp_st.push_back(stoll(str));
    }
    vector<long long>::iterator smallest = min_element(tmp_st.begin(), tmp_st.end());
    min_distance = std::distance(tmp_st.begin(), smallest);
}

void find_suffix::output_resule(){
    cout<<"The earliest file is "<< files[0] << "and in the "<< 0<<" of the files vector" <<endl;
}


CAutoOutPutBlackBox::CAutoOutPutBlackBox(const int iDequeSum)
{
    m_LostTimes = 0;
    m_iRecodeSum = iDequeSum;
    m_qBlackBoxQue.clear();
    m_bNeedDelet = FALSE;
	m_bStarted = false;
}
void CAutoOutPutBlackBox::BlackBoxExport()
{
    std::cout<<"start OutPut Share Mem Data To .TXT"<<std::endl;
	#ifdef USE_BLACK_BOX
     NavBox.Save();       // Navigation black box
     EventBox.Save();
     CanBox.Save();
     CustomBox.Save();
     LaserBox.Save();   // The laser scanner blackbox
     NetBox.Save();
     LocBox.Save();
     MsgBox.Save();
	 #endif

     auto pMapping = LaserMappingSingleton::GetInstance();
     pMapping->SaveFile();
    std::cout<<"OutPut Share Mem Data To .TXT ok !"<<std::endl;
}
void CAutoOutPutBlackBox::DelBlackBox()
{
    std::cout<<"Start DelAll File"<<std::endl;

    char l_c8Command[128] = {0};
    memset(l_c8Command, 0, sizeof(l_c8Command));

    //Delete .txt
    string strCom("");
           strCom += "rm -rf ";
           strCom += LOG_FILE_PATH;
           strCom += "*";
    sprintf(l_c8Command, strCom.c_str());
    int iRet = system(l_c8Command);

    //Delete .tar.gz
    string strCom1("");
           strCom1 += "rm -rf ";
           strCom1 += LOG_TAR_PATH;
           strCom1 += "BlackBox.tar.gz";
    //sprintf(l_c8Command, "rm -rf ./Make.tar.gz");
    sprintf(l_c8Command, strCom1.c_str());
    int iRet1 = system(l_c8Command);

    if(iRet == 0 && iRet1 == 0 /*&& iRet2 == 0*/)
    {
        std::cout<<"delete ok......"<<std::endl;
   }
    else
    {
        std::cout<<"delete fail......"<<std::endl;
    }
}

string CAutoOutPutBlackBox::RetCurrentTime()
{
    char m_chText[100];
    struct tm * lpstLocalTime = GetCurrentTime();
    sprintf(m_chText, "%04d-%02d-%02d-%02d:%02d:%02d-",
            lpstLocalTime->tm_year + 1900,
            lpstLocalTime->tm_mon + 1,
            lpstLocalTime->tm_mday,
            lpstLocalTime->tm_hour,
            lpstLocalTime->tm_min,
            lpstLocalTime->tm_sec
        );
    string strRet(m_chText);

    return strRet;
}

void CAutoOutPutBlackBox::Compress(string &strNeedAddNames)
{
    std::cout<<"Start Compress File ......"<<std::endl;

    char l_c8Command[128] = {0};
    memset(l_c8Command, 0, sizeof(l_c8Command));

   string strCom("");
          strCom += "tar -zcvf ";
          strCom += LOG_TAR_PATH;
          strCom += "BlackBox.tar.gz -C ";
          strCom += LOG_FILE_PATH;
          strCom += " .";
   sprintf(l_c8Command, strCom.c_str());
   int iRet = system(l_c8Command);

    if(iRet == 0)
    {
        std::cout<<" Compress File OK...... "<<std::endl;
    }
    else
    {
        std::cout<<"Compress File Fail ...... "<<std::endl;
    }

    memset(l_c8Command, 0, sizeof(l_c8Command));
    strCom = ("");
           strCom += "cp ";
           strCom += LOG_TAR_PATH;
           strCom += "BlackBox.tar.gz ";
           strCom += WORK_PATH;
           strCom += strNeedAddNames;
           strCom += "BlackBox.tar.gz";
    sprintf(l_c8Command, strCom.c_str());
    iRet = system(l_c8Command);
    strNeedAddNames += "BlackBox.tar.gz";
    if(iRet == 0)
    {
        std::cout<<" cp BlackBox.tar.gz OK...... "<<std::endl;
    }
    else
    {
        std::cout<<"cp BlackBox.tar.gz Fail...... "<<std::endl;
    }
}
void CAutoOutPutBlackBox::BlackBoxOperation(string &strNeedAddNames)
{
    BlackBoxExport();
    Compress(strNeedAddNames);
    DelBlackBox();
}
void CAutoOutPutBlackBox::RecordBlackBox( string strAddName)
{
    string strTime = RetCurrentTime();
    strAddName=strTime;
    BlackBoxOperation(strAddName);
    m_qBlackBoxQue.push_back(strAddName);
    if(m_qBlackBoxQue.size() > m_iRecodeSum)
    {
        string strNeedDelNames = m_qBlackBoxQue.front();
        DeleteBlackBox(strNeedDelNames);
        m_qBlackBoxQue.pop_front();
    }
}
SIASUN_PTHREAD_PROC_DECL AutoOutPutBlackBoxThreadProc(LPVOID pParam)
{
    CAutoOutPutBlackBox* pManager = (CAutoOutPutBlackBox*)pParam;
    while (WaitForSingleObject(pManager->m_hKillThread, 0) != WAIT_OBJECT_0)
    {
        // 自动保存日志文件
        if(pManager->GetNeedOutPutFlag())
        {
            pManager->RecordBlackBox("Auto");
            pManager->SetNeedOutPutFlag(FALSE);
        }

        // 通过ftptool读取黑匣子触发保存激光点云数据集
        if(CBlackBox::GetReserveData() == EXPORT_BLACKBOX){
            auto pMapping = LaserMappingSingleton::GetInstance();
            pMapping->SaveFile();
            CBlackBox::SetReserveData(NO_ACTION_BLACKBOX);
            std::cout<<"Save the map point clouds by the ftptool!"<<std::endl;
        }

        Sleep(1000);
    }
    SetEvent(pManager->m_hThreadDead);
    return 0;
}
BOOL CAutoOutPutBlackBox::ReadBlackBoxFileName()
{
    std::string path(WORK_PATH);
    std::string suffix("BlackBox.tar.gz");
    find_suffix file1(path, suffix);
    file1.find_min();
    for(int i  = 0 ; i < file1.files.size(); i++ )
    {
        string strAddName = file1.files.at(i);
        m_qBlackBoxQue.push_back(strAddName);
        std::cout<<"ReadBlackBoxFileName :"<<strAddName<<std::endl;
    }

    while(m_qBlackBoxQue.size() > m_iRecodeSum)
    {
        string strNeedDelNames = m_qBlackBoxQue.front();
        DeleteBlackBox(strNeedDelNames);
        m_qBlackBoxQue.pop_front();
    }

    return TRUE;
}
BOOL CAutoOutPutBlackBox::DeleteBlackBox(const string strName)
{
    //delete file
//    {
//        char l_c8Command[128] = {0};
//        memset(l_c8Command, 0, sizeof(l_c8Command));

//        //Delete .txt
//        string strCom("");
//               strCom += "rm -rf ";
//               strCom += WORK_PATH;
//               strCom += strName;
//        sprintf(l_c8Command, strCom.c_str());
//        int iRet = system(l_c8Command);

//        if(iRet == 0)
//        {
//            std::cout<<" delete File OK...... "<<std::endl;
//        }
//        else
//        {
//            std::cout<<"delete File Fail ...... "<<std::endl;
//        }
//    }

     string strCom("");
           strCom += WORK_PATH;
           strCom += strName;
     int bRet =  remove(strCom.c_str());
     std::cout<<"delete File  :"<<strCom<<std::endl;
     if(bRet==0)
     {
        std::cout<<" delete File OK...... "<<std::endl;
     }
     else
     {
          std::cout<<"delete File Fail: errno: "<<errno<<std::endl;
     }
     return bRet;
}
void CAutoOutPutBlackBox::Start()
{
/***************** find all BlackBox.tar.gz *******************************/
    ReadBlackBoxFileName();
/**************************************************************************/
    // Init signal events
    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    ASSERT(m_hKillThread != NULL);
    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    ASSERT(m_hThreadDead != NULL);
#ifdef WINDOWS_PLATFORM_USING
    SiaSun_AfxBeginThread(AutoOutPutBlackBoxThreadProc, (LPVOID)this, THREAD_PRIORITY_NORMAL);
#elif defined(LINUX_PLATFORM_USING)
    SiaSun_AfxBeginThread(AutoOutPutBlackBoxThreadProc, (LPVOID)this, &m_AutoOutPutThread,THREAD_PRIORITY_NORMAL);
#endif

    m_bStarted = true;
}

bool CAutoOutPutBlackBox::Stop()
{
    if (!m_bStarted)
        return false;

    SetEvent(m_hKillThread);
    WaitForSingleObject(m_hThreadDead, 5000);

    if (m_hKillThread != NULL)
    {
        CloseHandle(m_hKillThread);
        m_hKillThread = NULL;
    }

    if (m_hThreadDead != NULL)
    {
        CloseHandle(m_hThreadDead);
        m_hThreadDead = NULL;
    }

    m_bStarted = false;
    return true;
}
#pragma GCC pop_options
