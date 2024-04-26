#ifndef BLACKBOX_HELPER_H
#define BLACKBOX_HELPER_H

#if __cplusplus >= 201103L // c++11
#include "BlackBox.h"
#include "Project.h"
//body
namespace QUICK_BLACK_BOX {
    static void push_back(CBlackBox& bb, char * value){
        bb << value;
    }
    static void push_back(CBlackBox& bb, const char * value){
        bb << const_cast<char*>(value);
    }
    static void push_back(CBlackBox& bb, char value){
        bb << value;
    }
    static void push_back(CBlackBox& bb, unsigned char value){
        bb << value;
    }
    static void push_back(CBlackBox& bb, short value){
        bb << static_cast<int>(value);
    }
    static void push_back(CBlackBox& bb, unsigned short value){
        bb << value;
    }
    static void push_back(CBlackBox& bb, int value){
        bb << value;
    }
    static void push_back(CBlackBox& bb, unsigned int value){
        bb << static_cast<int>(value);
    }
    static void push_back(CBlackBox& bb, long value){
#ifdef _LINUX64
        bb << (int)value;
#else
        bb << value;
#endif
    }
    static void push_back(CBlackBox& bb, unsigned long value){
#ifdef _LINUX64
        bb << (int)value;
#else
        bb << static_cast<long>(value);
#endif
    }
    static void push_back(CBlackBox& bb, float value){
        bb << value;
    }
    static void push_back(CBlackBox& bb, double value){
        bb << static_cast<float>(value);
    }
    static void push_back(CBlackBox& bb, struct tm * value){
        bb << value;
    }
    template<typename T>
    void log(CBlackBox& bb, const T& value)
    {
        push_back(bb, value);
        bb.EndRecord();
    }
    template<typename T, typename... Args>
    void log(CBlackBox& bb, T& value, const Args&... args)
    {       
        push_back(bb, value);
        log(bb,args...);
    }  

    template<typename... Args>
    void thinBlackbox(CBlackBox& bb, const Args&... args)
    {
        bb.NewRecord();
        log(bb,args...);
    }
};

//quick caller :depend blackbox, type:BYTE,char,char* , const char*, ushort, int, long, float, struct tm*
template<typename... Args>
void NV_BlackBox(CBlackBox& bb,  const Args&... args)
{
#ifdef USE_BLACK_BOX
    if(bb.m_bCreateBlackboxOK){
        if(bb.m_bUseNvramBlackbox)  QUICK_BLACK_BOX::thinBlackbox(bb,args...);
    }
#endif
}
template<typename... Args>
void FILE_BlackBox(CBlackBox& bb,  const Args&... args)
{
#ifdef USE_BLACK_BOX
     if(bb.m_bCreateBlackboxOK){
        if(!bb.m_bUseNvramBlackbox) QUICK_BLACK_BOX::thinBlackbox(bb,args...);
     }
#endif
}
template<typename... Args_nv, typename... Args_file>
void Combin_BlackBox(CBlackBox& bb,  const Args_nv&... args_nv, const Args_file&... args_file)
{
#ifdef USE_BLACK_BOX
    if(bb.m_bCreateBlackboxOK){
        if(bb.m_bUseNvramBlackbox)   QUICK_BLACK_BOX::thinBlackbox(bb,args_nv...);
        else   QUICK_BLACK_BOX::thinBlackbox(bb,args_file...);
    }
#endif
}
#endif
#endif
