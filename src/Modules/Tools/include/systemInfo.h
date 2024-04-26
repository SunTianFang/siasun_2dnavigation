#include <mutex>
#include <thread>
#include <vector>
#ifndef SYSTEMINFO_H
#define SYSTEMINFO_H

typedef struct CPU_PACKED
{
    char name[20];
    unsigned int user, nice, system, idle, iowait, irq, softirq;
}CPU_OCCUPY;

typedef struct MEM_PACKED
{
    char name[20];
    unsigned long total;
    char name2[20];
}MEM_OCCUPY;

typedef struct MEM_PACK
{
    double total,mem_free,mem_used,used_rate;
}MEM_PACK;

typedef struct DEV_MEM
{
    double total,used_rate;
}DEV_MEM;

class SystemInfo
{
public:
    SystemInfo(int cpuNum=1, bool needRamInfo=true, bool needRomInfo=true);
    ~SystemInfo();
private:
    double      calCpuoccupy(CPU_OCCUPY *o, CPU_OCCUPY *n);
    void        getCpuoccupy();
    void        updateInfo(unsigned int millisecondElapse=1000);
public:
    MEM_PACK    getMemoccupy();
    DEV_MEM     getDevmem();
    double      getCpuRates(int id=0);// multicore cpu id 0~(mnCoreNum-1)
    void        autoRun(unsigned int millisecondElapse=1000);
private:
    int         mnCoreNum;
    bool        mbRam,mbRom;
    bool        mbHaltThread;
    std::mutex  mMutex;
    std::mutex  mMutHalt;
    std::vector<double>     mvcoreRate;
    std::vector<CPU_OCCUPY> mvCpuOcpy0;
    std::vector<CPU_OCCUPY> mvCpuOcpy1;
    MEM_PACK    mRamOcpy;
    DEV_MEM     mRomOcpy;

    std::thread* mpthAutoRun;
};


// generate core file when progress dump
void dumSetting();

#endif
