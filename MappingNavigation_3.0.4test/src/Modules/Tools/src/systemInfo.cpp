//                                  - GetSystemInfo.cpp -
//
//   The interface of class "cGetSystemInfo".
//
//   Author: sfe1012
//   Date:   2018. 10. 10
//
//   Modify: ZhangCR

#include "systemInfo.h"
#include <string.h>
#include <unistd.h>
#include <thread>

using namespace std;

SystemInfo::SystemInfo(int cpuNum, bool needRamInfo, bool needRomInfo)
    :mnCoreNum(cpuNum),mbRam(needRamInfo),mbRom(needRomInfo)
{
    mbHaltThread = false;
    mvcoreRate.clear();
    mvCpuOcpy0.clear();
    mvCpuOcpy1.clear();
    mvcoreRate.resize(cpuNum);
    mvCpuOcpy0.resize(cpuNum);
    mvCpuOcpy1.resize(cpuNum);
}
SystemInfo::~SystemInfo()
{
    if(mpthAutoRun)
    {
        mbHaltThread = true;
        mpthAutoRun->join();
    }
}
void SystemInfo::autoRun(unsigned int millisecondElapse)
{
    mpthAutoRun = new thread(&SystemInfo::updateInfo, this,millisecondElapse);
}

void SystemInfo::updateInfo(unsigned int millisecondElapse)
{
// CPU
    while(1)
    {
        if(mnCoreNum>0)
        {
            FILE *fcpu = NULL;
            char bufcpu[256];
            fcpu = fopen("/proc/stat", "r");
            if(fcpu == NULL)
                continue;
            fgets(bufcpu, sizeof(bufcpu), fcpu);
            for(int i=0;i<mnCoreNum;i++)
            {
                CPU_OCCUPY &cpu_occupy = mvCpuOcpy0[i];
                fgets(bufcpu, sizeof(bufcpu), fcpu);
                sscanf(bufcpu, "%s %u %u %u %u %u %u %u", cpu_occupy.name, &cpu_occupy.user, 
                        &cpu_occupy.nice,&cpu_occupy.system, &cpu_occupy.idle,
                        &cpu_occupy.iowait,&cpu_occupy.irq,&cpu_occupy.softirq);
            }
            if(fcpu)
            {
                fclose(fcpu);
                fcpu = NULL;
            }

            /***************** wait cpuInfo Update*******************/
            usleep(millisecondElapse*1000);
            /***************** wait cpuInfo Update*******************/
            fcpu = fopen("/proc/stat", "r");
            if(fcpu == NULL)
                continue;
            fgets(bufcpu, sizeof(bufcpu), fcpu);
            for(int i=0;i<mnCoreNum;i++)
            {
                CPU_OCCUPY &cpu_occupy = mvCpuOcpy1[i];
                fgets(bufcpu, sizeof(bufcpu), fcpu);
                sscanf(bufcpu, "%s %u %u %u %u %u %u %u", cpu_occupy.name, &cpu_occupy.user, 
                        &cpu_occupy.nice,&cpu_occupy.system, &cpu_occupy.idle,
                        &cpu_occupy.iowait,&cpu_occupy.irq,&cpu_occupy.softirq);
            }
            if(fcpu)
            {
                fclose(fcpu);
                fcpu = NULL;
            }
            // calculate
            for(int i=0;i<mnCoreNum;i++)
            {
                unique_lock<mutex> lock(mMutex);
                mvcoreRate[i] = calCpuoccupy(&mvCpuOcpy0[i], &mvCpuOcpy1[i]);
            }
        
        }
        // ROM
        if(mbRom)
        {
            FILE * fp = NULL;
            char a[80],d[80],e[80],f[80],bufrom[256];
            double b,c;
            fp=popen("df","r");
            if(fp != NULL)
            {
                fgets(bufrom,256,fp);
                double dev_total=0,dev_used=0;
                while(6==fscanf(fp,"%s %lf %lf %s %s %s",a,&b,&c,d,e,f))
                {
                    dev_total+=b;
                    dev_used+=c;
                }
                // lock
                unique_lock<mutex> lock(mMutex);
                mRomOcpy.total=dev_total/1024/1024;;
                mRomOcpy.used_rate=dev_used/dev_total*100;
            }
            if(fp)
            {
                pclose(fp);
                fp = NULL;
            }
        }
    
    // RAM
        if(mbRam)
        {
        
            FILE *fd = NULL;
            double mem_total,mem_free,mem_used_rate;;
            char buffram[256];
            MEM_OCCUPY m_stMemOccupy;
            fd = fopen ("/proc/meminfo", "r");
            if(fd != NULL)
            {
                fgets (buffram, sizeof(buffram), fd);
                sscanf (buffram, "%s %lu %s\n", m_stMemOccupy.name, &m_stMemOccupy.total, m_stMemOccupy.name2);
                mem_total=m_stMemOccupy.total;
                fgets (buffram, sizeof(buffram), fd);
                sscanf (buffram, "%s %lu %s\n", m_stMemOccupy.name, &m_stMemOccupy.total, m_stMemOccupy.name2);
                mem_free = m_stMemOccupy.total;
                mem_used_rate=(1-m_stMemOccupy.total/mem_total)*100;
                // lock
                unique_lock<mutex> lock(mMutex);
                mRamOcpy.total = mem_total;
                mRamOcpy.mem_free = mem_free;
                mRamOcpy.mem_used = mem_total - mem_free;
                mRamOcpy.used_rate= mem_used_rate;
            }
            if(fd)
            {
                fclose(fd);
                fd = NULL;
            } 
        }
        unique_lock<mutex> haltlock(mMutHalt);
        if(mbHaltThread)
            return;
    }
}

double SystemInfo::calCpuoccupy(CPU_OCCUPY *o, CPU_OCCUPY *n)
{
    double od, nd;
    double id, sd;
    double cpu_use;
    od = (double) (o->user + o->nice + o->system +o->idle+o->softirq+o->iowait+o->irq);
    nd = (double) (n->user + n->nice + n->system +n->idle+n->softirq+n->iowait+n->irq);
    id = (double) (n->idle);
    sd = (double) (o->idle);
    if((nd-od) != 0)
    cpu_use =100.0- ((id-sd))/(nd-od)*100.00;
    else cpu_use = 0;
    return cpu_use;
}

double SystemInfo::getCpuRates(int id)
{
    if(id>mnCoreNum)
        return -1;
    unique_lock<mutex> lock(mMutex);
    return mvcoreRate[id];
}

MEM_PACK SystemInfo::getMemoccupy()
{
    unique_lock<mutex> lock(mMutex);
    return mRamOcpy;
}

DEV_MEM SystemInfo::getDevmem()
{
    unique_lock<mutex> lock(mMutex);
    return mRomOcpy;
}
