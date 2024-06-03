/*****************************************
 *  class NDTMatchInfo
 *  用来记录优化过程中匹配质量
******************************************/
#pragma once

class NDTMatchInfo
{
public:
    NDTMatchInfo()
    {}

    void clear()
    {
        int lx = ly = lo = mx = my = mo = 0;
        int lkx = lky = lko = mkx = mky = mko = 0;
        int lpx = lpy = lpo = lpkx = lpky = lpko = 0;
        bool inScope = false;
        int iterCount = 0;
        int itertime = 0;  //ms
    }

    bool isOK_1(int threshCount, float threshRate)
    {
        if(inScope == false)
            return false;
//        int lxScore = lx + lo;
//        int lyScore = ly + lo;

        int lxScore = lx;
        int lyScore = ly;

        int lpxScore = lpx + lpo;
        int lpyScore = lpy + lpo;

        int mxScore = mx + mo;
        int myScore = my + mo;

//        int lkScore = lkx + lky + lko;
        int lkScore = lkx + lky;
        int mkScore = mkx + mky + mko;

//        if(lxScore < threshCount || lyScore < threshCount || mxScore < threshCount || myScore < threshCount)
        if(lpxScore < threshCount || lpyScore < threshCount || mxScore < threshCount || myScore < threshCount)
        {
//            if(lkScore < threshCount || mkScore < threshCount)
                return false;
        }
        if(lpxScore < lxScore * threshRate || lpyScore < lyScore * threshRate)
        {
//            if(mkScore < lkScore * threshRate)
                return false;
        }
        return true;
    }

    bool isOK_2(int threshCountX, int threshCountY, float threshRateX, float threshRateY)
    {
        if(inScope == false)
            return false;

        int lxScore = lx;
        int lyScore = ly;

        int lpxScore = lpx + lpo;
        int lpyScore = lpy + lpo;

        int mxScore = mx + mo;
        int myScore = my + mo;

        if(lpxScore < threshCountX || lpyScore < threshCountY || mxScore < threshCountX || myScore < threshCountY)
        {
                return false;
        }
        if(lpxScore < lxScore * threshRateX * 0.01 || lpyScore < lyScore * threshRateY * 0.01)
        {
                return false;
        }
        return true;
    }

    bool isOK_3(int threshCountX, int threshCountY, float threshRate)
    {
        if(inScope == false)
            return false;

        int lxScore = lx;
        int lyScore = ly;

        int lpxScore = lpx + lpo;
        int lpyScore = lpy + lpo;

        int mxScore = mx + mo;
        int myScore = my + mo;

        if(lpxScore < threshCountX || lpyScore < threshCountY || mxScore < threshCountX || myScore < threshCountY)
        {
                return false;
        }

        if(mxScore < lxScore * threshRate || myScore < lyScore * threshRate)
        {
                return false;
        }
        return true;
    }

public:
    // count
    int lx,ly,lo,mx,my,mo;   
    int lkx, lky, lko, mkx, mky, mko;
    int lpx, lpy, lpo, lpkx, lpky, lpko;
    // consistency Chenck
    bool inScope;
    // exe
    int iterCount;
    int iterTime;  //ms
};
